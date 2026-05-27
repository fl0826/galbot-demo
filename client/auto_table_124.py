"""
清理桌面全自动推理脚本（仅3个任务：1+2+4，跳过盖盖子）
启动后从任务1开始，按 action delta 自动判断任务完成并切换：
  pick_bag → bag_large_items → sweep_trash
全部完成后自动退出。

判定逻辑：连续 IDLE_FRAMES 次推理满足 (ARMS_max < IDLE_ARMS_THR) AND (LEG_max < IDLE_LEG_THR)
基于 aaa_20260523_*.txt 日志分析得到的阈值。
"""
import sys
import os

# 抑制 NvMMLite 等底层 C 库的 stderr 输出
_devnull_fd = os.open(os.devnull, os.O_WRONLY)
os.dup2(_devnull_fd, 2)

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from args import (
    Args,
    OBS_Head_IMG,
    OBS_PREV_STATE,
    OBS_STATE,
    OBS_Right_WRIST_IMG,
    OBS_Left_WRIST_IMG,
)
from model_agent.model_agent import ModelAgent
from galbot_control.galbot_control import GalbotControl
from tool.logger import LoggerManager
from tool.tool_shutdown import ShutdownTool
import threading
import time
import numpy as np
import copy


# ==================== 任务完成判定阈值 ====================
IDLE_ARMS_THR = 0.012   # 双臂 14 维 action delta 的最大值阈值
IDLE_LEG_THR = 0.004    # 腰部 5 维 action delta 的最大值阈值
IDLE_FRAMES = 30        # 连续多少次推理满足才判定为任务完成（约 3 秒）


# ==================== 任务序列 ====================
TASK_LIST = [
    {
        "name": "pick_bag",
        "label": "升降取垃圾袋",
        "task": "Pick up the bag and place it on the table.",
        "need_init_pose": True,
    },
    {
        "name": "bag_large_items",
        "label": "桌面物品清理",
        "task": "Put the large objects on the table into the bag.",
        "need_init_pose": False,
    },
    {
        "name": "sweep_trash",
        "label": "抹布清理",
        "task": "Sweep the remaining trash on the table into the white basin, then put it into the bag.",
        "need_init_pose": False,
    },
]


class GalbotVLAAuto:
    def __init__(self, args: Args):
        self.args = copy.deepcopy(args)

        self.model_agent = []
        self.flag_infer_thread_is_run = []
        self.flag_model_first_infer = []
        self.last_action_time_delay = []
        for i in range(len(args.host)):
            self.model_agent.append(
                ModelAgent(args.host[i], args.port[i], args.action_horizon)
            )
            self.flag_infer_thread_is_run.append(0)
            self.flag_model_first_infer.append(True)
            self.last_action_time_delay.append(0)
            self.model_agent[i].ws_client.network_latency_tolerance = (
                self.args.network_latency_tolerance
            )

        self.galbot = GalbotControl(args)
        self.flag_has_moved_to_init_pose = False
        self.grasp_count = 0
        self.shutdown_event = threading.Event()
        self.vla_model_error = False
        self.logger = LoggerManager.get_logger()
        self.vla_cost_time = 0
        self.lock_exit = threading.Lock()

        # 任务完成检测计数器
        self._idle_count = 0
        self._task_done = False  # 标记是因为任务完成而退出（区别于错误退出）

    def _check_idle(self, actions):
        """每次推理后调用：判断 action 是否处于"任务完成"状态。
        actions: shape (action_horizon, >=23) 的 numpy array (delta action)
        """
        if actions is None:
            return
        a = np.asarray(actions)
        if a.ndim != 2 or a.shape[1] < 23:
            return

        arms_max = float(np.abs(a[:, 7:14]).max())  # 左臂 7 维
        rarms_max = float(np.abs(a[:, 15:22]).max())  # 右臂 7 维
        arms_max = max(arms_max, rarms_max)
        leg_max = float(np.abs(a[:, 0:5]).max())

        if arms_max < IDLE_ARMS_THR and leg_max < IDLE_LEG_THR:
            self._idle_count += 1
            if self._idle_count >= IDLE_FRAMES:
                self.logger.info(
                    f"[auto_done] idle {self._idle_count} frames "
                    f"(arms_max<{IDLE_ARMS_THR}, leg_max<{IDLE_LEG_THR}), task done"
                )
                self._task_done = True
                # 关键：设 flag=-1 让 infer_thread 真正退出循环（仅设 shutdown_event 不够）
                with self.lock_exit:
                    for i in range(len(self.flag_infer_thread_is_run)):
                        if self.flag_infer_thread_is_run[i] == 1:
                            self.flag_infer_thread_is_run[i] = -1
                self.shutdown_event.set()
        else:
            self._idle_count = 0

    def run(self, args: Args):
        self.logger.info("in_Vla")

        self.args = copy.deepcopy(args)
        self.galbot.args = self.args
        self.galbot.galbot_interface.args = self.args

        if self.args.has_init_action:
            self.flag_has_moved_to_init_pose = False
        else:
            self.flag_has_moved_to_init_pose = True
        self.grasp_count = 0
        self.galbot.shutdown_event.clear()
        self.galbot.error_imformation = ""
        self.shutdown_event.clear()
        self.vla_model_error = False
        self.vla_cost_time = 0
        # 重置任务完成检测
        self._idle_count = 0
        self._task_done = False
        for i in range(len(self.args.host)):
            self.flag_infer_thread_is_run[i] = 0
            self.flag_model_first_infer[i] = True
            self.last_action_time_delay[i] = 0

        if self.args.blocking:
            self.blocking_mode()
        else:
            self.nonblocking_mode()

        # 等待 infer_thread 真正退出（避免下一轮启动时 ws 并发 recv 冲突）
        if not self.args.blocking:
            t_wait = time.perf_counter()
            while True:
                if all(f == 0 for f in self.flag_infer_thread_is_run):
                    break
                if time.perf_counter() - t_wait > 5:
                    self.logger.warning("infer_thread did not exit in 5s, force closing ws")
                    # 强制关闭 ws 连接，让阻塞的 recv 立即返回
                    for agent in self.model_agent:
                        try:
                            agent.ws_client.conn.close()
                        except Exception:
                            pass
                    # 再等一会儿让线程退出
                    time.sleep(0.5)
                    break
                time.sleep(0.05)

        time.sleep(0.1)
        self.galbot.args.has_init_action = False

        t0 = time.perf_counter()
        while True:
            res = self.galbot.request_vla_service("stop")
            self.logger.info("exit vla request_vla_service('stop'): " + res)
            if res == "stop":
                break
            else:
                time.sleep(0.1)
            if (
                time.perf_counter() - t0
                > self.args.action_horizon_use * self.args.dt_model_control + 1
            ):
                break

        self.logger.info("out_Vla")
        if self._task_done:
            return True, "auto_done"
        if self.galbot.error_imformation == "":
            return True, "success"
        else:
            return False, self.galbot.error_imformation

    def blocking_mode(self):
        while not self.shutdown_event.is_set():
            if not self.flag_has_moved_to_init_pose:
                self.galbot.move_to_init_pose_wholebody()
                self.flag_has_moved_to_init_pose = True

            obs, obs_visualization = self.galbot.get_obs_wholebody_compressed(
                self.flag_model_first_infer[0], 0
            )
            if self.flag_model_first_infer[0]:
                self.flag_model_first_infer[0] = False

            try:
                response = self.model_agent[0].infer(obs)
                # 自动判断任务是否完成
                if "actions" in response:
                    self._check_idle(response["actions"])
            except Exception as e:
                error_msg = f"vla model error : {type(e).__name__}: {str(e)}"
                self.logger.error(error_msg)
                self.galbot.error_imformation = (
                    self.galbot.error_imformation + error_msg + ", "
                )
                self.vla_model_error = True

            if self.shutdown_event.is_set():
                break
            if not self.vla_model_error:
                self.galbot.set_wholebody_command(obs, obs_visualization, response)

            if self.galbot.error_imformation != "":
                self.logger.error(self.galbot.error_imformation)
                self.shutdown_event.set()
                self.galbot.shutdown()
                break

    def nonblocking_mode(self):
        while not self.shutdown_event.is_set():
            if (not self.flag_has_moved_to_init_pose) and sum(
                self.flag_infer_thread_is_run
            ) == 0:
                self.galbot.move_to_init_pose_wholebody()
                self.vla_cost_time = 0
                self.flag_has_moved_to_init_pose = True

                for i in range(len(self.args.host)):
                    self.flag_model_first_infer[i] = True
                    self.last_action_time_delay[i] = 0

            for i in range(len(self.args.host)):
                if (
                    self.flag_infer_thread_is_run[i] == 0
                    and self.flag_has_moved_to_init_pose
                    and self.galbot.error_imformation == ""
                ):
                    self.flag_infer_thread_is_run[i] = 1
                    thread_ = threading.Thread(target=self.infer_thread, args=(i,))
                    thread_.start()

            if self.galbot.error_imformation != "":
                self.logger.error(self.galbot.error_imformation)
                with self.lock_exit:
                    if sum(self.flag_infer_thread_is_run) == len(self.args.host):
                        for i in range(len(self.args.host)):
                            self.flag_infer_thread_is_run[i] = -1
                self.shutdown_event.set()
                self.galbot.shutdown()
                break

            time.sleep(0.1)
            self.vla_cost_time = self.vla_cost_time + 0.1

    def infer_thread(self, i):
        while self.flag_infer_thread_is_run[i] == 1:
            obs, obs_visualization = self.galbot.get_obs_wholebody_compressed(
                self.flag_model_first_infer[i],
                round(
                    self.last_action_time_delay[i]
                    * 1000
                    * (0.1 / self.args.dt_model_control)
                ),
            )
            if obs is None:
                time.sleep(0.1)
                continue

            try:
                response = self.model_agent[i].infer(obs)
                # 自动判断任务是否完成
                if "actions" in response:
                    self._check_idle(response["actions"])
            except Exception as e:
                error_msg = (
                    f"vla model error (thread {i}): {type(e).__name__}: {str(e)}"
                )
                self.logger.error(error_msg)
                self.galbot.error_imformation = (
                    self.galbot.error_imformation + error_msg + ", "
                )
                self.vla_model_error = True
                response = {}

            with self.lock_exit:
                action_time_delay = (
                    time.time() - obs_visualization[OBS_STATE + "_timestamp"]
                )
                if self.flag_infer_thread_is_run[i] == 1 and (not self.vla_model_error):
                    if self.flag_model_first_infer[i]:
                        self.galbot.pub_wholebody_command(
                            obs, obs_visualization, response, 0
                        )
                    else:
                        self.galbot.pub_wholebody_command(
                            obs, obs_visualization, response, action_time_delay
                        )

            if self.flag_model_first_infer[i]:
                self.flag_model_first_infer[i] = False
            self.last_action_time_delay[i] = action_time_delay

            if ("infer_time" not in response) or self.vla_model_error:
                response["infer_time"] = 0

        time.sleep(0.1)
        self.flag_infer_thread_is_run[i] = 0

    def shutdown(self):
        with self.lock_exit:
            if (not self.args.blocking) and sum(self.flag_infer_thread_is_run) == len(
                self.args.host
            ):
                for i in range(len(self.args.host)):
                    self.flag_infer_thread_is_run[i] = -1
        self.shutdown_event.set()

        t0 = time.perf_counter()
        while True:
            res = self.galbot.request_vla_service("stop")
            if res == "stop":
                break
            else:
                time.sleep(0.1)
            if (
                time.perf_counter() - t0
                > self.args.action_horizon_use * self.args.dt_model_control + 1
            ):
                break


def auto_run_all(vla: GalbotVLAAuto, args: Args):
    """依次执行 TASK_LIST 中所有任务，自动切换"""
    print("\n" + "=" * 60)
    print("清理桌面全自动推理（跳过盖盖子）")
    print(f"任务序列: " + " → ".join(t["label"] for t in TASK_LIST))
    print(f"判定阈值: ARMS<{IDLE_ARMS_THR}, LEG<{IDLE_LEG_THR}, 连续{IDLE_FRAMES}次")
    print("Ctrl+C 中断")
    print("=" * 60 + "\n")

    for idx, task_config in enumerate(TASK_LIST, start=1):
        print(f"[{idx}/{len(TASK_LIST)}] >>> 当前任务: {task_config['label']} ({task_config['name']})")
        args.task = task_config["task"]
        args.has_init_action = task_config["need_init_pose"]

        # 切任务前确保上一轮彻底停干净（参考手动 table.py 的做法）
        vla.shutdown()
        time.sleep(0.3)

        try:
            success, msg = vla.run(args)
        except KeyboardInterrupt:
            print("\n[中断] 用户终止")
            vla.shutdown()
            return
        except Exception as e:
            print(f"\n[异常] {e}")
            vla.shutdown()
            return

        print(f"[{idx}/{len(TASK_LIST)}] 完成: {task_config['label']} - {msg}")

        if not success:
            print(f"[失败] 中止后续任务")
            return

        time.sleep(0.5)

    print("\n" + "=" * 60)
    print("全部任务执行完毕，自动退出")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    import logging

    # 抑制推理过程中的轨迹日志终端输出，只显示 ERROR 以上
    _logger = LoggerManager.get_logger()
    for listener in LoggerManager._queue_listeners.values():
        for handler in listener.handlers:
            if isinstance(handler, logging.StreamHandler) and not isinstance(handler, logging.FileHandler):
                handler.setLevel(logging.ERROR)

    parser = argparse.ArgumentParser(description="清理桌面全自动推理（跳过盖盖子）")
    parser.add_argument("--model-host", default=None, help="模型服务器IP（不传则用args.py中的配置）")
    cli_args = parser.parse_args()

    args = Args()
    if cli_args.model_host is not None:
        args.host = [cli_args.model_host]
    # 桌面专用参数（写死，不再依赖 args.py 默认值）
    args.port = [6686]
    args.init_pose_file = "config/init_pose/zhiyuan_pick_trash_stand.json"
    args.raw_image_size_left_arm = [1280, 720]   # 工作模式
    args.raw_image_size_right_arm = [1280, 720]  # 工作模式
    args.task = TASK_LIST[0]["task"]

    vla = GalbotVLAAuto(args)

    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(vla.shutdown)
    tool_shutdown.on_shutdown(vla.galbot.shutdown)
    for i in range(len(args.host)):
        tool_shutdown.on_shutdown(vla.model_agent[i].ws_client.conn.close)

    time.sleep(3)

    # stderr 保持重定向到 /dev/null，抑制 NvMMLite 等底层 C 库的输出
    # （Python 的 print 走 stdout 不受影响，logger 的 ERROR 也走 stdout）

    auto_run_all(vla, args)
