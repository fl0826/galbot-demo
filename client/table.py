"""
清理桌面交互式推理脚本
启动后在终端输入数字切换任务，无需重启：
  1 - 升降取垃圾袋
  2 - 桌面物品清理
  3 - 盖盖子
  4 - 抹布清理
  s - 停止当前推理
  q - 退出程序
"""

import sys
import os
import ctypes

# 抑制 NvMMLite 等底层 C 库的 stdout/stderr 输出
_devnull_fd = os.open(os.devnull, os.O_WRONLY)
_old_stderr_fd = os.dup(2)
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
from tool.tool_rate import ToolRate
import threading
import time
import numpy as np
import copy

# ==================== 任务定义 ====================
TASKS = {
    "1": {
        "name": "升降取垃圾袋",
        "task": "Pick up the bag and place it on the table.",
        "need_init_pose": True,
    },
    "2": {
        "name": "桌面物品清理",
        "task": "Put the large objects on the table into the bag.",
        "need_init_pose": False,
    },
    "3": {
        "name": "盖盖子",
        "task": "Close the box and put it into the bag.",
        "need_init_pose": False,
    },
    "4": {
        "name": "抹布清理",
        "task": "Sweep the remaining trash on the table into the white basin, then put it into the bag.",
        "need_init_pose": False,
    },
}


class GalbotVLAClearTable:
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
        for i in range(len(self.args.host)):
            self.flag_infer_thread_is_run[i] = 0
            self.flag_model_first_infer[i] = True
            self.last_action_time_delay[i] = 0

        if self.args.blocking:
            self.blocking_mode()
        else:
            self.nonblocking_mode()

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
                self.logger.info("exit vla request_vla_service('stop') time out stop ")
                break

        self.logger.info("out_Vla")
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

            if (
                self.check_stop()
                and self.args.auto_stop
                and self.flag_has_moved_to_init_pose
                and sum(self.flag_infer_thread_is_run) == len(self.args.host)
                and self.galbot.error_imformation == ""
            ):
                with self.lock_exit:
                    for i in range(len(self.args.host)):
                        self.flag_infer_thread_is_run[i] = -1

                q_wholebody = self.galbot.galbot_interface.pose_buffer[1]
                if q_wholebody[22] > self.args.success_gripper_width:
                    self.logger.info(
                        "success, gripper width is "
                        + str(q_wholebody[22])
                        + "m > "
                        + str(self.args.success_gripper_width)
                        + "m"
                    )
                    self.shutdown_event.set()
                    self.galbot.shutdown()
                    break
                elif self.args.allow_retry:
                    self.logger.error(
                        "start retry, right gripper width is "
                        + str(q_wholebody[22])
                        + "m <="
                        + str(self.args.success_gripper_width)
                        + "m"
                    )
                    self.flag_has_moved_to_init_pose = False
                    self.galbot.args.has_init_action = True
                    self.grasp_count = self.grasp_count + 1
                    if self.grasp_count >= self.args.retry_fail_max_num:
                        self.logger.error(
                            f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                        self.galbot.error_imformation = (
                            self.galbot.error_imformation
                            + f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                else:
                    self.galbot.error_imformation = (
                        self.galbot.error_imformation
                        + "failed, right gripper width is "
                        + str(q_wholebody[22])
                        + "m <="
                        + str(self.args.success_gripper_width)
                        + "m"
                    )
                    self.logger.error(
                        "failed, right gripper width is "
                        + str(q_wholebody[22])
                        + "m <="
                        + str(self.args.success_gripper_width)
                        + "m"
                    )

            if self.args.auto_stop and self.vla_cost_time > self.args.vla_max_cost_time:
                self.vla_cost_time = 0
                with self.lock_exit:
                    for i in range(len(self.args.host)):
                        self.flag_infer_thread_is_run[i] = -1

                if self.args.allow_retry:
                    self.logger.error(
                        "start retry, "
                        + f"vla_cost_time more than {self.args.vla_max_cost_time}s"
                    )
                    self.flag_has_moved_to_init_pose = False
                    self.galbot.args.has_init_action = True
                    self.grasp_count = self.grasp_count + 1
                    if self.grasp_count >= self.args.retry_fail_max_num:
                        self.logger.error(
                            f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                        self.galbot.error_imformation = (
                            self.galbot.error_imformation
                            + f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                else:
                    self.galbot.error_imformation = (
                        self.galbot.error_imformation
                        + f"vla_cost_time more than {self.args.vla_max_cost_time}s"
                    )

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
        self.logger.info(f"推理线程{i}退出")

    def check_stop(self):
        if self.galbot.error_imformation != "":
            return False

        dis = self.args.auto_stop_distance
        left_end_T = self.galbot.galbot_interface.left_end_T
        left_end_T_deepest = self.galbot.galbot_interface.left_end_T_deepest
        right_end_T = self.galbot.galbot_interface.right_end_T
        right_end_T_deepest = self.galbot.galbot_interface.right_end_T_deepest

        if len(self.args.object_name) == 1:
            if (
                np.linalg.norm(right_end_T[0:3, 3:4] - right_end_T_deepest[0:3, 3:4])
                > dis
            ) or (
                np.linalg.norm(left_end_T[0:3, 3:4] - left_end_T_deepest[0:3, 3:4])
                > dis
            ):
                return True
            else:
                return False
        elif len(self.args.object_name) == 2:
            if (
                np.linalg.norm(right_end_T[0:3, 3:4] - right_end_T_deepest[0:3, 3:4])
                > dis
            ) and (
                np.linalg.norm(left_end_T[0:3, 3:4] - left_end_T_deepest[0:3, 3:4])
                > dis
            ):
                return True
            else:
                return False
        else:
            return False

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
            self.logger.info("shutdown exit vla request_vla_service('stop'): " + res)
            if res == "stop":
                break
            else:
                time.sleep(0.1)
            if (
                time.perf_counter() - t0
                > self.args.action_horizon_use * self.args.dt_model_control + 1
            ):
                self.logger.info(
                    "shutdown exit vla request_vla_service('stop') time out stop "
                )
                break


# ==================== 键盘监听与任务调度 ====================
def keyboard_listener(vla: GalbotVLAClearTable, args: Args):
    """在终端监听键盘输入，切换任务"""
    logger = LoggerManager.get_logger()

    print("\n" + "=" * 60)
    print("清理桌面交互式推理 - 输入数字切换任务")
    print("  1 - 升降取垃圾袋   (含复位)")
    print("  2 - 桌面物品清理   (不复位)")
    print("  3 - 盖盖子         (不复位)")
    print("  4 - 抹布清理       (不复位)")
    print("  5 - 仅复位         (只复位不推理)")
    print("  s - 停止           q - 退出")
    print("=" * 60 + "\n")

    task_thread = None

    while True:
        try:
            cmd = input(">>> 输入指令: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            cmd = "q"

        if cmd == "q":
            vla.shutdown()
            if task_thread and task_thread.is_alive():
                task_thread.join(timeout=5)
            break

        elif cmd == "s":
            print("已停止")
            vla.shutdown()
            if task_thread and task_thread.is_alive():
                task_thread.join(timeout=5)

        elif cmd in TASKS:
            task_config = TASKS[cmd]
            print(f"切换到任务{cmd}: {task_config['name']}")

            # 先停止当前推理
            vla.shutdown()
            if task_thread and task_thread.is_alive():
                task_thread.join(timeout=5)
            time.sleep(0.3)

            # 设置新任务参数
            args.task = task_config["task"]
            args.has_init_action = task_config["need_init_pose"]

            # 启动新推理
            def run_task(a):
                success, msg = vla.run(a)
                logger.info(f"任务结束: success={success}, msg={msg}")
                print(f"任务{cmd}结束: {'成功' if success else '失败'} - {msg}")

            task_thread = threading.Thread(
                target=run_task, args=(copy.deepcopy(args),), daemon=True
            )
            task_thread.start()

        elif cmd == "5":
            print("执行复位...")
            # 先停止当前推理
            vla.shutdown()
            if task_thread and task_thread.is_alive():
                task_thread.join(timeout=5)
            time.sleep(0.3)

            # 只复位不推理
            args.has_init_action = True
            vla.args = copy.deepcopy(args)
            vla.galbot.args = vla.args
            vla.galbot.galbot_interface.args = vla.args
            vla.galbot.move_to_init_pose_wholebody()
            print("复位完成")

        else:
            pass


if __name__ == "__main__":
    import argparse
    import logging

    # 抑制推理过程中的轨迹日志终端输出，只显示 ERROR 以上
    # LoggerManager 使用 QueueListener，需要修改 console_handler 的级别
    _logger = LoggerManager.get_logger()
    for listener in LoggerManager._queue_listeners.values():
        for handler in listener.handlers:
            if isinstance(handler, logging.StreamHandler) and not isinstance(
                handler, logging.FileHandler
            ):
                handler.setLevel(logging.ERROR)

    parser = argparse.ArgumentParser(description="清理桌面交互式推理")
    parser.add_argument(
        "--model-host", default=None, help="模型服务器IP（不传则用args.py中的配置）"
    )
    parser.add_argument(
        "--model-port",
        type=int,
        default=None,
        help="模型端口（不传则用args.py中的配置）",
    )
    cli_args = parser.parse_args()

    args = Args()
    if cli_args.model_host is not None:
        args.host = [cli_args.model_host]
    if cli_args.model_port is not None:
        args.port = [cli_args.model_port]
    args.task = TASKS["1"]["task"]  # 默认任务1
    args.init_pose_file = "config/init_pose/zhiyuan_pick_trash_stand.json"

    vla = GalbotVLAClearTable(args)

    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(vla.shutdown)
    tool_shutdown.on_shutdown(vla.galbot.shutdown)
    for i in range(len(args.host)):
        tool_shutdown.on_shutdown(vla.model_agent[i].ws_client.conn.close)

    time.sleep(3)

    # 恢复 stderr，让 Python 的 print/error 正常输出
    os.dup2(_old_stderr_fd, 2)
    os.close(_devnull_fd)

    keyboard_listener(vla, args)
