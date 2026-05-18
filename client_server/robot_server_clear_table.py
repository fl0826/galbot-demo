import sys
import os

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
from tool.vla_profiler import profile_timeline, set_profile_enabled
import sys
import threading
import time
import numpy as np
import copy
from flask import Flask, jsonify
import argparse


class GalbotVLA:
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
        self.flag_start_listen_keyboard = True
        self.grasp_count = 0
        self.shutdown_event = threading.Event()
        self.vla_model_error = False
        self.logger = LoggerManager.get_logger()
        self.vla_cost_time = 0
        self.lock_exit = threading.Lock()

    # @profile_timeline(cat="vla", name="GalbotVLA.run", min_duration_ms=0.1)
    def run(self, args: Args):
        self.logger.info("in_Vla")

        self.args = copy.deepcopy(args)
        self.galbot.args = self.args
        self.galbot.galbot_interface.args = self.args

        # 如果不需要复位，直接标记为已完成，跳过 move_to_init_pose
        if self.args.has_init_action:
            self.flag_has_moved_to_init_pose = False
        else:
            self.flag_has_moved_to_init_pose = True
        self.grasp_count = 0
        self.galbot.shutdown_event.clear()
        self.galbot.error_imformation = ""
        # self.galbot.galbot_interface.last_chassis_pos = None
        self.shutdown_event.clear()
        self.vla_model_error = False
        self.vla_cost_time = 0
        for i in range(len(self.args.host)):
            self.flag_infer_thread_is_run[i] = 0
            self.flag_model_first_infer[i] = True
            self.last_action_time_delay[i] = 0

        # if self.change_control_mode():
        #     if self.args.vla_type == "VLA_training_time_RTC":
        #         self.blocking_mode_training_time_RTC()
        #     elif self.args.vla_type == "VLA":
        if self.args.blocking:
            self.blocking_mode()
        else:
            self.nonblocking_mode()

        # return

        time.sleep(
            0.1
        )  # 有时候存在在同一个singorix周期里，同时有service和topic消息，有时候会先触发service后topic，导致错误，故这里sleep(0.1)，后期底层修改后可删
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

    # @profile_timeline(cat="vla", name="GalbotVLA.blocking_mode_training_time_RTC", min_duration_ms=0.1)
    def blocking_mode_training_time_RTC(self):
        if not self.flag_has_moved_to_init_pose:
            self.galbot.move_to_init_pose_wholebody()
            self.flag_has_moved_to_init_pose = True

        response = None
        obs, obs_visualization = self.galbot.get_obs_wholebody_compressed(
            self.flag_model_first_infer[0], 0, response
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
            return
        self.galbot.pub_wholebody_command(obs, obs_visualization, response, 0)

        rate = ToolRate(
            1.0 / (self.args.dt_model_control * self.args.action_horizon_use)
        )
        while not self.shutdown_event.is_set():
            obs, obs_visualization = self.galbot.get_obs_wholebody_compressed(
                self.flag_model_first_infer[0], 0, response
            )
            try:
                response = self.model_agent[0].infer(obs)
            except Exception as e:
                error_msg = f"vla model error : {type(e).__name__}: {str(e)}"
                self.logger.error(error_msg)
                self.galbot.error_imformation = (
                    self.galbot.error_imformation + error_msg + ", "
                )
                self.vla_model_error = True
                break
            if not self.vla_model_error:
                self.galbot.pub_wholebody_command(obs, obs_visualization, response, 0)
            rate.sleep()

    # @profile_timeline(cat="vla", name="GalbotVLA.blocking_mode", min_duration_ms=0.1)
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

    # @profile_timeline(cat="vla", name="GalbotVLA.nonblocking_mode", min_duration_ms=0.1)
    def nonblocking_mode(self):
        while not self.shutdown_event.is_set():
            if (not self.flag_has_moved_to_init_pose) and sum(
                self.flag_infer_thread_is_run
            ) == 0:  # 如果没初始化，且没有模型在跑，走初始化
                self.galbot.move_to_init_pose_wholebody()
                self.vla_cost_time = 0
                self.flag_has_moved_to_init_pose = True

                for i in range(len(self.args.host)):
                    self.flag_model_first_infer[i] = True
                    self.last_action_time_delay[i] = 0
                # 等待相机图像就绪
                self.logger.info("waiting for camera images to be ready...")
                while not self.shutdown_event.is_set():
                    head_ok = self.galbot.galbot_interface._image_head_left is not None
                    left_ok = self.galbot.galbot_interface._image_hand_left is not None
                    right_ok = (
                        self.galbot.galbot_interface._image_hand_right is not None
                    )
                    if head_ok and left_ok and right_ok:
                        self.logger.info("all camera images are ready")
                        break
                    self.logger.info(
                        f"waiting images... head:{head_ok} hand_left:{left_ok} hand_right:{right_ok}"
                    )
                    time.sleep(1.0)

            for i in range(len(self.args.host)):
                if (
                    self.flag_infer_thread_is_run[i] == 0
                    and self.flag_has_moved_to_init_pose
                    and self.galbot.error_imformation == ""
                ):  # 初始化后，启动模型
                    self.flag_infer_thread_is_run[i] = 1
                    thread_ = threading.Thread(target=self.infer_thread, args=(i,))
                    thread_.start()

            if (
                self.check_stop()
                and self.args.auto_stop
                and self.flag_has_moved_to_init_pose
                and sum(self.flag_infer_thread_is_run) == len(self.args.host)
                and self.galbot.error_imformation == ""
            ):  # 回退到位，检测是否成功
                with self.lock_exit:
                    for i in range(len(self.args.host)):  # 停止推理线程
                        self.flag_infer_thread_is_run[i] = (
                            -1
                        )  # -1表示退出  0表示未运行  1表示正在运行

                q_wholebody = self.galbot.galbot_interface.pose_buffer[1]
                if (
                    q_wholebody[22] > self.args.success_gripper_width
                ):  # 检测到有物体，成功并退出
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
                elif self.args.allow_retry:  # 未检测到物体，允许重试时，进行重试
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
                    if (
                        self.grasp_count >= self.args.retry_fail_max_num
                    ):  # 超过最大重试次数，失败并退出
                        self.logger.error(
                            f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                        self.galbot.error_imformation = (
                            self.galbot.error_imformation
                            + f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                else:  # 未检测到物体，不允许重试，失败并退出
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

            if (
                self.args.auto_stop and self.vla_cost_time > self.args.vla_max_cost_time
            ):  # 执行超时
                self.vla_cost_time = 0
                with self.lock_exit:
                    for i in range(len(self.args.host)):  # 停止推理线程
                        self.flag_infer_thread_is_run[i] = (
                            -1
                        )  # -1表示退出  0表示未运行  1表示正在运行

                if self.args.allow_retry:  # 超时，允许重试时，进行重试
                    self.logger.error(
                        "start retry, "
                        + f"vla_cost_time more than {self.args.vla_max_cost_time}s"
                    )
                    self.flag_has_moved_to_init_pose = False
                    self.galbot.args.has_init_action = True
                    self.grasp_count = self.grasp_count + 1
                    if (
                        self.grasp_count >= self.args.retry_fail_max_num
                    ):  # 超过最大重试次数，失败并退出
                        self.logger.error(
                            f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                        self.galbot.error_imformation = (
                            self.galbot.error_imformation
                            + f"retry exceeded the limit {self.args.retry_fail_max_num} times, "
                        )
                else:  # 不允许重试，失败并退出
                    self.galbot.error_imformation = (
                        self.galbot.error_imformation
                        + f"vla_cost_time more than {self.args.vla_max_cost_time}s"
                    )

            if self.galbot.error_imformation != "":  # 有错误信息，退出
                self.logger.error(self.galbot.error_imformation)
                with self.lock_exit:
                    if sum(self.flag_infer_thread_is_run) == len(self.args.host):
                        for i in range(len(self.args.host)):  # 停止推理线程
                            self.flag_infer_thread_is_run[i] = (
                                -1
                            )  # -1表示退出  0表示未运行  1表示正在运行

                self.shutdown_event.set()
                self.galbot.shutdown()
                break

            time.sleep(0.1)
            self.vla_cost_time = self.vla_cost_time + 0.1

    # @profile_timeline(cat="vla", name="GalbotVLA.infer_thread", min_duration_ms=0.1)
    def infer_thread(self, i):
        while self.flag_infer_thread_is_run[i] == 1:

            t_obs_start = time.perf_counter()
            obs, obs_visualization = self.galbot.get_obs_wholebody_compressed(
                self.flag_model_first_infer[i],
                round(
                    self.last_action_time_delay[i]
                    * 1000
                    * (0.1 / self.args.dt_model_control)
                ),
            )
            t_obs_end = time.perf_counter()
            if obs is None:
                time.sleep(0.1)
                continue

            t_model_start = time.perf_counter()
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
            t_model_end = time.perf_counter()

            t_pub_start = time.perf_counter()
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
            t_pub_end = time.perf_counter()

            if self.flag_model_first_infer[i]:
                self.flag_model_first_infer[i] = False
            self.last_action_time_delay[i] = action_time_delay

            if ("infer_time" not in response) or self.vla_model_error:
                response["infer_time"] = 0
            # self.logger.info(
            #     f"\n get_obs_cost = {(t_obs_end - t_obs_start)*1000}ms, nv crop and resize cost = {self.galbot.img_process.get_cal_time()}ms"
            #     + f"\n model cost {(t_model_end - t_model_start)*1000}ms = model infer time {response['infer_time']}ms + internet_latency {(t_model_end - t_model_start)*1000 - response['infer_time']}ms"
            #     + f"\n pub command cost = {(t_pub_end - t_pub_start)*1000}ms"
            #     + f"\n total cost = {(t_pub_end - t_obs_start)*1000}ms"
            #     + f"\n action_time_delay = {action_time_delay*1000}ms"
            #     + f"\n sensor callback frequency = {self.galbot.galbot_interface.callback_frequency_sensor.frequency}"
            #     + f"\n odom callback frequency = {self.galbot.galbot_interface.callback_frequency_odom.frequency}"
            #     + f"\n image_head_left callback frequency = {self.galbot.galbot_interface.callback_frequency_image_head_left.frequency}"
            #     + f"\n image_hand_left callback frequency = {self.galbot.galbot_interface.callback_frequency_image_hand_left.frequency}"
            #     + f"\n image_hand_right callback frequency = {self.galbot.galbot_interface.callback_frequency_image_hand_right.frequency}"
            # )

        time.sleep(0.1)
        self.flag_infer_thread_is_run[i] = 0
        self.logger.info(f"推理线程{i}退出")

    # @profile_timeline(cat="vla", name="GalbotVLA.check_stop", min_duration_ms=0.02)
    def check_stop(self):
        if self.galbot.error_imformation != "":
            return

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

    # @profile_timeline(cat="vla", name="GalbotVLA.change_control_mode", min_duration_ms=0.02)
    def change_control_mode(self):
        change_success = False
        if self.args.vla_type == "VLA_training_time_RTC":
            str_req = "plan_mode=PlanJoint_Vla_TrainingRTC"
        elif self.args.vla_type == "VLA":
            str_req = "plan_mode=PlanJoint_Vla_WeightingPosByAccCtrl"
        elif self.args.vla_type == "PlanEndEffector_Vla_WeightingPosByAccCtrl":
            str_req = "plan_mode=PlanEndEffector_Vla_WeightingPosByAccCtrl"
        else:
            self.logger.error("unsupported vla_type: " + self.args.vla_type)
            self.galbot.error_imformation = (
                self.galbot.error_imformation
                + "unsupported vla_type: "
                + self.args.vla_type
                + ", "
            )
            return False

        t0 = time.perf_counter()
        while True:
            res = self.galbot.request_vla_service(str_req)
            self.logger.info(str_req + ": " + res)
            if (
                self.args.vla_type == "VLA_training_time_RTC"
                and res == "PlanJoint_Vla_TrainingRTC"
            ):
                change_success = True
                break
            elif (
                self.args.vla_type == "VLA"
                and res == "PlanJoint_Vla_WeightingPosByAccCtrl"
            ):
                change_success = True
                break
            elif (
                self.args.vla_type == "PlanEndEffector_Vla_WeightingPosByAccCtrl"
                and res == "PlanEndEffector_Vla_WeightingPosByAccCtrl"
            ):
                change_success = True
                break
            else:
                time.sleep(0.1)
            if time.perf_counter() - t0 > 1:
                self.logger.info("change_control_mode time out")
                self.galbot.error_imformation = (
                    self.galbot.error_imformation + "change_control_mode time out, "
                )
                break

        return change_success

    # @profile_timeline(cat="lifecycle", name="GalbotVLA.shutdown", min_duration_ms=0.05)
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


# ==================== HTTP 服务 ====================
app = Flask(__name__)

SERVER_PORT = 9052

_vla: GalbotVLA = None
_task_thread: threading.Thread = None
_task_lock = threading.Lock()
_task_status = {
    "running": False,
    "success": None,
    "message": "",
    "start_time": None,
    "end_time": None,
}
# ==================== 模型配置 ====================
MODEL_PORT = 6686  # 整理桌面模型端口

_args_global = Args()


def _ok(data=None, msg=""):
    return jsonify({"code": 0, "data": data or {}, "msg": msg})


def _err(msg, status=400):
    return jsonify({"code": 1, "data": {}, "msg": msg}), status


# ==================== 任务定义 ====================
TASK_MAP = {
    "pick_bag": {
        "task": "Pick up the bag and place it on the table.",
        "need_init_pose": True,
    },
        "bag_large_items": {
        "task": "Put the large objects on the table into the bag.",
        "need_init_pose": False,
    },
    "close_box": {
        "task": "Close the box and put it into the bag.",
        "need_init_pose": False,
    },
    "sweep_trash": {
        "task": "Sweep the remaining trash on the table into the white basin, then put it into the bag.",
        "need_init_pose": False,
    }
}


def _run_task(task_key: str):
    global _task_status
    task_config = TASK_MAP[task_key]
    task_prompt = task_config["task"]
    _args_global.task = task_prompt
    _args_global.has_init_action = task_config["need_init_pose"]

    # 每个子任务开始前先确保上一轮推理彻底停止
    logger = LoggerManager.get_logger()
    logger.info(f"[{task_key}] 执行前先调用 stop 确保机器人静止...")
    _vla.shutdown()
    # 等待 shutdown 完成后重置状态，以便新任务能正常启动
    time.sleep(0.3)

    with _task_lock:
        _task_status.update(
            {
                "running": True,
                "success": None,
                "message": f"推理执行中... [{task_key}]",
                "task_key": task_key,
                "task_prompt": task_prompt,
                "start_time": time.time(),
                "end_time": None,
            }
        )
    try:
        success, msg = _vla.run(_args_global)
        with _task_lock:
            _task_status.update(
                {
                    "running": False,
                    "success": success,
                    "message": msg,
                    "end_time": time.time(),
                }
            )
    except Exception as e:
        LoggerManager.get_logger().error(f"任务执行异常: {e}", exc_info=True)
        with _task_lock:
            _task_status.update(
                {
                    "running": False,
                    "success": False,
                    "message": str(e),
                    "end_time": time.time(),
                }
            )


def _start_task(task_key: str, label: str):
    """通用启动逻辑：检查并发 → 启动后台线程"""
    global _task_thread
    with _task_lock:
        if _task_status["running"]:
            return _err("任务正在执行中，请稍后再试")
    _task_thread = threading.Thread(target=_run_task, args=(task_key,), daemon=True)
    _task_thread.start()
    return _ok(msg=f"{label}推理任务已启动")


@app.route("/api/pick_bag", methods=["POST"])
def api_pick_bag():
    return _start_task("pick_bag", "升降取垃圾袋")


@app.route("/api/bag_large_items", methods=["POST"])
def api_bag_large_items():
    return _start_task("bag_large_items", "桌面物品清理")


@app.route("/api/close_box", methods=["POST"])
def api_close_box():
    return _start_task("close_box", "盖盖子")


@app.route("/api/sweep_trash", methods=["POST"])
def api_sweep_trash():
    return _start_task("sweep_trash", "抹布清理")


@app.route("/api/stop", methods=["POST"])
def api_stop():
    if _vla:
        _vla.shutdown()
    return _ok(msg="停止信号已发送")


@app.route("/api/status", methods=["GET"])
def api_status():
    with _task_lock:
        status_copy = _task_status.copy()
    if status_copy["start_time"]:
        end = status_copy["end_time"] or time.time()
        status_copy["duration"] = round(end - status_copy["start_time"], 2)
    else:
        status_copy["duration"] = 0
    return _ok(status_copy)


@app.route("/api/health", methods=["GET"])
def api_health():
    with _task_lock:
        return _ok(
            {
                "service": "clear_table",
                "task_running": _task_status["running"],
                "last_task_success": _task_status["success"],
            }
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-host", default="172.20.10.4", help="模型服务器IP")
    cli_args = parser.parse_args()

    _args_global.host = [cli_args.model_host]
    _args_global.port = [MODEL_PORT]

    _vla = GalbotVLA(_args_global)

    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(_vla.shutdown)
    tool_shutdown.on_shutdown(_vla.galbot.shutdown)
    for i in range(len(_args_global.host)):
        tool_shutdown.on_shutdown(_vla.model_agent[i].ws_client.conn.close)

    print("=" * 60)
    print("银河1号真机服务 - 清理桌面（4个子任务）")
    print(f"[Model]  模型服务: {_args_global.host[0]}:{_args_global.port[0]}")
    print("=" * 60)
    print(f"[Server] 端口: {SERVER_PORT}")
    print(f"[API]    升降取垃圾袋: POST http://localhost:{SERVER_PORT}/api/pick_bag")
    print(
        f"[API]    桌面物品清理: POST http://localhost:{SERVER_PORT}/api/bag_large_items"
    )
    print(f"[API]    盖盖子:       POST http://localhost:{SERVER_PORT}/api/close_box")
    print(f"[API]    抹布清理:     POST http://localhost:{SERVER_PORT}/api/sweep_trash")
    print(f"[API]    停止任务:     POST http://localhost:{SERVER_PORT}/api/stop")
    print(f"[API]    任务状态:     GET  http://localhost:{SERVER_PORT}/api/status")
    print(f"[API]    健康检查:     GET  http://localhost:{SERVER_PORT}/api/health")
    print("=" * 60)

    time.sleep(3)
    app.run(host="0.0.0.0", port=SERVER_PORT, threaded=True)
