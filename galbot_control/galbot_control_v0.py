import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from args import Args, OBS_Head_IMG, OBS_PREV_STATE, OBS_STATE, OBS_Right_WRIST_IMG, OBS_Left_WRIST_IMG
from galbot_control.galbot_interface.galbot_interface import GalbotInterface
from model_agent.openpi_client import image_tools
import json
import copy
import time
import threading
import numpy as np
import datetime
import cv2
import zmq
import base64
from tool import math_utils
from tool.tool_fk import ToolFK
from tool.tool_rate import ToolRate
from tool.logger import LoggerManager
from tool.vla_profiler import profile_span, profile_timeline
from tool.ImageProcess import ImageProcess
from .galbot_interface.embosa_publisher import EmbosaPublisher
from .galbot_interface.embosa_client import EmbosaClient
from concurrent.futures import ThreadPoolExecutor, wait, ALL_COMPLETED


class GalbotControl:
    def __init__(self, args: Args):
        self.args = args
        self.galbot_interface = GalbotInterface(args)
        self.embosa_vla_publisher = EmbosaPublisher("embosa_vla_command")
        self.embosa_vla_client = EmbosaClient("embosa_vla_service")
        self.shutdown_event = threading.Event()
        self.init_pose = None
        self.mat_pub = None
        self.error_imformation = ""
        self.logger = LoggerManager.get_logger()
        resize_size = [args.target_image_size_head, args.target_image_size_left_arm, args.target_image_size_right_arm]
        crop_pos = [self.get_img_crop_para(args.raw_image_size_head, resize_size[0]), self.get_img_crop_para(args.raw_image_size_left_arm, resize_size[1]), self.get_img_crop_para(args.raw_image_size_right_arm, resize_size[2])]
        self.img_process = ImageProcess(crop_pos, resize_size, 90)
        np.set_printoptions(linewidth=1000)
        np.set_printoptions(suppress=True)

    # wholebody
    #@profile_timeline(cat="control", name="GalbotControl.move_to_init_pose_wholebody", min_duration_ms=0.1)
    def move_to_init_pose_wholebody(self):
        # Region A (general): init_pose comes from a datacook init_pose.json artifact.
        # Region B (grocery): init_pose comes from configs_golf/*.json selected by prompt_type.
        if getattr(self.args, "input_source", "grocery") == "general":
            init_pose_file = getattr(self.args, "init_pose_file", "")
            if not init_pose_file:
                self.error_imformation = self.error_imformation + "input_source='general' requires args.init_pose_file, "
                return
            repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            init_pose_path = init_pose_file if os.path.isabs(init_pose_file) else os.path.join(repo_root, init_pose_file)
            with open(init_pose_path, "r") as f:
                loaded = json.load(f).get("init_pose", [])
            if len(loaded) != 23:
                self.error_imformation = self.error_imformation + f"{init_pose_path} missing 23-dim 'init_pose', "
                return
            self.init_pose = list(loaded)
        else:
            cur_dir = os.path.dirname(os.path.abspath(__file__))
            if any(prompt_type in self.args.prompt_type for prompt_type in ["zjn", "fix"]):
                config_path = os.path.join( os.path.dirname(cur_dir), "config", self.args.config_path, "fix_height" + str(self.args.object_location[0]) + ".json" )
            elif any(prompt_type in self.args.prompt_type for prompt_type in ["wb", "lcl"]):
                config_path = os.path.join( os.path.dirname(cur_dir), "config", self.args.config_path, "shelf_height_wb2.json")
            elif any(prompt_type in self.args.prompt_type for prompt_type in ["czy", "stock"]):
                config_path = os.path.join( os.path.dirname(cur_dir), "config", self.args.config_path, "stocking_" + self.args.hand_used + ".json")

            with open(config_path, "r") as f:
                data = json.load(f)
                self.init_pose = data["init_pose"]

        # gripper override: non-empty list wins; None/[] falls back to init_pose[14]/[22].
        if self.args.gripper_init_width:
            self.init_pose[14] = self.args.gripper_init_width[0]
            self.init_pose[22] = self.args.gripper_init_width[1]

        if self.galbot_interface.last_chassis_pos is None and self.args.enable_chassis>0:
            self.error_imformation = self.error_imformation + "can't get chassis_pos, "
            return
        if self.galbot_interface._joint_sensor_vla is None or len(self.galbot_interface.pose_buffer)<2:
            self.error_imformation = self.error_imformation + "_joint_sensor_vla is None, "
            return
        
        if self.args.has_init_action:
            self.logger.info("start init pose")
            self.set_wholebody_angle_asynchronous(np.array([4] + self.init_pose + self.galbot_interface.chassis_pos).reshape(-1, 1), 0.03, 0.004)
            self.logger.info("finish init pose")

        self.set_parameter(self.args)

        q_wholebody = self.galbot_interface.pose_buffer[1]
        [self.galbot_interface.left_end_T, self.galbot_interface.right_end_T] = self.galbot_interface.tool_fk.get_T_frame(np.array(q_wholebody[0:14]+q_wholebody[15:22]),["left_arm_end_effector_mount_link","right_arm_end_effector_mount_link"])
        self.galbot_interface.right_end_T_deepest = copy.deepcopy(self.galbot_interface.right_end_T)
        self.galbot_interface.left_end_T_deepest = copy.deepcopy(self.galbot_interface.left_end_T)
        self.mat_pub = np.zeros((2 + len(q_wholebody), 1))
        self.mat_pub[2 : 2 + len(q_wholebody), 0:1] = np.array(q_wholebody).reshape(-1, 1)

        pose_wholebody = self.galbot_interface.pose_buffer[1]
        for i in range(200):
            self.galbot_interface.pose_buffer.appendleft(pose_wholebody)

        if self.args.visualization:
            msg = {"type": "clear_data"}
            self.galbot_interface.socket_current_state.send(json.dumps(msg).encode("utf-8"))

    #@profile_timeline(cat="obs", name="GalbotControl.get_obs_wholebody_compressed", min_duration_ms=0.1)
    def get_obs_wholebody_compressed(self, flag_first_run, last_action_time_delay=300,response = None):
        head_img, head_img_res = self.galbot_interface.get_image("head_left")
        right_wrist_img, right_wrist_img_res = self.galbot_interface.get_image("hand_right")
        left_wrist_img, left_wrist_img_res = self.galbot_interface.get_image("hand_left")

        if (head_img is None) or (right_wrist_img is None) or  (left_wrist_img is None):
            self.error_imformation = self.error_imformation + head_img_res + right_wrist_img_res +  left_wrist_img_res
            return None, None
        if not self.check_img_para():
            return None, None
        
        _image_head_left_timestamp = self.galbot_interface._image_head_left_timestamp
        _image_hand_left_timestamp = self.galbot_interface._image_hand_left_timestamp
        _image_hand_right_timestamp = self.galbot_interface._image_hand_right_timestamp
        self.img_process.pipeline_decode_crop_resize_python_zero_copy([head_img,left_wrist_img,right_wrist_img])

        idx = self.find_closest_idx_desc(self.galbot_interface.pose_timestamp_buffer, _image_head_left_timestamp) 
        obs_state = self.galbot_interface.pose_buffer[idx]
        obs_state_timestamp = self.galbot_interface.pose_timestamp_buffer[idx]

        obs_prve_state = []
        timestamp_state_sent = [obs_state_timestamp]
        for i in range(1,self.args.proprio_step):
            idx = self.find_closest_idx_desc(self.galbot_interface.pose_timestamp_buffer, obs_state_timestamp - self.args.dt_model_control * i) 
            obs_prve_state.append(self.galbot_interface.pose_buffer[idx])
            timestamp_state_sent.append(self.galbot_interface.pose_timestamp_buffer[idx])
        self.logger.info("\n timestamp_state_sent: " + str(timestamp_state_sent) + 
                         "\n the latest timestamp:  " + str(self.galbot_interface.pose_timestamp_buffer[0]) + 
                         "\n head_left, left_arm, right_arm timestamp: " + str([_image_head_left_timestamp, _image_hand_left_timestamp, _image_hand_right_timestamp]))

        while(not self.img_process.is_finished()):
            pass
        [obs_head_img, obs_left_wrist_img, obs_right_wrist_img] = self.img_process.get_result()

        obs = {
            OBS_Head_IMG: obs_head_img,
            OBS_Right_WRIST_IMG: obs_right_wrist_img,
            OBS_Left_WRIST_IMG: obs_left_wrist_img,
            OBS_STATE: np.array(obs_state[0:23]),
            OBS_PREV_STATE: [np.array(state[:23]) for state in obs_prve_state],
            "observation/chassis": np.array(obs_state[23:26]),
            "observation/prev_chassis": [np.array(state[23:26]) for state in obs_prve_state],
            "tag/new_task": 1 if flag_first_run else 0,
            "observation/delay": last_action_time_delay,
            "SKU_name": self.args.object_name[0],
            "robot_type": self.args.robot_type,
            "control/is_blocking": self.args.blocking,
            "control/action_horizon_use": self.args.action_horizon_use,
        }
        obs.update(self.individual_fields())
        if response is None:
            obs["known_actions"] = []
        else:
            obs["known_actions"] = response["actions"][0:self.args.action_horizon_use,:]

        obs_visualization = {
            "time_obs": obs_state_timestamp,
            OBS_Head_IMG: obs_head_img,
            OBS_Head_IMG + "_timestamp": self.galbot_interface._image_head_left_timestamp,
            OBS_Right_WRIST_IMG: obs_right_wrist_img,
            OBS_Right_WRIST_IMG + "_timestamp": self.galbot_interface._image_hand_right_timestamp,
            OBS_Left_WRIST_IMG: obs_left_wrist_img,
            OBS_Left_WRIST_IMG + "_timestamp": self.galbot_interface._image_hand_left_timestamp,
            OBS_STATE: obs_state,
            OBS_STATE + "_timestamp": obs_state_timestamp,
            OBS_PREV_STATE: obs_prve_state[0],
        }

        return obs, obs_visualization

    #@profile_timeline(cat="control", name="GalbotControl.pub_wholebody_command", min_duration_ms=0.1)
    def pub_wholebody_command(self, obs, obs_visualization, response, action_time_delay):
        
        leg_command, head_command, left_command, right_command, chassis_command = self.get_abs_command_wholebody(obs, response)

        mat = np.zeros((2, left_command.shape[1]))
        mat[0, 0] = action_time_delay
        for i in range(left_command.shape[1]):
            mat[1, i] = mat[1, i] + i * self.args.dt_model_control

        mat = np.append(mat, leg_command, axis=0)
        mat = np.append(mat, head_command, axis=0)
        mat = np.append(mat, left_command, axis=0)
        mat = np.append(mat, right_command, axis=0)
        mat = np.append(mat, chassis_command, axis=0)
        mat_pub = mat[:, 0 : 1 + self.args.action_horizon_use]

        self.embosa_vla_publisher.pub_mat(mat_pub)
        self.mat_pub = mat_pub

        if self.args.visualization:
            msg = {
                "type": "model_point",
                "time_pub": time.time(),
                "model_result": mat_pub.tolist(),
                "box_front_camera_left": np.array(response["bbox"]["front_camera_left"]).tolist(),
                "box_front_camera": np.array(response["bbox"]["front_camera"]).tolist(),
                "box_left_camera": np.array(response["bbox"]["left_camera"]).tolist(),
                "box_right_camera": np.array(response["bbox"]["right_camera"]).tolist(),
                "action_time_delay": float(action_time_delay),
            }
            self.galbot_interface.socket_model_result.send(obs_visualization[OBS_Head_IMG], flags=zmq.SNDMORE)
            self.galbot_interface.socket_model_result.send(obs_visualization[OBS_Left_WRIST_IMG], flags=zmq.SNDMORE)
            self.galbot_interface.socket_model_result.send(obs_visualization[OBS_Right_WRIST_IMG], flags=zmq.SNDMORE)
            obs_visualization.pop(OBS_Head_IMG)
            obs_visualization.pop(OBS_Left_WRIST_IMG)
            obs_visualization.pop(OBS_Right_WRIST_IMG)
            msg.update(obs_visualization)
            self.galbot_interface.socket_model_result.send(json.dumps(msg).encode("utf-8"))

        self.logger.info("mat_sent \n" + self.img_process.numpy2D_to_str(mat_pub,5))

    #@profile_timeline(cat="control", name="GalbotControl.set_wholebody_command", min_duration_ms=0.1)
    def set_wholebody_command(self, obs, obs_visualization, response):
        self.pub_wholebody_command(obs, obs_visualization, response, 0)

        wait_before_check = self.args.action_horizon_use * self.args.dt_model_control - 0.05
        if wait_before_check > 0:
            time.sleep(wait_before_check)

        t0 = time.perf_counter()
        timeout = self.args.action_horizon_use * self.args.dt_model_control + 5.0
        while not self.shutdown_event.is_set():
            res = self.request_vla_service("if_all_stopped")
            self.logger.info("if_stop: " + res)
            if res == "yes":
                self.logger.info(self.request_vla_service("stop"))
                break
            if time.perf_counter() - t0 > timeout:
                self.error_imformation = self.error_imformation + "set_wholebody_command wait if_all_stopped timeout, "
                self.logger.error(self.error_imformation)
                self.logger.info(self.request_vla_service("stop"))
                break
            time.sleep(0.03)

        if self.shutdown_event.is_set():
            self.logger.info(self.request_vla_service("stop"))

    #@profile_timeline(cat="control", name="GalbotControl.get_abs_command_wholebody", min_duration_ms=0.05)
    def get_abs_command_wholebody(self, obs, response):
        delta_actions = copy.deepcopy(response["actions"])

        if self.args.control_mode == "joint":
            leg_delta_actions = delta_actions[:, 0:5]
            head_delta_actions = delta_actions[:, 5:7]
            left_delta_actions = delta_actions[:, 7:15]
            right_delta_actions = delta_actions[:, 15:23]
            chassis_delta_actions = delta_actions[:, 23:26]

            if self.args.vla_type == "VLA":
                command = np.hstack([obs[OBS_STATE]]).reshape(-1, 1)
                chassis_command = np.hstack([obs["observation/chassis"]]).reshape(-1, 1)
            elif self.args.vla_type == "VLA_training_time_RTC":
                command = self.mat_pub[2:26+2,self.mat_pub.shape[1]-1:self.mat_pub.shape[1]]
                command[14] = command[14]*0.001
                command[22] = command[22]*0.001
                chassis_command = command[23:26, 0:1]
            leg_command = command[0:5, 0:1]
            head_command = command[5:7, 0:1]
            left_command = command[7:15, 0:1]
            right_command = command[15:23, 0:1]                

            leg_command = np.append(leg_command, leg_delta_actions.transpose(), axis=1)
            head_command = np.append(head_command, head_delta_actions.transpose(), axis=1)
            left_command = np.append(left_command, left_delta_actions.transpose(), axis=1)
            left_command[7:8, 0:] = 1000 * left_command[7:8, 0:]
            right_command = np.append(right_command, right_delta_actions.transpose(), axis=1)
            right_command[7:8, 0:] = 1000 * right_command[7:8, 0:]
            chassis_command = np.append(chassis_command, chassis_delta_actions.transpose(), axis=1)

            for i in range(1, right_command.shape[1]):
                tmp = leg_command[0:5, i]
                leg_command[0:5, i] = leg_command[0:5, i - 1] + tmp

                tmp = head_command[0:2, i]
                head_command[0:2, i] = head_command[0:2, i - 1] + tmp

                tmp = left_command[0:7, i]
                left_command[0:7, i] = left_command[0:7, i - 1] + tmp

                tmp = right_command[0:7, i]
                right_command[0:7, i] = right_command[0:7, i - 1] + tmp

                tmp = chassis_command[0:3, i]
                R_mat = np.array([ [ np.cos(chassis_command[2,i-1]), -np.sin(chassis_command[2,i-1]) ],[ np.sin(chassis_command[2,i-1]), np.cos(chassis_command[2,i-1]) ] ])
                chassis_command[0:2, i:i+1] = chassis_command[0:2, i-1:i] + R_mat @ tmp[0:2].reshape(-1,1)
                chassis_command[2, i] = chassis_command[2, i - 1] + tmp[2]

            return leg_command, head_command, left_command, right_command, chassis_command

        elif self.args.control_mode == "eef":
            leg_delta_actions = delta_actions[:, 0:5]
            head_delta_actions = delta_actions[:, 5:7]
            left_delta_actions = delta_actions[:, 7:14]
            right_delta_actions = delta_actions[:, 14:21]
            chassis_delta_actions = delta_actions[:, 21:24]

            command = np.hstack([obs[OBS_STATE]]).reshape(-1, 1)
            leg_command = command[0:5, 0:1]
            head_command = command[5:7, 0:1]
            [left_end_T, right_end_T] = self.galbot_interface.tool_fk.get_T_frame(np.hstack([command[0:14],command[15:22]]),["left_arm_end_effector_mount_link","right_arm_end_effector_mount_link"])
            left_command = np.zeros((7,1))
            left_command[0:3, 0] = left_end_T[0:3, 3].reshape(-1)
            left_command[3:6, 0] = math_utils.rotm2axis_angle(left_end_T[0:3, 0:3]).reshape(-1)
            left_command[6,0] = command[14, 0]
            right_command = np.zeros((7,1))
            right_command[0:3, 0] = right_end_T[0:3, 3].reshape(-1)
            right_command[3:6, 0] = math_utils.rotm2axis_angle(right_end_T[0:3, 0:3]).reshape(-1)
            right_command[6,0] = command[22, 0]
            chassis_command = np.hstack([obs["observation/chassis"]]).reshape(-1, 1)

            leg_command = np.append(leg_command, leg_delta_actions.transpose(), axis=1)
            head_command = np.append(head_command, head_delta_actions.transpose(), axis=1)
            left_command = np.append(left_command, left_delta_actions.transpose(), axis=1)
            right_command = np.append(right_command, right_delta_actions.transpose(), axis=1)
            chassis_command = np.append(chassis_command, chassis_delta_actions.transpose(), axis=1)
            left_command[6:7, 0:] = 1000 * left_command[6:7, 0:]
            right_command[6:7, 0:] = 1000 * right_command[6:7, 0:]
            R_left = left_end_T[0:3, 0:3]
            R_right = right_end_T[0:3, 0:3]

            for i in range(1, right_command.shape[1]):
                tmp = leg_command[0:5, i]
                leg_command[0:5, i] = leg_command[0:5, i - 1] + tmp

                tmp = head_command[0:2, i]
                head_command[0:2, i] = head_command[0:2, i - 1] + tmp

                tmp = left_command[0:3, i]
                left_command[0:3, i] = left_command[0:3, i - 1] + tmp
                tmp = R_left.copy()
                R_left = tmp @ math_utils.axis_angle2rotm(left_command[3:6, i])
                left_command[3:6, i] = math_utils.rotm2axis_angle(R_left)

                tmp = right_command[0:3, i]
                right_command[0:3, i] = right_command[0:3, i - 1] + tmp
                tmp = R_right.copy()
                R_right = tmp @ math_utils.axis_angle2rotm(right_command[3:6, i])
                right_command[3:6, i] = math_utils.rotm2axis_angle(R_right)

                tmp = chassis_command[0:3, i]
                R_mat = np.array([ [ np.cos(chassis_command[2,i-1]), -np.sin(chassis_command[2,i-1]) ],[ np.sin(chassis_command[2,i-1]), np.cos(chassis_command[2,i-1]) ] ])
                chassis_command[0:2, i:i+1] = chassis_command[0:2, i-1:i] + R_mat @ tmp[0:2].reshape(-1,1)
                chassis_command[2, i] = chassis_command[2, i - 1] + tmp[2]

            return leg_command, head_command, left_command, right_command, chassis_command

    #@profile_timeline(cat="prompt", name="GalbotControl.individual_fields", min_duration_ms=0.02)
    def individual_fields(self):
        dic = {}
        dic["task_images"] = list(getattr(self.args, "task_images", []) or [])
        # Region A (general): use args.task verbatim (must match training tasks.jsonl).
        # Region B (grocery): fall through to prompt_type-based templates below.
        if getattr(self.args, "input_source", "grocery") == "general":
            dic["prompt"] = self.args.task
            return dic
        if self.args.prompt_type == "zjn":
            dic["prompt"] =  f"grasp the {self.args.object_name[0]} located at front row 1 col {self.args.object_location[1]}, then take it out of the shelf"
        elif self.args.prompt_type in ["fix_shelf_freezer", "zjn_2"]:
            dic["prompt"] = f"grasp the {self.args.object_name[0]} located at col {self.args.object_location[1]} with {self.args.hand_used} hand, then take it out of the shelf"
        elif self.args.prompt_type in ["fix_shelf_general"]:
            dic["prompt"] = f"grasp the next {self.args.object_name[0]} with {self.args.hand_used} hand, then take it out of the shelf"
        elif self.args.prompt_type in ["wb_shelf_freezer", "lcl"]:
            if len(self.args.shelf_location) > 0:
                dic["prompt"] = f"grasp the {self.args.object_name[0]} located at col {self.args.object_location[1]} with {self.args.hand_used} hand, then take it back to the table. the {self.args.object_name[0]} is on the layer {self.args.object_location[0]} {self.args.object_location_on_shelf} side of the shelf, and the shelf is at the {self.args.shelf_location[0]} of the table."
            else:
                dic["prompt"] = f"grasp the {self.args.object_name[0]} located at col {self.args.object_location[1]} with {self.args.hand_used} hand, then take it back to the table."
        elif self.args.prompt_type == "wb_shelf_general":
            if len(self.args.shelf_location) > 0:
                dic["prompt"] = f"grasp the next {self.args.object_name[0]} with {self.args.hand_used} hand, then take it back to the table. the {self.args.object_name[0]} is on the layer {self.args.object_location[0]} {self.args.object_location_on_shelf} side of the shelf, and the shelf is at the {self.args.shelf_location[0]} of the table."
            else:
                dic["prompt"] = f"grasp the next {self.args.object_name[0]} with {self.args.hand_used} hand, then take it back to the table."
        elif self.args.prompt_type in ["czy", "shelf_stock"]:
            print(f'use {self.args.hand_used} hand')
            sku_name = self.args.object_name[0]
            obj_location = f'row {self.args.object_location[0]}, col {self.args.object_location[1]}'
            dic["prompt"] = f"grasp the {sku_name} with {self.args.hand_used} hand, then put it on the {obj_location} of the shelf" # qwen3vl v0
        return dic
    
    # fun
    #@profile_timeline(cat="control", name="GalbotControl.set_wholebody_angle_asynchronous", min_duration_ms=0.1)
    def set_wholebody_angle_asynchronous(self, aim_pos, tolerance_motor, tolerance_gripper):
        target_time = aim_pos[0, aim_pos.shape[1] - 1]
        tolerance_time = 6
        t0 = time.perf_counter()

        while not self.shutdown_event.is_set():
            res = self.request_vla_service("stop")
            self.logger.info("set_wholebody_angle_asynchronous request_vla_service('stop'): " + res)
            if res == "stop":
                time.sleep(0.1)
                break
            else:
                time.sleep(0.1)
            if time.perf_counter() - t0 > target_time + tolerance_time:
                self.error_imformation = ( self.error_imformation + "set_wholebody_angle_asynchronous request_vla_service('stop') timeout, " )
                break

        args = copy.deepcopy(self.args)
        args.lim_vel = 5
        args.lim_acc = 15
        self.set_parameter(args)

        if self.error_imformation != "":
            return
        q_wholebody = self.galbot_interface.pose_buffer[1]

        mat = np.zeros((2 + len(q_wholebody), 2))
        mat[2 : 2 + len(q_wholebody), 0:1] = np.array(q_wholebody).reshape(-1, 1)
        mat[1 : 2 + len(q_wholebody), 1 : 1 + aim_pos.shape[1]] = aim_pos
        mat[16:17, :] = 1000 * mat[16:17, :]
        mat[24:25, :] = 1000 * mat[24:25, :]
        self.logger.info("init pose mat_sent \n" + self.img_process.numpy2D_to_str(mat,5))
        self.embosa_vla_publisher.pub_mat(mat)
        time.sleep(0.1)

        while not self.shutdown_event.is_set():
            res = self.request_vla_service("if_any_module_is_running")
            self.logger.info("set_wholebody_angle_asynchronous if_any_module_is_running: " + res)
            if "yes" in res:
                break
            elif res == "no":
                self.embosa_vla_publisher.pub_mat(mat)
                time.sleep(0.1)
            else:
                time.sleep(0.1)
            if time.perf_counter() - t0 > target_time + tolerance_time:
                self.error_imformation = ( self.error_imformation + "set_wholebody_angle_asynchronous pub_mat(mat) timeout, ")
                break

        if self.error_imformation != "":
            return

        time.sleep(target_time - 0.1)

        while not self.shutdown_event.is_set():
            res = self.request_vla_service("if_all_stopped")
            self.logger.info("set_wholebody_angle_asynchronous request_vla_service('if_all_stopped'): " + res)
            if res == "yes":
                break
            time.sleep(0.1)
            if time.perf_counter() - t0 > target_time + tolerance_time:
                self.error_imformation = (self.error_imformation + "set_wholebody_angle_asynchronous check if_all_stopped timeout, ")
                break

        if self.error_imformation != "":
            return

        timeout = False
        while not self.shutdown_event.is_set():
            if time.perf_counter() - t0 > target_time + tolerance_time:
                timeout = True
                self.error_imformation = self.error_imformation + "set_wholebody_angle_asynchronous timeout, "

            pose_wholebody = self.galbot_interface.pose_buffer[1]
            finish_init_pose = True
            for i in range(23):
                if i == 14 or i == 22:
                    if (i == 14 and pose_wholebody[14]>-0.5) or (i == 22 and pose_wholebody[22]>-0.5):
                        if abs(aim_pos[1 + i, aim_pos.shape[1] - 1] - pose_wholebody[i]) > tolerance_gripper:
                            finish_init_pose = False
                            if timeout:
                                self.error_imformation = ( self.error_imformation + "joint " + str(i) + "err is:" + str(abs(aim_pos[1 + i, aim_pos.shape[1] - 1] - pose_wholebody[i])) + ", " )
                else:
                    if abs(aim_pos[1 + i, aim_pos.shape[1] - 1] - pose_wholebody[i]) > tolerance_motor:
                        finish_init_pose = False
                        if timeout:
                            self.error_imformation = ( self.error_imformation + "joint " + str(i) + "err is:" + str(abs(aim_pos[1 + i, aim_pos.shape[1] - 1] - pose_wholebody[i])) + ", ")

            if finish_init_pose or timeout:
                break
            time.sleep(0.05)

    #@profile_timeline(cat="control", name="GalbotControl.set_parameter", min_duration_ms=0.05)
    def set_parameter(self, args):
        msg = (
            "set_parameter"
            f"<leg_module_enable      :[1]>\n"
            f"<head_module_enable     :[1]>\n"
            f"<left_arm_module_enable :[1]>\n"
            f"<right_arm_module_enable:[1]>\n"
            f"<chassis_module_enable  :[{args.enable_chassis}]>\n"
            f"<enable_takeover        :[{args.enable_takeover}]>\n"
            f"<gripper_effort         :[{args.gripper_effort}]>\n"
            f"<gripper_vel            :[{args.gripper_vel}]>\n"
            f"<lambda_proportion      :[{args.lambda_para[0]},{args.lambda_para[1]}]>\n"
            f"<lim_vel                :[{args.lim_vel}]>\n"
            f"<lim_acc                :[{args.lim_acc}]>\n"
            f"<damping_D              :[{args.damping_D}]>\n"
            f"<stiffness_K            :[{args.stiffness_K}]>\n"
            f"<traj_filtering         :[{args.traj_filtering}]>\n"
            f"<para_chassis_p         :[{args.para_chassis_p[0]},{args.para_chassis_p[1]},{args.para_chassis_p[2]}]>\n"
        )
        t0 = time.perf_counter()
        while not self.shutdown_event.is_set():
            res = self.request_vla_service(msg)
            self.logger.info(msg + ": " + res)
            if res == "set_parameter":
                time.sleep(0.05)
                break
            else:
                time.sleep(0.1)
            if time.perf_counter() - t0 > 5:
                self.error_imformation = self.error_imformation + "set_parameter timeout, "
                break

    #@profile_timeline(cat="control", name="GalbotControl.request_vla_service", min_duration_ms=0.05)
    def request_vla_service(self, msg_str):
        return self.embosa_vla_client.request_service(msg_str)

    #@profile_timeline(cat="obs", name="GalbotControl.get_img_crop_para", min_duration_ms=0.02)
    def get_img_crop_para(self, raw_size, aim_size):
        # aim_size = [横向尺寸， 纵向尺寸]
        #以左上角为(0,0),横向向右为正，纵向向下为正， 返回 [ 左上角点在横向的位置，左上角点在纵向的位置， 横向尺寸，纵向尺寸 ]
        if float(raw_size[0])/float(aim_size[0]) > float(raw_size[1])/float(aim_size[1]):
            width = raw_size[0]
            height = raw_size[1]
            coff = float(aim_size[0])/float(aim_size[1])
            return [  int((width - coff*height)/2) ,   0,   int(coff*height),  raw_size[1]  ]
        else:
            width = raw_size[0]
            height = raw_size[1]
            coff = float(aim_size[1])/float(aim_size[0])
            return [  0 ,  int((height - coff*width)/2)  ,  raw_size[0]  ,  int(coff*width)   ]

    #@profile_timeline(cat="obs", name="GalbotControl.get_undistorted_image", min_duration_ms=0.1)
    def get_undistorted_image(self, image_np, image_info):
        camera_matrix = np.array(image_info["k"], dtype=np.float32).reshape((3, 3))
        dist_coeffs = np.array(image_info["d"], dtype=np.float32)
        image_size = (image_info["width"], image_info["height"])

        if image_info["distortion_model"] == "plumb_bob":
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, image_size, 1, image_size
            )

            map1, map2 = cv2.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, None, new_camera_matrix, image_size, cv2.CV_16SC2
            )

            undistorted_image = cv2.remap(image_np, map1, map2, cv2.INTER_LINEAR)

            return undistorted_image

        elif image_info["distortion_model"] == "kannala_brandt":
            new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                camera_matrix, dist_coeffs[0:4], image_size, np.eye(3), balance=0.0
            )

            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix, dist_coeffs[0:4], np.eye(3), new_camera_matrix, image_size, cv2.CV_16SC2
            )

            undistorted_image = cv2.remap(
                image_np, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
            )

            return undistorted_image

    #@profile_timeline(cat="util", name="GalbotControl.find_closest_idx_desc", min_duration_ms=0.02)
    def find_closest_idx_desc(self,ts_buffer, target):
        left = 0
        right = len(ts_buffer) - 1

        if target >= ts_buffer[0]:
            return 0
        if target <= ts_buffer[-1]:
            return right

        while left <= right:
            mid = (left + right) // 2
            if ts_buffer[mid] == target:
                return mid
            elif ts_buffer[mid] > target:
                left = mid + 1
            else:
                right = mid - 1

        if abs(ts_buffer[left] - target) < abs(ts_buffer[left-1] - target):
            return left
        else:
            return left - 1

    #@profile_timeline(cat="obs", name="GalbotControl.check_img_para", min_duration_ms=0.02)
    def check_img_para(self):
        
        if self.galbot_interface._image_info_head_left is None:
            self.error_imformation = self.error_imformation + "_image_info_head_left is None, "
            return False
        if self.galbot_interface._image_info_hand_left is None:
            self.error_imformation = self.error_imformation + "_image_info_hand_left is None, "
            return False
        if self.galbot_interface._image_info_hand_right is None:
            self.error_imformation = self.error_imformation + "_image_info_hand_right is None, "
            return False

        if( self.galbot_interface._image_info_head_left != self.args.raw_image_size_head ):
            self.error_imformation = self.error_imformation + "head image size is " + str(self.galbot_interface._image_info_head_left) + "!=" + str(self.args.raw_image_size_head) + ", "
            return False
        if( self.galbot_interface._image_info_hand_left != self.args.raw_image_size_left_arm ):
            self.error_imformation = self.error_imformation + "left_wrist image size is " + str(self.galbot_interface._image_info_hand_left) + "!=" + str(self.args.raw_image_size_left_arm) + ", "
            return False
        if( self.galbot_interface._image_info_hand_right != self.args.raw_image_size_right_arm ):
            self.error_imformation = self.error_imformation + "right_wrist image size is " + str(self.galbot_interface._image_info_hand_right) + "!=" + str(self.args.raw_image_size_right_arm) + ", "
            return False
        return True

    #@profile_timeline(cat="lifecycle", name="GalbotControl.shutdown", min_duration_ms=0.02)
    def shutdown(self):
        self.shutdown_event.set()
