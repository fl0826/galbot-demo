#!/usr/bin/env python3
import os
import sys
import time
from typing import List
import numpy as np
import math
from collections import deque
import zmq
import json
import copy

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append("/data/galbot/lib/python3.8.10")
sys.path.append("/data/galbot/lib/pw1/python3/dist-packages")
sys.path.append("/data/galbot/lib/pw1/python3/site-packages")
from .generate_request import generate_target_etool, generate_target_whole
import galbot.singorix_proto.singorix_error_pb2 as singorix_error_pb2
import galbot.singorix_proto.singorix_sensor_pb2 as singorix_sensor_pb2
import galbot.singorix_proto.singorix_target_pb2 as singorix_target_pb2
import galbot.navigation_proto.odometry_pb2 as odometry_pb2
import galbot.sensor_proto.image_pb2 as image_pb2
import galbot.sensor_proto.camera_pb2 as camera_pb2
import embosa_python
from tool.logger import logger, LoggerManager
from tool.tool_fk import ToolFK
from tool.tool_frequency_cal import ToolFrequencyCal
from tool.vla_profiler import profile_timeline
from args import Args

class GalbotInterface:
    def __init__(self, args: Args, node=None, node_name="sxs_vla_node"):
        # Initialize embosa if not already done
        if embosa_python.GetState() == embosa_python.STATE_UNINITIALIZED:
            embosa_python.EmbosaInit()

        # Create DDS node if not provided
        self.m_dds_node = node
        if not self.m_dds_node:
            self.m_dds_node = embosa_python.CreateNode(node_name)
            if not self.m_dds_node:
                logger.error("Failed to create DDS node.")
                raise RuntimeError("Failed to create DDS node.")

        self.logger = LoggerManager.get_logger()
        # Initialize member variables
        self._image_head_left = None
        self._image_head_left_timestamp = 0 #timestamp 用msg自带的时间
        self._image_hand_left = None
        self._image_hand_left_timestamp = 0
        self._image_hand_right = None
        self._image_hand_right_timestamp = 0
        self._image_info_head_left = None
        self._image_info_hand_left = None
        self._image_info_hand_right = None
        self._joint_sensor_vla = None
        self.chassis_pos = [0,0,0]
        self.last_chassis_pos = None
        self.callback_frequency_image_head_left = ToolFrequencyCal(30)
        self.callback_frequency_image_hand_left = ToolFrequencyCal(30)
        self.callback_frequency_image_hand_right = ToolFrequencyCal(30)
        self.callback_frequency_sensor = ToolFrequencyCal(100)
        self.callback_frequency_odom = ToolFrequencyCal(100)

        self.pose_buffer = deque(maxlen=200)
        self.pose_timestamp_buffer = deque(maxlen=200)

        cur_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(os.path.dirname(cur_dir), "..","tool", "galbot_one_golf.urdf")
        self.tool_fk = ToolFK(urdf_path)
        self.right_end_T = np.eye(4)
        self.left_end_T = np.eye(4)
        self.right_end_T_deepest = np.eye(4)
        self.left_end_T_deepest = np.eye(4)
        self.cal_T_count = 0
        self.args = args
        if self.args.visualization:
            self.socket_context = zmq.Context()
            self.socket_current_state = self.socket_context.socket(zmq.PUB)
            self.socket_current_state.bind("tcp://*:2173")  # 接受来自所有网卡上的连接请求
            self.socket_model_result = self.socket_context.socket(zmq.PUB)
            self.socket_model_result.bind("tcp://*:2174")  # 接受来自所有网卡上的连接请求

        try:
            # Configure QoS parameters
            sync_qos = embosa_python.Qos()
            sync_qos.intra_core_qos.qos_sync_callback_policy.queue_depth = 5
            sync_qos.intra_core_qos.qos_sync_callback_policy.sub_sync_mode = embosa_python.SYNCHRONOUS_SUB_MODE
            sync_qos.intra_core_qos.qos_callback_mode_policy = embosa_python.CALLBACK_SYNC_MODE
            auto_qos = embosa_python.Qos()
            auto_qos.intra_core_qos.qos_sync_callback_policy.queue_depth = 1
            auto_qos.intra_core_qos.qos_callback_mode_policy = embosa_python.CALLBACK_AUTO_MODE

            auto_qos_image = embosa_python.Qos()
            auto_qos_image.intra_core_qos.qos_sync_callback_policy.queue_depth = 1
            auto_qos_image.intra_core_qos.qos_callback_mode_policy = embosa_python.CALLBACK_AUTO_MODE
            auto_qos_image.intra_core_qos.transport_type = embosa_python.LARGE_DATA_TRANSPORT

            self.m_robot_sensor_reader_vla = self.m_dds_node.CreateSerializationReader(
                singorix_sensor_pb2.SingoriXSensor,
                "singorix/wbcs/sensor",
                self.sensorCallback_vla,
                auto_qos,
            )

            self.m_robot_odom_reader_vla = self.m_dds_node.CreateSerializationReader(
                odometry_pb2.Odometry,
                "/odom/base_link",
                self.odomCallback_vla,
                auto_qos,
            )

            self.m_robot_image_head_left_info_reader = self.m_dds_node.CreateSerializationReader(
                camera_pb2.CameraInfo,
                "/front_head_camera/left_color/camera_info",
                self.ImageInfoHeadLeftCallback,
                sync_qos,
            )

            self.m_robot_image_hand_left_info_reader = self.m_dds_node.CreateSerializationReader(
                camera_pb2.CameraInfo,
                "/left_arm_camera/color/camera_info",
                self.ImageInfoHandLeftCallback,
                sync_qos,
            )

            self.m_robot_image_hand_right_info_reader = self.m_dds_node.CreateSerializationReader(
                camera_pb2.CameraInfo,
                "/right_arm_camera/color/camera_info",
                self.ImageInfoHandRightCallback,
                sync_qos,
            )
            
            self.m_robot_image_head_left_reader = self.m_dds_node.CreateSerializationReader(
                image_pb2.CompressedImage,
                "/front_head_camera/left_color/image_raw",
                self.ImageHeadLeftCallback,
                auto_qos_image,
            )

            self.m_robot_image_hand_left_reader = self.m_dds_node.CreateSerializationReader(
                image_pb2.CompressedImage,
                "/left_arm_camera/color/image_raw",
                self.ImageHandLeftCallback,
                auto_qos_image,
            )            

            self.m_robot_image_hand_right_reader = self.m_dds_node.CreateSerializationReader(
                image_pb2.CompressedImage,
                "/right_arm_camera/color/image_raw",
                self.ImageHandRightCallback,
                auto_qos_image,
            )

            # Create target writer
            self.m_wbcs_target_writer = self.m_dds_node.CreateSerializationWriter(
                singorix_target_pb2.SingoriXTarget, "singorix/wbcs/target"
            )

            # Configure RPCQoS parameters
            rpc_qos = embosa_python.RpcQos()
            rpc_qos.intra_core_rpc_qos.transport_type = embosa_python.LARGE_DATA_TRANSPORT
            # Create target client
            self.m_wbcs_target_client = self.m_dds_node.CreateSerializationClient(
                singorix_target_pb2.SingoriXTarget,
                singorix_error_pb2.SingoriXError,
                "singorix/wbcs/target_server",
                rpc_qos,
            )
            if not self.m_wbcs_target_client:
                logger.error("Create wbcs_target_client fail.")
                raise RuntimeError("Create wbcs_target_client fail.")
            else:
                # Wait for server connection
                logger.info("Waiting for wbcs_target_client to connect to server...")
                self.m_wbcs_target_client.WaitForServerConnected()
                logger.info("wbcs_target_client has connected to server")

        except Exception as e:
            logger.error(f"Exception during SxServices construction: {e}")
            raise

    def __del__(self):
        """Destructor for SxServices"""
        logger.info("SxServices object destroy!")

    #@profile_timeline(cat="sensor", name="GalbotInterface.sensorCallback_vla", min_duration_ms=0.2)
    def sensorCallback_vla(self, message):
        """Callback for robot sensor data"""
        self._joint_sensor_vla = message.joint_sensor_map
        self.callback_frequency_sensor.update()
        
        if "left_gripper" in self._joint_sensor_vla:
            left_gripper_width = list(self._joint_sensor_vla["left_gripper"].position)[0]/1000.0
        else:
            left_gripper_width = -1
        if "right_gripper" in self._joint_sensor_vla:
            right_gripper_width = list(self._joint_sensor_vla["right_gripper"].position)[0]/1000.0
        else:
            right_gripper_width = -1

        q_wholebody = (
            list(self._joint_sensor_vla["leg"].position) +
            list(self._joint_sensor_vla["head"].position) +
            list(self._joint_sensor_vla["left_arm"].position) +
            [left_gripper_width] +
            list(self._joint_sensor_vla["right_arm"].position) +
            [right_gripper_width]+
            self.chassis_pos
        )

        self.cal_T_count = self.cal_T_count + 1
        if self.cal_T_count == self.args.cal_T_count:
            self.cal_T_count = 0
            [self.left_end_T, self.right_end_T] = self.tool_fk.get_T_frame(np.array(q_wholebody[0:14]+q_wholebody[15:22]),["left_arm_end_effector_mount_link","right_arm_end_effector_mount_link"])
            if self.right_end_T[0, 3] > self.right_end_T_deepest[0, 3]:
                self.right_end_T_deepest = self.right_end_T
            if self.left_end_T[0, 3] > self.left_end_T_deepest[0, 3]:
                self.left_end_T_deepest = self.left_end_T

        self.pose_buffer.appendleft(q_wholebody)
        self.pose_timestamp_buffer.appendleft(float(message.header.timestamp.sec) + float(message.header.timestamp.nanosec)/(1e9))

        if self.args.visualization:
            msg = {
                "type": "joint_q",  # 你自己定义
                "q": q_wholebody,  # 数据本体
                "time_stamp_joint": float(message.header.timestamp.sec) + float(message.header.timestamp.nanosec)/(1e9),
            }
            self.socket_current_state.send(json.dumps(msg).encode("utf-8"))

    #@profile_timeline(cat="sensor", name="GalbotInterface.odomCallback_vla", min_duration_ms=0.2)
    def odomCallback_vla(self,message):
        self.callback_frequency_odom.update()

        x = message.pose.pose.position.x
        y = message.pose.pose.position.y    
        z = message.pose.pose.orientation.z
        w = message.pose.pose.orientation.w
        self.chassis_pos = [x,y,math.atan2(z, w)*2]
        if self.last_chassis_pos is not None:
            self.chassis_pos[2] = self.align_rotation_angle(self.chassis_pos[2],self.last_chassis_pos[2])
        self.last_chassis_pos = self.chassis_pos

    #@profile_timeline(cat="util", name="GalbotInterface.align_rotation_angle", min_duration_ms=0.02)
    def align_rotation_angle(self,new,old):
        if new>old:
            while( abs(new - old) > 3.14159265358979323846):
                new = new - 2*np.pi
            return new
        else:
            while( abs(new - old) > 3.14159265358979323846):
                new = new + 2*np.pi
            return new

    #@profile_timeline(cat="image", name="GalbotInterface.ImageHeadLeftCallback", min_duration_ms=0.5)
    def ImageHeadLeftCallback(self, message):
        self._image_head_left = message.data
        self._image_head_left_timestamp = float(message.header.timestamp.sec) + float(message.header.timestamp.nanosec)/(1e9)
        self.callback_frequency_image_head_left.update()

        if message.data is None:
            self.logger.error("ImageHeadLeftCallback message.data is None")
        if self._image_info_head_left is None:
            self.m_robot_image_head_left_info_reader.SyncOnce()

    #@profile_timeline(cat="image", name="GalbotInterface.ImageHandLeftCallback", min_duration_ms=0.5)
    def ImageHandLeftCallback(self, message):
        self._image_hand_left = message.data
        self._image_hand_left_timestamp = float(message.header.timestamp.sec) + float(message.header.timestamp.nanosec)/(1e9)
        self.callback_frequency_image_hand_left.update()

        if message.data is None:
            self.logger.error("ImageHandLeftCallback message.data is None")
        if self._image_info_hand_left is None:
            self.m_robot_image_hand_left_info_reader.SyncOnce()

    #@profile_timeline(cat="image", name="GalbotInterface.ImageHandRightCallback", min_duration_ms=0.5)
    def ImageHandRightCallback(self, message):
        self._image_hand_right = message.data
        self._image_hand_right_timestamp = float(message.header.timestamp.sec) + float(message.header.timestamp.nanosec)/(1e9)
        self.callback_frequency_image_hand_right.update()

        if message.data is None:
            self.logger.error("ImageHandRightCallback message.data is None")
        if self._image_info_hand_right is None:
            self.m_robot_image_hand_right_info_reader.SyncOnce()

    #@profile_timeline(cat="image", name="GalbotInterface.ImageInfoHeadLeftCallback", min_duration_ms=0.2)
    def ImageInfoHeadLeftCallback(self, message):
        self._image_info_head_left = [message.roi.width, message.roi.height]

    #@profile_timeline(cat="image", name="GalbotInterface.ImageInfoHandLeftCallback", min_duration_ms=0.2)
    def ImageInfoHandLeftCallback(self, message):
        self._image_info_hand_left = [message.roi.width, message.roi.height]

    #@profile_timeline(cat="image", name="GalbotInterface.ImageInfoHandRightCallback", min_duration_ms=0.2)
    def ImageInfoHandRightCallback(self, message):
        self._image_info_hand_right = [message.roi.width, message.roi.height]

    #@profile_timeline(cat="image", name="GalbotInterface.get_image", min_duration_ms=0.05)
    def get_image(self, target):
        if target == "head_left":
            if self._image_head_left is None:
                return None, "head_left_img is None, "
            else:
                return self._image_head_left, ""
        elif target == "hand_right":
            if self._image_hand_right is None:
                return None, "hand_right_img is None, "
            else:
                return self._image_hand_right, ""
        elif target == "hand_left":
            if self._image_hand_left is None:
                return None, "hand_left_img is None, "
            else:
                return self._image_hand_left, ""
        else:
            return None, "get_image error input, "

    #@profile_timeline(cat="control", name="GalbotInterface.set_gripper_status", min_duration_ms=0.1)
    def set_gripper_status(
        self,
        width,  # m
        speed=1,
        force=0.5,
        gripper="left_gripper",
        threshold=0.005,  # 5mm
        timeout=10,
        frequency: float = 100.0,
        asynchronous=False,
        retry: int = 1,
    ):
        # width = max(min(width, self.gripper_stroke), 0.0)
        # width += self.min_gripper_width
        width = width * 1000  # m -> mm
        status, msg = self.set_etool_joints(
            "left" if "left" in gripper else "right",
            width,
            v=speed * 100,
            e=force,
            tool_type="gripper",
            target_time=100 / frequency,
            asynchronous=asynchronous,
            timeout=timeout,
        )
        if status:
            return True, msg
        else:
            logger.error(f"set gripper status failed: {msg}")
            return False, msg

    def set_etool_joints(
        self,
        side: str,
        q: float,
        v: float = 300.0,
        e: float = 50.0,
        tool_type: str = "gripper",
        target_time: float = 10,
        asynchronous=False,
        timeout: float = 15.0,
    ):
        """通用的工具设置方法"""
        if "left" in side:
            side = "left"
        elif "right" in side:
            side = "right"
        else:
            logger.error(f"Tool side '{side}' not supported. Use 'left' or 'right'.")
            return False, f"Tool side '{side}' not supported."

        group_name = f"{side}_{tool_type}"
        if tool_type == "suction_cup":
            e = q
            q = 0
        target = generate_target_etool(group_name, f"{group_name}_joint1", q, v, e, target_time)

        # Send command
        res = singorix_error_pb2.SingoriXError()
        self.m_wbcs_target_client.SendRequestWrapper(target, res)
        status = self.print_robot_error(res)
        if not status:
            msg = f"{group_name} send failed"
            logger.error(msg)
            return False, msg

        return status, f"set {group_name} joint angle"

    def print_robot_error(self, message):
        """Print robot error message"""
        if len(message.error_map) > 0:
            logger.error(f"Errors Timestamp: {message.header.timestamp.sec}.{message.header.timestamp.nanosec}")
            for error_key, error in message.error_map.items():
                logger.error(
                    f"Errors component: {error_key}, error code: {hex(error.error_code)}, description: {error.description}"
                )
            return False
        else:
            # logger.info("No errors.")
            return True
