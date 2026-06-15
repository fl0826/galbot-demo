#!/usr/bin/env python3

import time
import threading
import logging
from typing import Dict, List, Any, Optional, Union, Tuple
import sys
import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime

# Import embosa middleware
import embosa_python
import embosa_extend_node

from galbot.core_proto.header_pb2 import Header
from galbot.core_proto.wrapper_pb2 import StringValue
from galbot.sensor_proto.joint_pb2 import JointSensor
from galbot.navigation_proto.odometry_pb2 import Odometry
from galbot.singorix_proto.singorix_target_pb2 import SingoriXTarget, TargetData, TargetType, TargetSampling
from galbot.singorix_proto.singorix_error_pb2 import SingoriXError
from galbot.singorix_proto.singorix_info_pb2 import WBCInfo, WBCSInfo
from galbot.singorix_proto.singorix_sensor_pb2 import SingoriXSensor
from galbot.singorix_proto.singorix_command_pb2 import GroupCommand, TaskCommand
from galbot.singorix_proto.singorix_controller_pb2 import SingoriXController, ControllerAvailable, ControllerOption


def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角(roll, pitch, yaw)转换为四元数(x, y, z, w)
    角度单位：弧度
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    # 四元数 (x, y, z, w)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return (x, y, z, w)

def quaternion_to_euler(x, y, z, w):
    """
    将四元数(x, y, z, w)转换为欧拉角(roll, pitch, yaw)
    返回弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用90度如果超出范围
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)


# 在模块级别配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)

class SingorixInterface:
    def __init__(self, node=None):
        try:
            self.logger = logging.getLogger(__name__)
            self.target_id = os.path.splitext(os.path.basename(sys.argv[0]))[0] or "wbcs_test"

            self.robot_error = None
            self.robot_error_lock = threading.Lock()

            self.robot_sensor = None
            self.robot_sensor_lock = threading.Lock()

            self.wbc_info = None
            self.wbc_info_lock = threading.Lock()


            if embosa_python.GetState() == embosa_python.STATE_UNINITIALIZED:
                embosa_python.EmbosaInit()

            # Create DDS node
            self.embosa_node = node
            if not self.embosa_node:
                self.embosa_node = embosa_python.CreateNode("singorix_interface")
                if not self.embosa_node:
                    raise RuntimeError("Failed to create DDS node")

            # Configure QoS parametersrobot_error
            self.qos = embosa_python.Qos()

            self.qos_reader = embosa_python.Qos()
            # self.qos_reader.intra_core_qos.qos_callback_mode_policy = embosa_python.CALLBACK_SYNC_MODE
            # self.qos_reader.intra_core_qos.qos_sync_callback_policy.sub_sync_mode = embosa_python.SYNCHRONOUS_SUB_MODE
            # self.qos_reader.intra_core_qos.qos_sync_callback_policy.queue_depth = 1
            self.qos_client = embosa_python.RpcQos()
            self.qos_client.intra_core_rpc_qos.transport_type = embosa_python.LARGE_DATA_TRANSPORT

            self.robot_error_reader = self.embosa_node.CreateSerializationReader(
                SingoriXError,
                "singorix/wbcs/error",
                self.robot_error_callback,
                self.qos_reader
            )
            if self.robot_error_reader is None:
                self.logger.error("Create robot_error_reader fail")
                raise RuntimeError(
                    "Failed to create robot_error_reader"
                )


            self.robot_sensor_reader = self.embosa_node.CreateSerializationReader(
                SingoriXSensor,
                "singorix/wbcs/sensor",
                self.robot_sensor_callback,
                self.qos_reader
            )
            if self.robot_sensor_reader is None:
                self.logger.error("Create robot_sensor_reader fail")
                raise RuntimeError(
                    "Failed to create robot_sensor_reader"
                )

            self.wbc_info_reader = self.embosa_node.CreateSerializationReader(
                WBCInfo,
                "singorix/wbcs/wbc_info",
                self.wbc_info_callback,
                self.qos_reader
            )
            if self.wbc_info_reader is None:
                self.logger.error("Create wbc_info_reader fail")
                raise RuntimeError(
                    "Failed to create wbc_info_reader"
                )


            self.wbcs_target_writer = self.embosa_node.CreateSerializationWriter(SingoriXTarget, "singorix/wbcs/target", self.qos)

            self.wbcs_target_client = self.embosa_node.CreateSerializationClient(SingoriXTarget, SingoriXError, "singorix/wbcs/target_server", self.qos_client)
            self.wbcs_controller_client = self.embosa_node.CreateSerializationClient(SingoriXController, SingoriXError, "singorix/wbcs/controller_server", self.qos_client)
            self.wbcs_controller_available = self.embosa_node.CreateSerializationClient(StringValue, ControllerAvailable, "singorix/wbcs/controller_available", self.qos_client)

            self.wbcs_target_client.WaitForServerConnected()
            self.wbcs_controller_client.WaitForServerConnected()
            self.wbcs_controller_available.WaitForServerConnected()

            self.logger.info("SingorixInterface created successfully!")

        except Exception as e:
            self.logger.error(f"Exception during SingorixClient construction: {str(e)}")
            raise

    def robot_error_callback(self, robot_error):
        if len(robot_error.error_map):
            with self.robot_error_lock:
                self.robot_error = robot_error.error_map
        else:
            self.robot_error = None

    def robot_sensor_callback(self, robot_sensor):
        with self.robot_sensor_lock:
            self.robot_sensor = robot_sensor

    def wbc_info_callback(self, wbc_info):
        with self.wbc_info_lock:
            self.wbc_info = wbc_info

    def _stamp_target_config(self, target_config, target_ts_ns: int):
        target_config.target_id = self.target_id
        target_config.target_ts.sec = target_ts_ns // 1_000_000_000
        target_config.target_ts.nanosec = target_ts_ns % 1_000_000_000

    def _stamp_target_tracking_fields(self, target_proto: SingoriXTarget):
        target_ts_ns = time.time_ns()
        for traj in target_proto.target_group_trajectory_map.values():
            self._stamp_target_config(traj.target_config, target_ts_ns)
        for traj in target_proto.target_task_trajectory_map.values():
            self._stamp_target_config(traj.target_config, target_ts_ns)

    def _publish_target(self, target_proto: SingoriXTarget):
        self._stamp_target_tracking_fields(target_proto)
        self.wbcs_target_writer.Publish(target_proto)

    def _send_target_request(self, target_proto: SingoriXTarget, res: SingoriXError):
        self._stamp_target_tracking_fields(target_proto)
        self.wbcs_target_client.SendRequestWrapper(target_proto, res)

    # def read_robot_error(self):
    #     self.robot_error_reader.SyncOnce()

    # def read_robot_sensor(self):
    #     self.robot_sensor_reader.SyncOnce()

    # def read_wbc_info(self):
    #     self.wbc_info_reader.SyncOnce()

    def get_robot_error(self):
        with self.robot_error_lock:
            return self.robot_error

    def get_robot_sensor(self):
        with self.robot_sensor_lock:
            return self.robot_sensor

    def get_wbc_info(self):
        with self.wbc_info_lock:
            return self.wbc_info

    def get_group_joint(self, group):
        with self.robot_sensor_lock:
            return self.robot_sensor.joint_sensor_map[group]

    def get_group_joint_position(self, group):
        with self.robot_sensor_lock:
            return self.robot_sensor.joint_sensor_map[group].position

    def get_group_joint_velocity(self, group):
        with self.robot_sensor_lock:
            return self.robot_sensor.joint_sensor_map[group].velocity

    def get_group_joint_effort(self, group):
        with self.robot_sensor_lock:
            return self.robot_sensor.joint_sensor_map[group].effort

    def get_odometer(self):
        with self.wbc_info_lock:
            odom_proto = self.wbc_info.fusion_info.state_frame_map["wheel_odometer/base_link"]
            x = odom_proto.pose_msg.pose.position.x
            y = odom_proto.pose_msg.pose.position.y
            rx = odom_proto.pose_msg.pose.orientation.x
            ry = odom_proto.pose_msg.pose.orientation.y
            rz = odom_proto.pose_msg.pose.orientation.z
            rw = odom_proto.pose_msg.pose.orientation.w
            roll, pitch, yaw = quaternion_to_euler(rx, ry, rz, rw)
            return (x, y, yaw)

    def stop(self, group="all"):
        controllers = SingoriXController()
        controller = controllers.ctrl_config_map[group]
        controller.ctrl_option = ControllerOption.CONTROLLER_OPTIONS_CONTROLER_STOP
        res = SingoriXError()
        self.wbcs_controller_client.SendRequestWrapper(controllers, res)

    def switch(self, group: str, ctrl_name: str):
        controllers = SingoriXController()
        controller = controllers.ctrl_config_map[group]
        controller.ctrl_name = ctrl_name
        controller.ctrl_option = ControllerOption.CONTROLLER_OPTIONS_CONTROLER_SWITCH
        res = SingoriXError()
        self.wbcs_controller_client.SendRequestWrapper(controllers, res)

    def pub_chassis_pose(self, x: float, y: float, yaw: float, time: float = 0):
        target_proto = SingoriXTarget()
        tt = target_proto.target_task_trajectory_map["chassis"]
        tt.target_config.target_data = TargetData.TARGET_DATA_DEFAULT
        tt.target_config.target_type = TargetType.TARGET_TYPE_PROVERRIDE
        tt.target_config.target_sampling = TargetSampling.TARGET_SAMPLING_DEFAULT
        tt.group_names.extend(["chassis"])
        tt.subtask_names.extend(["123"])
        tc = tt.task_commands.add()
        tc.time_from_start.sec = time
        stc = tc.subtask_commands.add()
        stc.header.frame_id = "abs"
        stc.body_frame_id = "base_link"
        stc.reference_frame_id = "odom"
        stc.pose_msg.pose.position.x = x
        stc.pose_msg.pose.position.y = y
        rx, ry, rz, rw = euler_to_quaternion(0, 0, yaw)
        stc.pose_msg.pose.orientation.x = rx
        stc.pose_msg.pose.orientation.y = ry
        stc.pose_msg.pose.orientation.z = rz
        stc.pose_msg.pose.orientation.w = rw
        self._publish_target(target_proto)

    def pub_chassis_twist(self, vx: float, vy: float, vyaw: float, time: float = 0):
        target_proto = SingoriXTarget()
        tt = target_proto.target_task_trajectory_map["chassis"]
        tt.target_config.target_data = TargetData.TARGET_DATA_DEFAULT
        tt.target_config.target_type = TargetType.TARGET_TYPE_PROVERRIDE
        tt.target_config.target_sampling = TargetSampling.TARGET_SAMPLING_DIRECT_PASS
        tt.group_names.extend(["chassis"])
        tt.subtask_names.extend(["123"])
        tc = tt.task_commands.add()
        tc.time_from_start.sec = time
        stc = tc.subtask_commands.add()
        stc.header.frame_id = "rel"
        stc.body_frame_id = "base_link"
        stc.reference_frame_id = "base_link"
        stc.twist_msg.twist.linear.x = vx
        stc.twist_msg.twist.linear.y = vy
        stc.twist_msg.twist.angular.z = vyaw
        self._publish_target(target_proto)


    # point: position->[group][joint], time->[group], group_names->[group], joint_names_list->[group][joint]
    def pub_groups_position(self, position: List[List[float]], time: List[float], group_names: List[str], joint_names_list: List[List[str]]):
        if len(group_names) != len(joint_names_list):
            print("size mismatch for group_names and joint_names_list")
            return False
        target_proto = SingoriXTarget()
        for group_idx in range(len(group_names)):
            gt = target_proto.target_group_trajectory_map[group_names[group_idx]]
            gt.target_config.target_data = TargetData.TARGET_DATA_DEFAULT
            gt.target_config.target_type = TargetType.TARGET_TYPE_PROVERRIDE
            gt.target_config.target_sampling = TargetSampling.TARGET_SAMPLING_DEFAULT
            gt.joint_names.extend(joint_names_list[group_idx])
            gc = gt.group_commands.add()
            gc.time_from_start.sec = time[group_idx]
            for j in range(len(position[group_idx])):
                jc = gc.joint_commands.add()
                jc.position = position[group_idx][j]
                jc.velocity = 0
                jc.acceleration = 0
                jc.effort = 0
        self._publish_target(target_proto)

    # point: position->[joint]
    def pub_group_position(self, position: List[float], time: float, group: str):
        if group == "left_arm":
            joint_names = ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7"]
        elif group == "right_arm":
            joint_names = ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7"]
        elif group == "leg4":
            group = "leg"
            joint_names = ["leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4"]
        elif group == "leg":
            joint_names = ["leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4", "leg_joint5"]
        elif group == "head":
            joint_names = ["head_joint1", "head_joint2"]
        elif group == "torso":
            joint_names = ["torso_base_joint"]
        else:
            self.logger.error(f"group {group} unknown")
            return False

        if len(position) != len(joint_names):
            self.logger.error("size mismatch for position and joint_names")
            return False

        return self.pub_groups_position([position], [time], [group], [joint_names])


    # traj: position->[group][point][joint], time->[group][point], group_names->[group], joint_names_list->[group][joint]
    def set_groups_positions(self, position: List[List[List[float]]], time: List[List[float]], group_names: List[str], joint_names_list: List[List[str]]):
        if len(group_names) != len(joint_names_list):
            self.logger.error("size mismatch for group_names and joint_names_list")
            return False
        target_proto = SingoriXTarget()
        for group_idx in range(len(group_names)):
            gt = target_proto.target_group_trajectory_map[group_names[group_idx]]
            gt.target_config.target_data = TargetData.TARGET_DATA_DEFAULT
            gt.target_config.target_type = TargetType.TARGET_TYPE_PROVERRIDE
            gt.target_config.target_sampling = TargetSampling.TARGET_SAMPLING_DEFAULT
            gt.joint_names.extend(joint_names_list[group_idx])
            for i in range(len(position[group_idx])):
                gc = gt.group_commands.add()
                gc.time_from_start.sec = time[group_idx][i]
                for j in range(len(position[group_idx][i])):
                    jc = gc.joint_commands.add()
                    jc.position = position[group_idx][i][j]
                    jc.velocity = 0
                    jc.acceleration = 0
                    jc.effort = 0
        res = SingoriXError()
        self._send_target_request(target_proto, res)

    # traj: position->[point][joint], time->[point]
    def set_group_positions(self, position: List[List[float]], time: List[float], group: str):
        if group == "left_arm":
            joint_names = ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7"]
        elif group == "right_arm":
            joint_names = ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7"]
        elif group == "leg4":
            group = "leg"
            joint_names = ["leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4"]
        elif group == "leg":
            joint_names = ["leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4", "leg_joint5"]
        elif group == "head":
            joint_names = ["head_joint1", "head_joint2"]
        elif group == "torso":
            joint_names = ["torso_base_joint"]
        else:
            self.logger.error(f"group {group} unknown")
            return False

        if len(position[0]) != len(joint_names):
            self.logger.error("size mismatch for position and joint_names")
            return False

        return self.set_groups_positions([position], [time], [group], [joint_names])


# 全局单例实例
_singorix_interface_instance = None


def get_singorix_interface():
    """获取或创建 SingorixInterface 单例"""
    global _singorix_interface_instance
    if _singorix_interface_instance is None:
        _singorix_interface_instance = SingorixInterface()
    return _singorix_interface_instance


def handle_switch_cp(si=None):
    """切换到底盘位姿控制模式"""
    if si is None:
        si = get_singorix_interface()
    si.switch("chassis", "chassis_pose_ctrl")


def handle_switch_ct(si=None):
    """切换到底盘速度控制模式"""
    if si is None:
        si = get_singorix_interface()
    si.switch("chassis", "chassis_twist_ctrl")


def main():
    
    si = SingorixInterface()


    def clear_screen():
        """清空屏幕"""
        os.system('cls' if os.name == 'nt' else 'clear')

    def show_menu():
        """显示菜单选项"""
        print("=" * 50)
        print("命令执行器 v1.0")
        print("=" * 50)
        print("可用的命令:")
        print("help         - 显示帮助信息")
        print("clear        - 清空屏幕")
        print("err          - 读取错误信息")
        print("ss           - 读取传感器信息")
        print("info         - 读取wbc info")
        print("s            - 停止所有控制器")
        print("get          - 获取关节信息")
        print("getp         - 获取关节信位置")
        print("getv         - 获取关节信速度")
        print("gete         - 获取关节信力矩")
        print("odom         - 获取 odom 信息")
        print("pub_cp       - pub_chassis_pose 命令")
        print("pub_ct       - pub_chassis_twist 命令")
        print("pub          - pub group 命令")
        print("set          - set group 命令")
        print("switch_cc    - 切换底盘控制器")
        print("exit/quit/q  - 退出程序")
        print("-" * 50)

    def handle_help():
        """处理help命令"""
        show_menu()
        print("\n使用说明:")
        print("1. 直接输入命令名称执行对应操作")
        print("2. 命令不区分大小写")
        print("3. 输入q或exit或quit可以退出程序")


    def handle_print_error():
        print(si.get_robot_error())


    def handle_print_sensor():
        print(si.get_robot_sensor())

    def handle_print_wbc_info():
        print(si.get_wbc_info())


    def handle_print_group_joint():
        print(si.get_group_joint(input("group: ")))

    def handle_print_group_joint_position():
        print(si.get_group_joint_position(input("group: ")))

    def handle_print_group_joint_velocity():
        print(si.get_group_joint_velocity(input("group: ")))

    def handle_print_group_joint_effort():
        print(si.get_group_joint_effort(input("group: ")))

    def handle_print_odometer():
        print(si.get_odometer())

    def handle_switch_cp():
        si.switch("chassis", "chassis_pose_ctrl")

    def handle_switch_ct():
        si.switch("chassis", "chassis_twist_ctrl")

    def handle_pub():
        group = input("group: ").strip().lower()
        position = list(map(float, input("position: ").split()))
        time = float(input("time: "))
        si.pub_group_position(position=position, time=time, group=group)

    def handle_set():
        print("not support yet")

    def handle_pub_chassis_pose():
        x, y, yaw = map(float, input("x, y, yaw: ").split())
        si.pub_chassis_pose(x, y, yaw)

    def handle_pub_chassis_twist():
        vx, vy, vyaw = map(float, input("vx, vy, vyaw: ").split())
        si.pub_chassis_twist(vx, vy, vyaw)

    def handle_chassis():
        print("not support yet")


    def handle_unknown(command):
        """处理未知命令"""
        print(f"未知命令: '{command}'")
        print("输入 'help' 查看可用命令")


    """主函数"""
    clear_screen()


    # 命令映射字典
    commands = {
        'h': handle_help,
        'help': handle_help,
        'clear': clear_screen,
        'err': handle_print_error,
        'ss': handle_print_sensor,
        'info': handle_print_wbc_info,
        's': si.stop,
        'get': handle_print_group_joint,
        'getp': handle_print_group_joint_position,
        'getv': handle_print_group_joint_velocity,
        'gete': handle_print_group_joint_effort,
        'odom': handle_print_odometer,
        'pub_cp': handle_pub_chassis_pose,
        'pub_ct': handle_pub_chassis_twist,
        'switch_cp': handle_switch_cp,
        'switch_ct': handle_switch_ct,
        'pub': handle_pub,
        'set': handle_set,
        'chassis': handle_chassis,
    }

    # 退出命令
    exit_commands = ['exit', 'quit', 'q']
    
    print("欢迎使用命令执行器!")
    print("输入 'help' 查看可用命令")

    while True:
        try:
            # 获取用户输入
            user_input = input("\n命令> ").strip().lower()
            # 检查是否退出
            if user_input in exit_commands:
                print("感谢使用，再见!")
                break
            # 跳过空输入
            if not user_input:
                continue
            # 执行对应命令
            if user_input in commands:
                commands[user_input]()
            else:
                handle_unknown(user_input)
        except KeyboardInterrupt:
            print("\n检测到中断信号，正在退出...")
            break
        except EOFError:
            print("\n检测到EOF，正在退出...")
            break
        except Exception as e:
            print(f"发生错误: {e}")



if __name__ == "__main__":
    main()
