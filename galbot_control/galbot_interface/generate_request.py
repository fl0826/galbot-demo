import os
import sys
import time
from typing import Dict, List, Any, Optional, Union, Tuple
import numpy as np

sys.path.append("/data/galbot/lib/python3.8.10")
sys.path.append("/data/galbot/lib/pw1/python3/dist-packages")
sys.path.append("/data/galbot/lib/pw1/python3/site-packages")

# Import protobuf messages
import galbot.singorix_proto.singorix_command_pb2 as singorix_command_pb2
import galbot.singorix_proto.singorix_controller_pb2 as singorix_controller_pb2
import galbot.singorix_proto.singorix_error_pb2 as singorix_error_pb2
import galbot.singorix_proto.singorix_info_pb2 as singorix_info_pb2
import galbot.singorix_proto.singorix_sensor_pb2 as singorix_sensor_pb2
import galbot.singorix_proto.singorix_target_pb2 as singorix_target_pb2

import embosa_python
import embosa_extend_node

m_leg_size = 5


def generate_target_head(q: List[float], target_time: float = 10):
    """Generate target for head joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["head"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    joint_size = 2
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"head_joint{j}")

    # 创建一个 group_command
    group_command = target_joint_traj.group_commands.add()
    group_command.time_from_start.sec = target_time

    # Add joint commands
    for j in range(joint_size):
        joint_command = group_command.joint_commands.add()
        joint_command.position = q[j]
        joint_command.velocity = 0.0

    return target


def generate_target_leg(q: List[float], target_time: float = 10):
    """Generate target for leg joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["leg"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    joint_size = m_leg_size
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"leg_joint{j}")

    # 创建一个 group_command
    group_command = target_joint_traj.group_commands.add()
    group_command.time_from_start.sec = target_time

    # Add joint commands
    for j in range(joint_size):
        joint_command = group_command.joint_commands.add()
        joint_command.position = q[j]
        joint_command.velocity = 0.0

    return target


def generate_target_left_arm(q: List[float], target_time: float = 10):
    """Generate target for left arm joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["left_arm"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    joint_size = 7
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"left_arm_joint{j}")

    # 创建一个 group_command
    group_command = target_joint_traj.group_commands.add()
    group_command.time_from_start.sec = target_time

    # Add joint commands
    for j in range(joint_size):
        joint_command = group_command.joint_commands.add()
        joint_command.position = q[j]
        joint_command.velocity = 0.0

    return target


def generate_target_right_arm(q: List[float], target_time: float = 10):
    """Generate target for right arm joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["right_arm"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    joint_size = 7
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"right_arm_joint{j}")

    # 创建一个 group_command
    group_command = target_joint_traj.group_commands.add()
    group_command.time_from_start.sec = target_time

    # Add joint commands
    for j in range(joint_size):
        joint_command = group_command.joint_commands.add()
        joint_command.position = q[j]
        joint_command.velocity = 0.0

    return target


def generate_target_etool(
    group_name: str,
    joint_name: str,
    q: float,
    v: float = 300.0,
    e: float = 50.0,
    target_time: float = 10,
):
    """Generate trajectory for gripper joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map[group_name]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    target_joint_traj.joint_names.append(joint_name)

    # 创建一个 group_command
    group_command = target_joint_traj.group_commands.add()
    group_command.time_from_start.sec = target_time

    # Add joint commands
    joint_command = group_command.joint_commands.add()
    joint_command.position = q
    joint_command.velocity = v
    joint_command.effort = e

    return target


def _merge_singorix_target(dest, src):
    """
    将 src 中的 group 和 task trajectory 合并到 dest 中
    :param dest: SingoriXTarget
    :param src: SingoriXTarget
    """
    for key, value in src.target_group_trajectory_map.items():
        dest.target_group_trajectory_map[key].CopyFrom(value)

    for key, value in src.target_task_trajectory_map.items():
        dest.target_task_trajectory_map[key].CopyFrom(value)


def generate_target_whole(q: List[float], target_time: float = 10):
    """Generate target for whole body joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # Split the joint positions for different chains
    leg_q = q[:m_leg_size]
    head_q = q[m_leg_size : m_leg_size + 2]
    left_arm_q = q[m_leg_size + 2 : m_leg_size + 9]
    right_arm_q = q[m_leg_size + 9 : m_leg_size + 16]

    # Generate targets for each chain
    leg_target = generate_target_leg(leg_q, target_time)
    _merge_singorix_target(target, leg_target)
    head_target = generate_target_head(head_q, target_time)
    _merge_singorix_target(target, head_target)
    left_arm_target = generate_target_left_arm(left_arm_q, target_time)
    _merge_singorix_target(target, left_arm_target)
    right_arm_target = generate_target_right_arm(right_arm_q, target_time)
    _merge_singorix_target(target, right_arm_target)

    return target


def generate_stop_command(group_name: str = "all"):
    """Generate stop command for a chain"""
    # Create SingoriXTarget object
    controllers = singorix_controller_pb2.SingoriXController()
    controller = controllers.ctrl_config_map[group_name]
    controller.ctrl_option = (
        singorix_controller_pb2.ControllerOption.CONTROLLER_OPTIONS_CONTROLER_STOP
    )
    return controllers


def generate_trajectory_head(trajectory, dt=0.008):
    """Generate trajectory for head joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["head"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    joint_size = 2
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"head_joint{j}")

    # Add trajectory points
    time = 0.0
    for state in trajectory:
        group_command = target_joint_traj.group_commands.add()
        time += dt
        group_command.time_from_start.sec = time

        # Add joint commands for this point
        for j in range(joint_size):
            joint_command = group_command.joint_commands.add()
            joint_command.position = state[j]
            joint_command.velocity = 0.0

    return target


def generate_trajectory_leg(trajectory, dt=0.008):
    """Generate trajectory for leg joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["leg"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    joint_size = m_leg_size
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"leg_joint{j}")

    # Add trajectory points
    time = 0.0
    for state in trajectory:
        group_command = target_joint_traj.group_commands.add()
        time += dt
        group_command.time_from_start.sec = time

        # Add joint commands for this point
        for j in range(joint_size):
            joint_command = group_command.joint_commands.add()
            joint_command.position = state[j]
            joint_command.velocity = 0.0

    return target


def generate_trajectory_left_arm(trajectory, dt=0.008):
    """Generate trajectory for left arm joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["left_arm"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    joint_size = 7
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"left_arm_joint{j}")

    # Add trajectory points
    time = 0.0
    for state in trajectory:
        group_command = target_joint_traj.group_commands.add()
        time += dt
        group_command.time_from_start.sec = time

        # Add joint commands for this point
        for j in range(joint_size):
            joint_command = group_command.joint_commands.add()
            joint_command.position = state[j]
            joint_command.velocity = 0.0

    return target


def generate_trajectory_right_arm(trajectory, dt=0.008):
    """Generate trajectory for right arm joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # 构建 TargetGroupTrajectory
    target_joint_traj = target.target_group_trajectory_map["right_arm"]

    # 设置 target config（假设这些变量已定义）
    target_joint_traj.target_config.target_data = (
        singorix_target_pb2.TargetData.TARGET_DATA_DEFAULT
    )
    target_joint_traj.target_config.target_type = (
        singorix_target_pb2.TargetType.TARGET_TYPE_DEFAULT
    )
    target_joint_traj.target_config.target_sampling = (
        singorix_target_pb2.TargetSampling.TARGET_SAMPLING_DEFAULT
    )
    target_joint_traj.target_config.target_priority = 1

    # Set joint parameters
    joint_size = 7
    for j in range(1, joint_size + 1):
        target_joint_traj.joint_names.append(f"right_arm_joint{j}")

    # Add trajectory points
    time = 0.0
    for state in trajectory:
        group_command = target_joint_traj.group_commands.add()
        time += dt
        group_command.time_from_start.sec = time

        # Add joint commands for this point
        for j in range(joint_size):
            joint_command = group_command.joint_commands.add()
            joint_command.position = state[j]
            joint_command.velocity = 0.0

    return target


def generate_trajectory_whole(trajectory, dt=0.008):
    """Generate trajectory for whole body joints"""
    # Create SingoriXTarget object
    target = singorix_target_pb2.SingoriXTarget()

    # Precompute chain trajectories for efficiency
    trajectory_arr = np.array(trajectory)
    trajectory_leg = trajectory_arr[..., :m_leg_size]
    trajectory_head = trajectory_arr[..., m_leg_size : m_leg_size + 2]
    trajectory_left_arm = trajectory_arr[..., m_leg_size + 2 : m_leg_size + 9]
    trajectory_right_arm = trajectory_arr[..., m_leg_size + 9 : m_leg_size + 16]

    # Generate trajectories for each chain
    leg_target = generate_trajectory_leg(trajectory_leg, dt)
    _merge_singorix_target(target, leg_target)
    head_target = generate_trajectory_head(trajectory_head, dt)
    _merge_singorix_target(target, head_target)
    left_arm_target = generate_trajectory_left_arm(trajectory_left_arm, dt)
    _merge_singorix_target(target, left_arm_target)
    right_arm_target = generate_trajectory_right_arm(trajectory_right_arm, dt)
    _merge_singorix_target(target, right_arm_target)

    return target
