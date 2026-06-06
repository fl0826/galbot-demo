"""
机器人位姿工具脚本
两个功能：
  1. get_current_pose() - 获取机器人当前 23 维位姿（init_pose 格式），可选保存为 json 文件
  2. reset_to_pose(pose23) - 把传入的 23 维位姿当作初始位姿，复位机器人过去

23 维 init_pose 结构（与 config/init_pose/*.json 一致）：
  index 0:5   leg        (5)
  index 5:7   head       (2)
  index 7:14  left_arm   (7)
  index 14    left_gripper
  index 15:22 right_arm  (7)
  index 22    right_gripper

用法：
  # 获取当前位姿并打印 + 保存到 my_pose.json
  python pose_tool.py get --out my_pose.json

  # 只打印当前位姿（不保存）
  python pose_tool.py get

  # 用某个 json 文件里的位姿去复位
  python pose_tool.py reset --pose-file my_pose.json

  # 复位调慢一点（运动时间6秒）
  python pose_tool.py reset --pose-file my_pose.json --move-time 6
"""

import sys
import os

# 抑制 NvMMLite 等底层 C 库的 stderr 输出
_devnull_fd = os.open(os.devnull, os.O_WRONLY)
os.dup2(_devnull_fd, 2)

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from args import Args
from galbot_control.galbot_control import GalbotControl
from tool.tool_shutdown import ShutdownTool
from tool.logger import LoggerManager
import argparse
import logging
import json
import time
import numpy as np
import copy


def _wait_sensor_ready(galbot, timeout=5.0):
    """轮询等待传感器就绪，返回是否就绪"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        joint_ok = galbot.galbot_interface._joint_sensor_vla is not None
        pose_ok = len(galbot.galbot_interface.pose_buffer) >= 2
        if joint_ok and pose_ok:
            return True
        time.sleep(0.05)
    return False


def get_current_pose(galbot, out_file=None):
    """
    功能1：获取机器人当前 23 维位姿（init_pose 格式）。
    返回 list[float] (23 维)，可选保存为 json 文件。
    """
    if not _wait_sensor_ready(galbot):
        print("[get] 传感器未就绪，无法获取位姿")
        return None

    cur = list(galbot.galbot_interface.pose_buffer[1])  # 26 维
    if len(cur) < 23:
        print(f"[get] pose_buffer 维度异常: {len(cur)}")
        return None

    pose23 = [float(x) for x in cur[0:23]]

    # 打印（按身体部位分组，方便看）
    print("\n========== 当前位姿 (23维 init_pose 格式) ==========")
    print(f"leg          (0:5)  : {pose23[0:5]}")
    print(f"head         (5:7)  : {pose23[5:7]}")
    print(f"left_arm     (7:14) : {pose23[7:14]}")
    print(f"left_gripper (14)   : {pose23[14]}")
    print(f"right_arm    (15:22): {pose23[15:22]}")
    print(f"right_gripper(22)   : {pose23[22]}")
    print("====================================================")
    print("init_pose 数组：")
    print(json.dumps(pose23, indent=2))

    if out_file:
        # 保存成和 config/init_pose/*.json 一样的结构，方便直接替换使用
        data = {
            "source": "captured_by_pose_tool",
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "init_pose": pose23,
        }
        with open(out_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        print(f"\n[get] 已保存到: {os.path.abspath(out_file)}")

    return pose23


def reset_to_pose(galbot, pose23, move_time=4):
    """
    功能2：把传入的 23 维位姿当作初始位姿，复位机器人过去。
    pose23: list[float]，23 维 init_pose 格式
    move_time: 运动时间（秒），越大越慢
    """
    if len(pose23) != 23:
        print(f"[reset] 位姿维度错误: 需要 23 维，实际 {len(pose23)}")
        return False

    if not _wait_sensor_ready(galbot):
        print("[reset] 传感器未就绪，无法复位")
        return False

    if (
        galbot.galbot_interface.last_chassis_pos is None
        and galbot.args.enable_chassis > 0
    ):
        print("[reset] 无法获取底盘位置")
        return False

    galbot.shutdown_event.clear()
    galbot.error_imformation = ""

    # 底盘保持当前位置
    chassis = list(galbot.galbot_interface.chassis_pos)

    # 第一行 [move_time] 是运动时间
    aim_pos = np.array([move_time] + list(pose23) + chassis).reshape(-1, 1)
    print(f"[reset] 复位到指定位姿（运动时间 {move_time}s）...")
    galbot.set_wholebody_angle_asynchronous(aim_pos, 0.03, 0.004)

    if galbot.error_imformation:
        print(f"[reset] 复位失败: {galbot.error_imformation}")
        return False
    print("[reset] 复位完成")
    return True


def _make_galbot():
    """创建 GalbotControl（位姿工具不连模型服务器，只读传感器/发关节指令）"""
    args = Args()
    args.has_init_action = True
    galbot = GalbotControl(args)
    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(galbot.shutdown)
    # 等底层连接就绪，连好了立即继续，最多等 3 秒兜底
    t0 = time.time()
    while time.time() - t0 < 3:
        if _wait_sensor_ready(galbot, timeout=0.05):
            break
        time.sleep(0.05)
    return galbot


if __name__ == "__main__":
    # 抑制推理日志终端输出，保留 ERROR
    LoggerManager.get_logger()
    for listener in LoggerManager._queue_listeners.values():
        for handler in listener.handlers:
            if isinstance(handler, logging.StreamHandler) and not isinstance(
                handler, logging.FileHandler
            ):
                handler.setLevel(logging.ERROR)

    parser = argparse.ArgumentParser(
        description="机器人位姿工具：获取当前位姿 / 用指定位姿复位"
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    # get 子命令
    p_get = sub.add_parser("get", help="获取当前机器人位姿")
    p_get.add_argument("--out", default=None, help="保存位姿到 json 文件（可选）")

    # reset 子命令
    p_reset = sub.add_parser("reset", help="用指定位姿复位")
    p_reset.add_argument(
        "--pose-file", required=True, help="位姿 json 文件（含 init_pose 字段）"
    )
    p_reset.add_argument(
        "--move-time", type=float, default=4, help="运动时间(秒)，越大越慢，默认4"
    )

    cli_args = parser.parse_args()

    galbot = _make_galbot()

    if cli_args.cmd == "get":
        get_current_pose(galbot, cli_args.out)

    elif cli_args.cmd == "reset":
        with open(cli_args.pose_file, "r", encoding="utf-8") as f:
            data = json.load(f)
        pose23 = data.get("init_pose")
        if not pose23:
            print(f"[reset] {cli_args.pose_file} 缺少 init_pose 字段")
            sys.exit(1)
        reset_to_pose(galbot, pose23, cli_args.move_time)
