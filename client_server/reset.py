"""
独立复位脚本 - 把机器人移动到初始位姿后退出
用法：
  python reset.py              # 默认用桌面位姿 (stand)
  python reset.py --pose floor # 用地面/垃圾袋位姿 (zhiyuan_pick_trash.json)
  python reset.py --pose table # 用桌面位姿 (zhiyuan_pick_trash_stand.json)
  python reset.py --pose-file config/init_pose/xxx.json   # 自定义位姿文件
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
import time


# 预设位姿文件
POSE_PRESETS = {
    "floor": "config/init_pose/zhiyuan_pick_trash.json",        # 地面 / 垃圾袋
    "table": "config/init_pose/zhiyuan_pick_trash_stand.json",  # 桌面
}


if __name__ == "__main__":
    # 抑制推理日志的终端输出，保留 ERROR
    LoggerManager.get_logger()
    for listener in LoggerManager._queue_listeners.values():
        for handler in listener.handlers:
            if isinstance(handler, logging.StreamHandler) and not isinstance(handler, logging.FileHandler):
                handler.setLevel(logging.ERROR)

    parser = argparse.ArgumentParser(description="机器人复位脚本")
    parser.add_argument("--pose", choices=POSE_PRESETS.keys(), default="table",
                        help="位姿预设: floor=地面/垃圾袋, table=桌面 (默认: table)")
    parser.add_argument("--pose-file", default=None,
                        help="自定义位姿文件路径（覆盖 --pose）")
    cli_args = parser.parse_args()

    pose_file = cli_args.pose_file or POSE_PRESETS[cli_args.pose]

    args = Args()
    args.init_pose_file = pose_file
    args.has_init_action = True  # 必须为 True 才会真正移动手臂

    print(f"[reset] 初始位姿文件: {pose_file}")
    print("[reset] 初始化 GalbotControl...")
    galbot = GalbotControl(args)

    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(galbot.shutdown)

    # 等待传感器就绪
    print("[reset] 等待传感器就绪...")
    time.sleep(3)

    print("[reset] 开始复位...")
    galbot.move_to_init_pose_wholebody()

    if galbot.error_imformation:
        print(f"[reset] 失败: {galbot.error_imformation}")
        sys.exit(1)

    print("[reset] 复位完成")
