"""
================================================================
  数据回放(replay)脚本 —— 把采集的 LeRobot parquet 回放到真机
================================================================

【用法】
  python replay.py                                  # 用下方 PARQUET_PATH
  python replay.py --parquet xxx/episode_000000.parquet
  python replay.py --speed 0.5                      # 慢放
  python replay.py --start-frame 0 --end-frame 100  # 只放一段

【参数】
  --parquet     parquet 路径；不传则用脚本里的 PARQUET_PATH
                （命令行 --parquet 优先级更高）
  --fps         帧率，默认 30
  --speed       速度倍率，1.0原速 / 0.5慢放
  --start-frame / --end-frame   回放帧区间
  --move-time   复位到第一帧的时间(秒)，默认 4

【注意事项】
  1. 第一次上真机务必 --speed 0.3 慢放 + 人在旁边随时急停。
  2. 只需要 episode_000000.parquet 一个文件即可，不需要 videos/meta。
  3. 底盘按里程计绝对坐标回放，若底盘不动多为底层模式问题，与本脚本无关。
  4. 依赖: pip install pandas pyarrow

【数据维度映射】
  数据集 action(38) 前23维有用:
    0:7 右臂 | 7 右爪(mm) | 8:15 左臂 | 15 左爪(mm) | 16:21 腰 | 21:23 头 | 31:35 里程计
  机器人 init_pose(23):
    0:5 腰 | 5:7 头 | 7:14 左臂 | 14 左爪 | 15:22 右臂 | 22 右爪  (夹爪 mm/1000=m)
================================================================
"""
import sys
import os

# 抑制底层 C 库(NvMMLite 等)的 stderr；调试看报错时把下行注释掉
# os.dup2(os.open(os.devnull, os.O_WRONLY), 2)

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from args import Args
from galbot_control.galbot_control import GalbotControl
from tool.tool_shutdown import ShutdownTool
from tool.logger import LoggerManager
import argparse
import logging
import time
import math
import numpy as np
import pandas as pd


# 默认 parquet 路径（命令行 --parquet 会覆盖它）
PARQUET_PATH = "/home/galbot/vla_client/episode_000000.parquet"


def action38_to_joints23(a):
    """数据集 38 维 action -> 机器人 23 维 init_pose 格式（夹爪 mm->m）"""
    a = np.asarray(a, dtype=float)
    return (
        list(a[16:21])          # 0:5  腰
        + list(a[21:23])        # 5:7  头
        + list(a[8:15])         # 7:14 左臂
        + [a[15] / 1000.0]      # 14   左爪
        + list(a[0:7])          # 15:22 右臂
        + [a[7] / 1000.0]       # 22   右爪
    )


def action38_to_chassis(a):
    """数据集 38 维 action -> 底盘 [x, y, yaw]（里程计字段）"""
    a = np.asarray(a, dtype=float)
    return [a[31], a[32], math.atan2(a[33], a[34]) * 2]


def replay(galbot, df, fps, speed, start_frame, end_frame, move_time):
    n = len(df)
    end_frame = n if (end_frame is None or end_frame > n) else end_frame
    actions = list(df["action"].iloc[max(0, start_frame):end_frame])
    if not actions:
        print("[replay] 没有可回放的帧")
        return

    # 等传感器就绪
    t0 = time.time()
    while time.time() - t0 < 5:
        if galbot.galbot_interface._joint_sensor_vla is not None and len(galbot.galbot_interface.pose_buffer) >= 2:
            break
        time.sleep(0.05)
    if galbot.galbot_interface.last_chassis_pos is None and galbot.args.enable_chassis > 0:
        print("[replay] 无法获取底盘位置")
        return

    # 复位到第一帧
    print(f"[replay] 复位到第一帧（{move_time}s）...")
    galbot.shutdown_event.clear()
    galbot.error_imformation = ""
    aim = np.array([move_time] + action38_to_joints23(actions[0]) + list(galbot.galbot_interface.chassis_pos)).reshape(-1, 1)
    galbot.set_wholebody_angle_asynchronous(aim, 0.03, 0.004)
    if galbot.error_imformation:
        print(f"[replay] 复位失败: {galbot.error_imformation}")
        return
    print("[replay] 复位完成，开始回放...")

    # 逐帧发布：每帧构造单列 mat，按 fps 节拍发给底层
    frame_dt = 1.0 / fps / max(speed, 1e-6)
    total = len(actions)
    t_start = time.perf_counter()

    for i, a in enumerate(actions):
        if galbot.shutdown_event.is_set():
            print("\n[replay] 被中断")
            break

        pose23 = action38_to_joints23(a)
        chassis = action38_to_chassis(a)
        mat = np.zeros((2 + 26, 1))
        mat[1, 0] = frame_dt
        mat[2:2 + 23, 0] = pose23
        mat[2 + 23:2 + 26, 0] = chassis
        mat[2 + 14, 0] *= 1000   # 左爪 m->mm
        mat[2 + 22, 0] *= 1000   # 右爪 m->mm
        galbot.embosa_vla_publisher.pub_mat(mat)

        pct = (i + 1) * 100 / total
        sys.stdout.write(f"\r[replay] 进度 {pct:5.1f}% ({i+1}/{total})")
        sys.stdout.flush()

        next_t = t_start + (i + 1) * frame_dt
        sleep_t = next_t - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)

    sys.stdout.write("\n")
    print("[replay] 回放结束")


if __name__ == "__main__":
    LoggerManager.get_logger()
    for listener in LoggerManager._queue_listeners.values():
        for h in listener.handlers:
            if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
                h.setLevel(logging.ERROR)

    parser = argparse.ArgumentParser(description="采集数据回放到真机")
    parser.add_argument("--parquet", default=None, help="parquet 路径（不传用 PARQUET_PATH）")
    parser.add_argument("--fps", type=int, default=30, help="帧率，默认30")
    parser.add_argument("--speed", type=float, default=1.0, help="速度倍率")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--move-time", type=float, default=4, help="复位到第一帧时间(秒)")
    cli = parser.parse_args()

    pq = cli.parquet or PARQUET_PATH   # 命令行优先
    if not os.path.exists(pq):
        print(f"[replay] parquet 不存在: {pq}")
        sys.exit(1)
    print(f"[replay] 数据文件: {pq}")
    df = pd.read_parquet(pq)
    print(f"[replay] 总帧数: {len(df)}  fps={cli.fps} speed={cli.speed}")

    galbot = GalbotControl(Args())
    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(galbot.shutdown)
    time.sleep(3)

    replay(galbot, df, cli.fps, cli.speed, cli.start_frame, cli.end_frame, cli.move_time)
