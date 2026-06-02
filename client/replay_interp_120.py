"""
================================================================
  数据回放插值版 —— 把 LeRobot parquet 轨迹插值到 120FPS 后回放到真机
================================================================

【用法】
  python replay_interp_100.py
  python replay_interp_100.py --parquet xxx/episode_000000.parquet
  python replay_interp_100.py --speed 0.5
  python replay_interp_100.py --start-frame 0 --end-frame 100

【参数】
  --parquet     parquet 路径；不传则用脚本里的 PARQUET_PATH
  --src-fps     原始轨迹帧率，默认 30
  --target-fps  插值后发布帧率，默认 120
  --speed       速度倍率，1.0原速 / 0.5慢放
  --start-frame / --end-frame   原始轨迹帧区间
  --move-time   复位到第一帧的时间(秒)，默认 4

【说明】
  1. 先按 action(38) 原始数据做线性插值，默认 30FPS -> 120FPS。
  2. 再按 target-fps 逐帧发布单列 mat。
  3. 夹爪、关节、底盘里程计字段都一起线性插值。
  4. 第一次上真机务必 --speed 0.3 慢放 + 人在旁边随时急停。
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


PARQUET_PATH = "/home/galbot/vla_client/episode_000000.parquet"


def action38_to_joints23(a):
    """数据集 38 维 action -> 机器人 23 维 init_pose 格式（夹爪 mm->m）"""
    a = np.asarray(a, dtype=float)
    return (
        list(a[16:21])
        + list(a[21:23])
        + list(a[8:15])
        + [a[15] / 1000.0]
        + list(a[0:7])
        + [a[7] / 1000.0]
    )


def action38_to_chassis(a):
    """数据集 38 维 action -> 底盘 [x, y, yaw]（里程计字段）"""
    a = np.asarray(a, dtype=float)
    return [a[31], a[32], math.atan2(a[33], a[34]) * 2]


def interpolate_actions(actions, src_fps=30, target_fps=100):
    """把原始 action 序列按时间线性插值到 target_fps。"""
    actions_np = np.asarray(actions, dtype=float)
    if len(actions_np) <= 1:
        return actions_np

    duration = (len(actions_np) - 1) / float(src_fps)
    src_t = np.arange(len(actions_np), dtype=float) / float(src_fps)
    target_count = int(round(duration * target_fps)) + 1
    target_t = np.arange(target_count, dtype=float) / float(target_fps)
    target_t[-1] = min(target_t[-1], src_t[-1])

    out = np.empty((target_count, actions_np.shape[1]), dtype=float)
    for dim in range(actions_np.shape[1]):
        out[:, dim] = np.interp(target_t, src_t, actions_np[:, dim])
    return out


def wait_sensor_ready(galbot, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        joint_ok = galbot.galbot_interface._joint_sensor_vla is not None
        pose_ok = len(galbot.galbot_interface.pose_buffer) >= 2
        chassis_ok = (
            galbot.galbot_interface.last_chassis_pos is not None
            or galbot.args.enable_chassis <= 0
        )
        if joint_ok and pose_ok and chassis_ok:
            return True
        time.sleep(0.05)
    return False


def replay(galbot, df, src_fps, target_fps, speed, start_frame, end_frame, move_time):
    n = len(df)
    end_frame = n if (end_frame is None or end_frame > n) else end_frame
    raw_actions = list(df["action"].iloc[max(0, start_frame):end_frame])
    if not raw_actions:
        print("[replay] 没有可回放的帧")
        return

    print(f"[replay] 原始帧数: {len(raw_actions)}  src_fps={src_fps}")
    actions = interpolate_actions(raw_actions, src_fps=src_fps, target_fps=target_fps)
    print(f"[replay] 插值后帧数: {len(actions)}  target_fps={target_fps}")

    print("[replay] 等待传感器就绪...")
    if not wait_sensor_ready(galbot, timeout=5):
        print("[replay] 传感器未就绪，无法回放")
        return

    print(f"[replay] 复位到第一帧（{move_time}s）...")
    galbot.shutdown_event.clear()
    galbot.error_imformation = ""
    aim = np.array([move_time] + action38_to_joints23(actions[0]) + list(galbot.galbot_interface.chassis_pos)).reshape(-1, 1)
    galbot.set_wholebody_angle_asynchronous(aim, 0.03, 0.004)
    if galbot.error_imformation:
        print(f"[replay] 复位失败: {galbot.error_imformation}")
        return
    print("[replay] 复位完成，开始回放...")

    frame_dt = 1.0 / target_fps / max(speed, 1e-6)
    total = len(actions)
    t_start = time.perf_counter()
    lag_warn_count = 0

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
        mat[2 + 14, 0] *= 1000
        mat[2 + 22, 0] *= 1000
        galbot.embosa_vla_publisher.pub_mat(mat)

        if i % 20 == 0 or i == total - 1:
            pct = (i + 1) * 100 / total
            sys.stdout.write(f"\r[replay] 进度 {pct:5.1f}% ({i+1}/{total})")
            sys.stdout.flush()

        next_t = t_start + (i + 1) * frame_dt
        sleep_t = next_t - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)
        elif sleep_t < -0.05 and lag_warn_count < 10:
            lag_warn_count += 1
            print(f"\n[replay] 警告：发布落后 {-sleep_t:.3f}s，100FPS 可能过高")

    sys.stdout.write("\n")
    print("[replay] 回放结束")


if __name__ == "__main__":
    LoggerManager.get_logger()
    for listener in LoggerManager._queue_listeners.values():
        for h in listener.handlers:
            if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
                h.setLevel(logging.ERROR)

    parser = argparse.ArgumentParser(description="采集数据插值到120FPS后回放到真机")
    parser.add_argument("--parquet", default=None, help="parquet 路径（不传用 PARQUET_PATH）")
    parser.add_argument("--src-fps", type=int, default=30, help="原始轨迹帧率，默认30")
    parser.add_argument("--target-fps", type=int, default=120, help="插值后发布帧率，默认120")
    parser.add_argument("--speed", type=float, default=1.0, help="速度倍率")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--move-time", type=float, default=4, help="复位到第一帧时间(秒)")
    cli = parser.parse_args()

    pq = cli.parquet or PARQUET_PATH
    if not os.path.exists(pq):
        print(f"[replay] parquet 不存在: {pq}")
        sys.exit(1)
    print(f"[replay] 数据文件: {pq}")
    df = pd.read_parquet(pq)
    print(f"[replay] 总帧数: {len(df)}  src_fps={cli.src_fps} target_fps={cli.target_fps} speed={cli.speed}")

    galbot = GalbotControl(Args())
    tool_shutdown = ShutdownTool()
    tool_shutdown.on_shutdown(galbot.shutdown)
    time.sleep(3)

    replay(
        galbot,
        df,
        cli.src_fps,
        cli.target_fps,
        cli.speed,
        cli.start_frame,
        cli.end_frame,
        cli.move_time,
    )
