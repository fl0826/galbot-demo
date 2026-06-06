"""
一键拉取机器人日志脚本（支持密码自动登录）
从 hpu 和 xcu 两台服务器拉取指定时间段的日志，分别在服务器上打包，
再通过 SFTP 下载到本地，最后自动解压。

整个流程一个脚本搞定：
  本地运行 -> SSH 登录两台服务器 -> 远程筛选并 tar 打包 -> SFTP 下载 -> 本地解压

============================================================
                  使 用 说 明
============================================================
1. 修改下面【配置区】里的所有参数：
   - 两台服务器的 IP / 用户名 / 密码
   - 要拉取的日志时间段（日期 + 起止时间）
   - 要拉取的日志目录
   - 本地下载目录
   - 拉哪几台（PULL_TARGETS）
2. 直接运行（不需要任何命令行参数）：
   python pull_logs.py

依赖：pip install paramiko
"""
import os
import sys
import tarfile
from datetime import datetime

import paramiko


# ============================================================
#                      配 置 区
#        （只改这里就行，下面的代码不用动）
# ============================================================

# ---- 1. 要拉取的日志时间段 ----
LOG_DATE = "2026-05-26"     # 日期 YYYY-MM-DD
LOG_START = "17:00"         # 起始时间 HH:MM 或 HH:MM:SS
LOG_END = "18:00"           # 结束时间 HH:MM 或 HH:MM:SS

# ---- 2. 拉哪几台服务器 ----
PULL_TARGETS = ["hpu", "xcu"]   # 想只拉一台就写 ["hpu"] 或 ["xcu"]

# ---- 3. 两台服务器连接信息 ----
SERVERS = {
    "hpu": {
        "host": "192.168.1.88",      # ← 改成 hpu 的 IP
        "port": 22,
        "user": "galbot",          # hpu 用户名
        "password": "gb@2023",      # ← 改成 hpu 的密码
    },
    "xcu": {
        "host": "192.168.1.66",      # ← 改成 xcu 的 IP
        "port": 22,
        "user": "root",            # xcu 用户名
        "password": "12345678",        # ← 改成 xcu 的密码
    },
}

# ---- 4. 要拉取的日志目录（两台服务器相同）----
LOG_DIRS = [
    "/userdata/log/embosa",
    "/userdata/log/monitor_logs",
]

# ---- 5. 本地下载根目录 ----
LOCAL_OUT_DIR = "robot_logs"

# ---- 6. 是否下载后自动解压 ----
AUTO_EXTRACT = False

# ============================================================
#                   以下为脚本逻辑（一般不用动）
# ============================================================


# 远程筛选 + 打包的 bash 脚本（占位符用 __XXX__ 替换）
REMOTE_BASH = r"""
set -u
WIN_START="__WIN_START__"
WIN_END="__WIN_END__"
DIRS=(__DIRS__)
REMOTE_TAR="__REMOTE_TAR__"
START_EPOCH=$(date -d "$WIN_START" +%s)
END_EPOCH=$(date -d "$WIN_END" +%s)
TMPLIST=$(mktemp)
COUNT=0
for DIR in "${DIRS[@]}"; do
  [ -d "$DIR" ] || { echo "SKIP(no dir): $DIR"; continue; }
  shopt -s nullglob
  for f in "$DIR"/*; do
    [ -f "$f" ] || continue
    base=$(basename "$f")
    fstart=""
    if [[ "$base" =~ ([0-9]{4})-([0-9]{2})-([0-9]{2})-([0-9]{2})-([0-9]{2})-([0-9]{2}) ]]; then
      fstart="${BASH_REMATCH[1]}-${BASH_REMATCH[2]}-${BASH_REMATCH[3]} ${BASH_REMATCH[4]}:${BASH_REMATCH[5]}:${BASH_REMATCH[6]}"
    elif [[ "$base" =~ ([0-9]{4})([0-9]{2})([0-9]{2})([0-9]{2})([0-9]{2})([0-9]{2}) ]]; then
      fstart="${BASH_REMATCH[1]}-${BASH_REMATCH[2]}-${BASH_REMATCH[3]} ${BASH_REMATCH[4]}:${BASH_REMATCH[5]}:${BASH_REMATCH[6]}"
    fi
    mtime_epoch=$(stat -c %Y "$f" 2>/dev/null || echo 0)
    keep=0
    if [ -n "$fstart" ]; then
      fstart_epoch=$(date -d "$fstart" +%s 2>/dev/null || echo 0)
      if [ "$fstart_epoch" -le "$END_EPOCH" ] && [ "$mtime_epoch" -ge "$START_EPOCH" ]; then
        keep=1
      fi
    else
      if [ "$mtime_epoch" -ge "$START_EPOCH" ] && [ "$mtime_epoch" -le "$END_EPOCH" ]; then
        keep=1
      fi
    fi
    if [ "$keep" -eq 1 ]; then
      echo "$f" >> "$TMPLIST"
      echo "MATCH: $f"
      COUNT=$((COUNT+1))
    fi
  done
done
echo "TOTAL_MATCHED=$COUNT"
if [ "$COUNT" -eq 0 ]; then
  rm -f "$TMPLIST"
  exit 3
fi
tar czf "$REMOTE_TAR" -T "$TMPLIST"
rm -f "$TMPLIST"
echo "REMOTE_TAR_DONE=$REMOTE_TAR"
"""


def build_remote_script(win_start, win_end, dirs, remote_tar):
    dirs_str = " ".join(f'"{d}"' for d in dirs)
    s = (
        REMOTE_BASH.replace("__WIN_START__", win_start)
        .replace("__WIN_END__", win_end)
        .replace("__DIRS__", dirs_str)
        .replace("__REMOTE_TAR__", remote_tar)
    )
    return s.replace("\r\n", "\n").replace("\r", "\n")


def pull_one(label, cfg, win_start, win_end, out_dir):
    """连接一台服务器，远程打包 -> 下载 -> 解压。返回是否成功。"""
    print(f"\n{'='*60}")
    print(f"[{label}] 连接 {cfg['user']}@{cfg['host']}:{cfg['port']}")
    print(f"{'='*60}")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname=cfg["host"],
            port=cfg["port"],
            username=cfg["user"],
            password=cfg["password"],
            timeout=15,
            allow_agent=False,
            look_for_keys=False,
        )
    except Exception as e:
        print(f"[{label}] SSH 连接失败: {e}")
        return False

    remote_tar = f"/tmp/robot_logs_{label}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.tar.gz"
    script = build_remote_script(win_start, win_end, LOG_DIRS, remote_tar)

    print(f"[{label}] 远程筛选并打包...")
    try:
        stdin, stdout, stderr = client.exec_command("bash -s")
        stdin.write(script)
        stdin.channel.shutdown_write()
        out_txt = stdout.read().decode("utf-8", errors="replace")
        err_txt = stderr.read().decode("utf-8", errors="replace")
        rc = stdout.channel.recv_exit_status()
    except Exception as e:
        print(f"[{label}] 远程执行异常: {e}")
        client.close()
        return False

    matched = 0
    for line in out_txt.splitlines():
        if line.startswith("MATCH:"):
            print(f"  {line}")
        elif line.startswith("TOTAL_MATCHED="):
            matched = int(line.split("=")[1])
        elif line.startswith("SKIP"):
            print(f"  {line}")

    if rc == 3:
        print(f"[{label}] 该时间段没有匹配的日志文件")
        client.close()
        return False
    if rc != 0:
        print(f"[{label}] 远程打包失败 (rc={rc})")
        if err_txt.strip():
            print(f"[{label}] stderr: {err_txt.strip()}")
        client.close()
        return False

    print(f"[{label}] 匹配 {matched} 个文件，开始下载...")

    local_tar = os.path.join(out_dir, f"{label}.tar.gz")
    try:
        sftp = client.open_sftp()

        # 下载进度回调
        def _progress(transferred, total):
            if total > 0:
                pct = transferred * 100 / total
                bar_len = 30
                filled = int(bar_len * transferred / total)
                bar = "#" * filled + "-" * (bar_len - filled)
                sys.stdout.write(
                    f"\r  [{label}] 下载 [{bar}] {pct:5.1f}% "
                    f"({transferred/1024:.0f}/{total/1024:.0f} KB)"
                )
                sys.stdout.flush()

        sftp.get(remote_tar, local_tar, callback=_progress)
        sys.stdout.write("\n")  # 进度条结束后换行
        sftp.remove(remote_tar)
        sftp.close()
    except Exception as e:
        print(f"\n[{label}] 下载失败: {e}")
        client.close()
        return False

    client.close()
    size_kb = os.path.getsize(local_tar) / 1024
    print(f"[{label}] 已下载: {local_tar} ({size_kb:.1f} KB)")

    if AUTO_EXTRACT:
        dest = os.path.join(out_dir, label)
        os.makedirs(dest, exist_ok=True)
        with tarfile.open(local_tar, "r:gz") as tar:
            tar.extractall(dest)
        print(f"[{label}] 已解压到: {dest}")

    return True


def main():
    def norm_time(t):
        return f"{t}:00" if len(t.split(":")) == 2 else t

    win_start = f"{LOG_DATE} {norm_time(LOG_START)}"
    win_end = f"{LOG_DATE} {norm_time(LOG_END)}"
    try:
        ts = datetime.strptime(win_start, "%Y-%m-%d %H:%M:%S")
        te = datetime.strptime(win_end, "%Y-%m-%d %H:%M:%S")
    except ValueError as e:
        print(f"时间格式错误: {e}")
        sys.exit(1)
    if te <= ts:
        print("错误：结束时间必须晚于起始时间")
        sys.exit(1)

    tag = f"{LOG_DATE}_{LOG_START.replace(':','')}-{LOG_END.replace(':','')}"
    out_dir = os.path.join(LOCAL_OUT_DIR, tag)
    os.makedirs(out_dir, exist_ok=True)

    print(f"时间窗: {win_start}  ~  {win_end}")
    print(f"目录:   {', '.join(LOG_DIRS)}")
    print(f"拉取:   {', '.join(PULL_TARGETS)}")
    print(f"输出到: {os.path.abspath(out_dir)}")

    results = []
    for label in PULL_TARGETS:
        if label not in SERVERS:
            print(f"[{label}] 配置区 SERVERS 中没有该服务器，跳过")
            results.append((label, False))
            continue
        ok = pull_one(label, SERVERS[label], win_start, win_end, out_dir)
        results.append((label, ok))

    print(f"\n{'='*60}\n汇总\n{'='*60}")
    for label, ok in results:
        print(f"  {label}: {'成功' if ok else '无日志/失败'}")
    print(f"\n全部输出在: {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main()
