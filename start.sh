#!/bin/bash

# ===== 修改这里的 IP =====
HOST_TABLE="192.168.95.28"
HOST_FLOOR="192.168.95.31"
HOST_BAG="192.168.95.29"
# ========================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

cleanup() {
    echo ""
    echo "[start] 正在停止所有服务..."
    kill "$PID_TABLE" "$PID_FLOOR" "$PID_BAG" 2>/dev/null
    wait "$PID_TABLE" "$PID_FLOOR" "$PID_BAG" 2>/dev/null
    echo "[start] 已退出"
}
trap cleanup EXIT INT TERM

cd "$SCRIPT_DIR/client_server" || exit 1

echo "[start] 启动 clear_table     -> logs/clear_table.log"
python robot_server_clear_table.py     --model-host "$HOST_TABLE" > "$LOG_DIR/clear_table.log"     2>&1 &
PID_TABLE=$!

echo "[start] 启动 clean_floor     -> logs/clean_floor.log"
python robot_server_clean_floor.py     --model-host "$HOST_FLOOR" > "$LOG_DIR/clean_floor.log"     2>&1 &
PID_FLOOR=$!

echo "[start] 启动 put_garbage_bag -> logs/put_garbage_bag.log"
python robot_server_put_garbage_bag.py --model-host "$HOST_BAG"   > "$LOG_DIR/put_garbage_bag.log" 2>&1 &
PID_BAG=$!

echo "[start] 等待服务就绪（日志在 logs/ 目录）..."
sleep 3

echo "[start] 启动 api_menu"
python api_menu.py
