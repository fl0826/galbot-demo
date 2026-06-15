"""
================================================================================
项目名称: Galbot VLA 交互控制后端
文件功能: 
    本脚本基于 FastAPI 框架开发，作为机器人算法（VLA）与前端 Web 界面之间的通信中枢。
    它负责管理算法的配置参数、控制进程的生命周期，并将运行日志实时推送到前端。

核心模块说明:

    1. 参数持久化管理 (Args Persistence):
       - 动态读取并重新加载 `args.py` 配置文件。
       - 使用正则表达式直接操作 Python 源码文件，实现 Web 端参数的实时修改与保存。

    2. 算法进程控制 (Process Management):
       - 异步启动 `galbotvla_wholebody.py` 算法子进程。
       - 采用进程组（Process Group）管理机制，确保在停止任务时能够完整销毁所有关联子进程，避免僵尸进程。

    3. 实时日志广播 (WebSocket Log Streaming):
       - 实时捕获子进程的 stdout/stderr 输出。
       - 通过 WebSocket 协议将带时间戳的日志即时广播给所有连接的前端客户端。
    4. 测试/正式环境:
       - 测试环境ip:192.168.121.202(须公司内网且指定机器人的ip)，端口:2334。
       - 正式环境ip:192.168.1.88(有线)，端口:2333。

    5. 日志系统:
       - 目录: galbot_vla_real/logs/
       - server 日志: {日期}_server.log —— 记录所有 server 自身的运行日志
       - VLA 子进程日志: {日期}_galbotvla_wholebody.log —— 记录所有子进程 stdout/stderr 输出
       - 日期使用北京时间(UTC+8)，精确到天，格式: 2026-03-17
       - 追加模式(mode='a')，日志只增不覆盖
       - 日志生命周期7天，启动时自动清理过期日志

使用说明:
    - 运行环境: 建议在 Linux 环境下运行（涉及 os.setsid 与 os.killpg）。
    - 启动命令: python server.py (默认运行在 0.0.0.0:8000)
    - 依赖文件: `args.py` 和 `galbotvla_wholebody.py` 位于项目根目录 galbot_vla_real/ 下。
================================================================================
"""

import os
import sys
import asyncio
import signal
import json
import logging
import time
from datetime import datetime, timezone, timedelta
from typing import List, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import re

from server_eval_api import eval_router

# --- 双日志系统初始化 ---

# 北京时间 UTC+8
BEIJING_TZ = timezone(timedelta(hours=8))

def get_beijing_date_str():
    """获取北京时间日期字符串，精确到天，格式: 2026-03-17"""
    return datetime.now(BEIJING_TZ).strftime("%Y-%m-%d")

def get_beijing_datetime_str():
    """获取北京时间完整时间戳，格式: 2026-03-17 14:30:05"""
    return datetime.now(BEIJING_TZ).strftime("%Y-%m-%d %H:%M:%S")

def get_beijing_time_str():
    """获取北京时间时分秒，格式: 14:30:05"""
    return datetime.now(BEIJING_TZ).strftime("%H:%M:%S")

# 日志目录: server_v1.py 在 galbot_vla_real/server/ 下，logs 在 galbot_vla_real/logs/ 下
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)  # galbot_vla_real/

# --- MAR171442 Start: 统一使用绝对路径，避免 cwd 导致找不到文件 ---
ARGS_FILE = os.path.join(PROJECT_ROOT, "args.py")
VLA_SCRIPT = os.path.join(PROJECT_ROOT, "galbotvla_wholebody.py")
INDEX_HTML = os.path.join(SCRIPT_DIR, "index.html")  # index.html 跟 server.py 同级
sys.path.insert(0, PROJECT_ROOT)  # 确保 import args 能找到 galbot_vla_real/args.py
# --- MAR171442 End ---

LOG_DIR = os.path.join(PROJECT_ROOT, "logs")
os.makedirs(LOG_DIR, exist_ok=True)

# --- 日志生命周期管理，保留7天，启动时自动清理过期日志 ---
LOG_RETENTION_DAYS = 7

def cleanup_expired_logs():
    """启动时扫描 logs 目录，删除超过 LOG_RETENTION_DAYS 天的日志文件。
    只删除 YYYY-MM-DD_server.log 和 YYYY-MM-DD_galbotvla_wholebody.log，不动其他文件。"""
    MANAGED_SUFFIXES = ("_server.log", "_galbotvla_wholebody.log")
    today = datetime.now(BEIJING_TZ).date()
    deleted = []
    kept = []
    for filename in os.listdir(LOG_DIR):
        # 只匹配 YYYY-MM-DD_server.log 或 YYYY-MM-DD_galbotvla_wholebody.log
        if not filename.endswith(MANAGED_SUFFIXES):
            continue
        date_part = filename[:10]
        try:
            file_date = datetime.strptime(date_part, "%Y-%m-%d").date()
        except ValueError:
            continue
        age_days = (today - file_date).days
        if age_days > LOG_RETENTION_DAYS:
            filepath = os.path.join(LOG_DIR, filename)
            try:
                os.remove(filepath)
                deleted.append(f"{filename}({age_days}天)")
            except OSError as e:
                print(f"[WARN] 删除过期日志失败: {filepath} -> {e}")
        else:
            kept.append(filename)
    return deleted, kept

_deleted_logs, _kept_logs = cleanup_expired_logs()

DATE_STR = get_beijing_date_str()
SERVER_LOG_FILE = os.path.join(LOG_DIR, f"{DATE_STR}_server.log")
VLA_LOG_FILE = os.path.join(LOG_DIR, f"{DATE_STR}_galbotvla_wholebody.log")

# --- Logger 1: server_logger —— 记录 server 自身所有日志 ---
server_logger = logging.getLogger("server")
server_logger.setLevel(logging.DEBUG)
server_logger.propagate = False  # 防止重复输出

_server_file_handler = logging.FileHandler(SERVER_LOG_FILE, mode='a', encoding='utf-8')
_server_file_handler.setLevel(logging.DEBUG)
_server_file_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))

_server_console_handler = logging.StreamHandler(sys.stdout)
_server_console_handler.setLevel(logging.INFO)
_server_console_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))

server_logger.addHandler(_server_file_handler)
server_logger.addHandler(_server_console_handler)

# --- Logger 2: vla_logger —— 记录 galbotvla_wholebody.py 子进程所有输出 ---
vla_logger = logging.getLogger("vla_subprocess")
vla_logger.setLevel(logging.DEBUG)
vla_logger.propagate = False

_vla_file_handler = logging.FileHandler(VLA_LOG_FILE, mode='a', encoding='utf-8')
_vla_file_handler.setLevel(logging.DEBUG)
_vla_file_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))

vla_logger.addHandler(_vla_file_handler)

server_logger.info("=" * 60)
server_logger.info("Server 启动，日志系统初始化完成")
server_logger.info(f"项目根目录: {PROJECT_ROOT}")
server_logger.info(f"args.py 路径: {ARGS_FILE}")  # MAR171442
server_logger.info(f"VLA 脚本路径: {VLA_SCRIPT}")  # MAR171442
server_logger.info(f"index.html 路径: {INDEX_HTML}")  # MAR171442
server_logger.info(f"日志目录: {LOG_DIR}")
server_logger.info(f"日志保留天数: {LOG_RETENTION_DAYS} 天")  
server_logger.info(f"本次清理过期日志: {_deleted_logs if _deleted_logs else '无'}")  
server_logger.info(f"当前保留日志: {_kept_logs}")  
server_logger.info(f"Server 日志文件: {SERVER_LOG_FILE}")
server_logger.info(f"VLA 子进程日志文件: {VLA_LOG_FILE}")
server_logger.info("=" * 60)



app = FastAPI()

# --- HTTP 请求日志中间件 ---
@app.middleware("http")
async def log_requests(request: Request, call_next):
    start_time = time.time()
    client_ip = request.client.host if request.client else "Unknown"
    server_logger.info(f"==> HTTP {request.method} {request.url} | IP: {client_ip}")
    try:
        response = await call_next(request)
        elapsed_ms = (time.time() - start_time) * 1000
        server_logger.info(f"<== HTTP {request.method} {request.url} | Status: {response.status_code} | {elapsed_ms:.1f}ms")
        return response
    except Exception as e:
        server_logger.error(f"<== HTTP ERROR {request.method} {request.url} | Exception: {e}")
        raise

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- 模型评测：仅评测参数读写，启动/停止仍用 /api/control ---
app.include_router(eval_router)

# --- 全局变量：管理算法子进程 ---
vla_process: Optional[asyncio.subprocess.Process] = None

class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        server_logger.info(f"WebSocket 客户端已连接，当前连接数: {len(self.active_connections)}")  
    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        server_logger.info(f"WebSocket 客户端已断开，当前连接数: {len(self.active_connections)}")  
    async def broadcast(self, message: str):
        server_logger.debug(f"WS 广播: {message}")  # 广播内容记入 server 日志(DEBUG级别)
        for connection in self.active_connections:
            try: await connection.send_text(message)
            except Exception as e:
                server_logger.warning(f"WS 发送失败: {e}")  

manager = ConnectionManager()

# --- 功能 1：参数交互与 Args 文件持久化 ---

def make_res(code=200, data=None, msg =""):
    return {"code": code, "data": data, "msg": msg}

def update_args_file(new_params: dict):
    """
    后端 Python 函数：安全更新 args.py。
    1. 改用 match.group 拼接，解决 invalid group reference 报错。
    2. 针对 host, prompt_type, object_name 强制使用双引号，其余使用单引号。
    3. 支持 JS 传来的复杂类型（List, Bool, Float 等）。
    """
    server_logger.info(f"update_args_file 被调用，参数: {new_params}")  

    # 针对 target_image_size_head 等 List[int] 字段做强制校验与转换
    for key, val in list(new_params.items()):
        if isinstance(val, str) and "," in val:
            # 去掉可能存在的括号，尝试按逗号分割
            clean_val = val.strip("[]() ")
            try:
                parts = [p.strip() for p in clean_val.split(",") if p.strip()]
                # 检查是否所有部分都是数字（允许负数和浮点字符串转int）
                if all(re.match(r'^-?\d+(\.\d+)?$', p) for p in parts):
                    new_params[key] = [int(float(p)) for p in parts]
                    server_logger.info(f"自动转换参数类型 {key}: '{val}' -> {new_params[key]}")
            except Exception as e:
                server_logger.warning(f"尝试转换参数 {key} 失败: {e}")

    
    try:
        if not os.path.exists(ARGS_FILE):  # MAR171442: 使用绝对路径
            server_logger.error(f"未找到 args.py 文件: {ARGS_FILE}") 
            return False, "未找到 args.py 文件", {}

        with open(ARGS_FILE, "r", encoding="utf-8") as f:  # MAR171442
            lines = f.readlines()

        updated_keys = set()
        new_lines = []

        # 需要强制双引号的 Key 列表
        double_quote_keys = ["host", "prompt_type", "object_name"]

        for line in lines:
            matched_any_key = False
            for key, val in new_params.items():
                # 匹配模式：捕获组 1 包含缩进、变量名、可选类型标注和等号
                pattern = rf"^(\s+{key}(?::\s*[\w\[\], ]+)?\s*=\s*).*$"

                match = re.match(pattern, line)
                if match:
                    prefix = match.group(1)

                    # 针对特定 Key 处理引号 ---
                    if key in double_quote_keys:
                        if isinstance(val, str):
                            # 纯字符串：直接包装双引号
                            formatted_val = f'"{val}"'
                        elif isinstance(val, list):
                            # 列表：将其中的字符串元素转为双引号，非字符串按原样处理
                            inner_items = [f'"{item}"' if isinstance(item, str) else repr(item) for item in val]
                            formatted_val = f"[{', '.join(inner_items)}]"
                        else:
                            formatted_val = repr(val)
                    else:
                        # 其余字段：使用 Python 默认 repr (字符串通常使用单引号)
                        formatted_val = repr(val)

                    if val is None:
                        formatted_val = "None"

                    # 直接拼接前缀和格式化后的值，避免正则反向引用报错
                    new_line = f"{prefix}{formatted_val}\n"
                    new_lines.append(new_line)
                    updated_keys.add(key)
                    matched_any_key = True
                    break

            if not matched_any_key:
                new_lines.append(line)

        # 检查是否有未成功匹配的 Key
        missing_keys = set(new_params.keys()) - updated_keys
        if missing_keys:
            server_logger.warning(f"部分参数在 args.py 中未定义: {list(missing_keys)}")  
            return False, f"参数 {list(missing_keys)} 在 args.py 中未定义", {"missing": list(missing_keys)}

        with open(ARGS_FILE, "w", encoding="utf-8") as f:  # MAR171442
            f.writelines(new_lines)

        server_logger.info(f"配置更新成功，已更新 keys: {list(updated_keys)}")  
        return True, "配置更新成功", {"updated": list(updated_keys)}

    except Exception as e:
        server_logger.error(f"update_args_file 异常: {e}", exc_info=True)  # 记录完整堆栈
        return False, f"系统错误: {str(e)}", {}

@app.get("/api/params")
async def get_params():
    """动态加载 args.py 确保前端看到的是当前磁盘上的默认值"""
    server_logger.info("API GET /api/params")  
    import args
    import importlib
    importlib.reload(args)
    global_args = args.Args()

    # 按照字段提取
    TARGET_FIELDS = [
        "host",
        "port",
        "object_name",
        "object_location",
        "blocking",
        "action_horizon_use",
        "dt_model_control",
        "prompt_type",
        "object_location_on_shelf",
        "hand_used",
        "action_horizon",
        "shelf_location",
        "gripper_vel",
        "gripper_effort",
        "gripper_init_width",
        "target_image_size_head",
        "target_image_size_left_arm",
        "target_image_size_right_arm"
    ]
    data = {field: getattr(global_args, field, None) for field in TARGET_FIELDS}
    server_logger.info(f"API GET /api/params 返回: {data}") 
    return make_res(data = data, msg = "参数加载成功！")

@app.post("/api/params")
async def set_params(request: Request):
    data = await request.json()
    server_logger.info(f"API POST /api/params 收到: {data}") 
    if update_args_file(data):
        return make_res(msg="高频参数已更新至args.py。")
    return make_res(code=500, msg="配置更新至args.py失败！")

# --- 功能 2：异步调用外部脚本 ---

async def log_reader(pipe, prefix):
    """使用异步方式读取 pipe，修复之前的 bytes 报错"""
    server_logger.info(f"log_reader 启动，prefix={prefix}")  
    while True:
        line = await pipe.readline()
        if not line:
            server_logger.info(f"log_reader [{prefix}] pipe 已关闭，退出读取循环")  
            break
        msg = line.decode('utf-8').strip()
        timestamp = get_beijing_time_str() 
        broadcast_msg = f"[{prefix}]-{timestamp}-{msg}"
        vla_logger.info(f"[{prefix}] {msg}")  # 子进程输出写入 VLA 日志文件
        await manager.broadcast(broadcast_msg)

@app.post("/api/control")
async def control_robot(request: Request):
    global vla_process
    data = await request.json()
    action = data.get("action")

    server_logger.info(f"API POST /api/control 收到 action='{action}'")  

    if action == 'start':
        if vla_process and vla_process.returncode is None:
            server_logger.warning(f"启动被拒绝: 任务已在运行中，PID={vla_process.pid}") 
            return make_res(code=400, msg="任务已经启动，请先去停止")

        server_logger.info("正在启动 galbotvla_wholebody.py 子进程...")  
        # 设置 PYTHONUNBUFFERED=1 禁用子进程 stdout 块缓冲，实现日志实时输出
        unbuffered_env = os.environ.copy()
        unbuffered_env["PYTHONUNBUFFERED"] = "1"
        # 使用 asyncio 版本的子进程创建方式
        vla_process = await asyncio.create_subprocess_exec(
            sys.executable, VLA_SCRIPT,  # MAR171442: 使用绝对路径
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            preexec_fn=os.setsid, # 仅限 Linux，用于结束进程组
            env=unbuffered_env,  # 入无缓冲环境变量
            cwd=PROJECT_ROOT  # MAR171442: 工作目录设为项目根目录，确保子进程内的相对路径正确
        )

        asyncio.create_task(log_reader(vla_process.stdout, "VLA"))
        server_logger.info(f"子进程启动成功，PID={vla_process.pid}")  
        vla_logger.info(f"========== 子进程启动 PID={vla_process.pid} ==========")  # VLA日志也记录启动事件
        return make_res(code=200, msg="任务启动成功...")

    elif action == 'stop':
        if not vla_process or vla_process.returncode is not None:
            server_logger.warning("停止被拒绝: 模型未运行或已退出")  
            return make_res(code=400, msg="模型未运行")

        try:
            pgid = os.getpgid(vla_process.pid)
            timestamp = get_beijing_time_str()  
            server_logger.info(f"开始终止进程，PID={vla_process.pid}, PGID={pgid}") 
            vla_logger.info(f"========== 收到停止指令 PID={vla_process.pid} ==========")  
            await manager.broadcast(f"[SYSTEM]-{timestamp}-开始终止模型进程(PID:{vla_process.pid})...")

            i = 0
            while i <= 10:
                # 先检查进程是否已退出，已死就不用继续杀了
                if vla_process.returncode is not None:
                    timestamp = get_beijing_time_str()  
                    server_logger.info(f"进程已在第{i}次 SIGKILL 前退出，returncode={vla_process.returncode}") 
                    vla_logger.info(f"进程已在第{i}次 SIGKILL 前退出，returncode={vla_process.returncode}")  
                    await manager.broadcast(f"[SYSTEM]-{timestamp}-进程已在第{i}次终止信号前退出")  
                    break

                os.killpg(pgid, signal.SIGKILL)
                await asyncio.sleep(0.05)
                timestamp = get_beijing_time_str()  
                server_logger.info(f"SIGKILL 第 {i+1}/11 次 | PID={vla_process.pid} | returncode={vla_process.returncode}")
                await manager.broadcast(f"[SYSTEM]-{timestamp}-发送 SIGKILL 第{i+1}/11次")
                i += 1

            # 回收进程
            try:
                await asyncio.wait_for(vla_process.wait(), timeout=2.0)
                server_logger.info(f"进程已回收，returncode={vla_process.returncode}")  
            except asyncio.TimeoutError:
                timestamp = get_beijing_time_str() 
                server_logger.warning("进程回收超时(2s)，发送最终 SIGKILL")  
                await manager.broadcast(f"[SYSTEM]-{timestamp}-进程回收超时，最终强杀...")  
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

            vla_process = None
            timestamp = get_beijing_time_str()  
            server_logger.info("模型进程已终止，vla_process 已置空")  
            vla_logger.info("========== 子进程已终止 ==========")  
            await manager.broadcast(f"[SYSTEM]-{timestamp}-模型已强制停止")
            return make_res(msg="已立即终止模型进程")

        except ProcessLookupError:
            vla_process = None
            timestamp = get_beijing_time_str()  
            server_logger.warning(f"ProcessLookupError: 进程已不存在，已清理引用")  
            vla_logger.info("========== 进程已不存在，清理引用 ==========")  
            await manager.broadcast(f"[SYSTEM]-{timestamp}-进程已不存在，已清理引用")  
            return make_res(msg="进程已不存在，已清理")
        except Exception as e:
            vla_process = None
            timestamp = get_beijing_time_str()  
            server_logger.error(f"终止进程异常: {e}", exc_info=True)  # 记录完整堆栈
            vla_logger.error(f"终止异常: {e}")  
            await manager.broadcast(f"[SYSTEM]-{timestamp}-终止异常: {str(e)}")
            return make_res(code=500, msg=f"终止异常: {str(e)}")

    server_logger.warning(f"无效的 control action: '{action}'") 
    return make_res(code=400, msg="无效的控制操作")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            client_msg = await websocket.receive_text()
            server_logger.debug(f"收到 WS 客户端消息: {client_msg}")  
    except WebSocketDisconnect: manager.disconnect(websocket)

@app.get("/", response_class=HTMLResponse)
async def read_index():
    server_logger.debug("GET / 加载 index.html")  
    with open(INDEX_HTML, "r", encoding="utf-8") as f:  # MAR171442: 使用绝对路径
        return f.read()

if __name__ == "__main__":
    server_logger.info(f"uvicorn 启动中，监听 0.0.0.0:2333")  
    uvicorn.run(app, host="0.0.0.0", port=2334)
