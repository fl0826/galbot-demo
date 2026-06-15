from __future__ import annotations

"""
================================================================================
模块名称: 模型评测后端路由 (Evaluation APIs)
文件功能:
    本模块只负责「模型评测」相关的 FastAPI 路由定义，作为现有 server.py 的补充。
    - 不直接启动 Uvicorn，不监听端口。
    - 需要在 server.py 中通过 app.include_router(eval_router) 手动挂载。

meta dump：默认通过 ssh/scp 推到远端 ~/mock_dump，无需 nfs/sshfs 挂载。
================================================================================
"""

import asyncio
import errno
import json
import logging
import os
import re
import subprocess
import sys
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Request
from pydantic import BaseModel, Field

# 与 server.py 中同名，输出到 logs/{日期}_server.log
server_logger = logging.getLogger("server")

BASE_DIR = Path(__file__).resolve().parent.parent  # 项目根目录 (galbot_vla_real/)

# 与现有 server.py 保持一致的路径约定
ARGS_FILE = BASE_DIR / "args.py"

# 与 server.py 一致：北京时间
BEIJING_TZ = timezone(timedelta(hours=8))

# 本地落盘根（仅当 EVAL_META_USE_REMOTE=0 时使用）
META_DUMP_LOCAL_DEFAULT = Path(os.environ.get("TMPDIR", "/tmp")) / "eval_meta_dump"

_SSH_OPTS = [
    "-o",
    "BatchMode=yes",
    "-o",
    "StrictHostKeyChecking=accept-new",
]


def _use_remote_dump() -> bool:
    """默认开启远端 scp；本地调试设 EVAL_META_USE_REMOTE=0。"""
    v = os.environ.get("EVAL_META_USE_REMOTE", "1").strip().lower()
    return v not in ("0", "false", "no", "off")


def _dump_oserror_hint(err: OSError) -> str:
    """Permission denied 等常见错误的可操作建议。"""
    if err.errno in (errno.EACCES, errno.EPERM):
        return "目录无写权限：请检查 EVAL_META_DUMP_BASE 路径属主，或改用远端模式（默认 EVAL_META_USE_REMOTE=1）。"
    return ""


def _meta_dump_base() -> Path:
    """本地模式下的落盘根目录：EVAL_META_DUMP_BASE，未设置则用 META_DUMP_LOCAL_DEFAULT。"""
    raw = os.environ.get("EVAL_META_DUMP_BASE")
    if raw:
        return Path(os.path.expandvars(os.path.expanduser(raw)))
    return META_DUMP_LOCAL_DEFAULT


def _remote_ssh_base(host: str, port: int, user: str) -> list[str]:
    return ["ssh", "-p", str(port), *_SSH_OPTS, f"{user}@{host}"]


def _remote_scp_to(
    host: str,
    port: int,
    user: str,
    local_file: Path,
    remote_relpath: str,
) -> None:
    """remote_relpath: 相对远端用户家目录，如 mock_dump/1_2/a_meta.json"""
    dest = f"{user}@{host}:~/{remote_relpath.lstrip('/')}"
    subprocess.run(
        ["scp", "-P", str(port), *_SSH_OPTS, str(local_file), dest],
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
    )


def _dump_via_ssh_scp(
    task_id: int,
    job_id: int,
    ts: str,
    payload: dict[str, Any],
) -> dict[str, Any]:
    """
    在远端 192.168.1.99（可配置）家目录下 EVAL_META_REMOTE_DIR 内写入
    {task_id}_{job_id}/{ts}_meta.json。需本机已对远端配置免密 ssh（ssh-copy-id）。
    """
    host = os.environ.get("EVAL_META_REMOTE_HOST", "192.168.1.99")
    user = os.environ.get("EVAL_META_REMOTE_USER", os.environ.get("USER", "galbot"))
    port = int(os.environ.get("EVAL_META_REMOTE_PORT", "22"))
    remote_dir = os.environ.get("EVAL_META_REMOTE_DIR", "mock_dump").strip().strip("/")
    folder_name = f"{task_id}_{job_id}"
    filename = f"{ts}_meta.json"
    rel_folder = f"{remote_dir}/{folder_name}"
    rel_file = f"{rel_folder}/{filename}"

    text = json.dumps(payload, ensure_ascii=False, indent=2)

    # 远端建目录
    r = subprocess.run(
        _remote_ssh_base(host, port, user) + [f"mkdir -p ~/{rel_folder}"],
        capture_output=True,
        text=True,
        timeout=60,
    )
    if r.returncode != 0:
        err = (r.stderr or r.stdout or "").strip()
        raise OSError(f"远端 mkdir 失败: {err or r.returncode}")

    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        delete=False,
        suffix=".json",
    ) as tf:
        tf.write(text)
        local_path = Path(tf.name)

    try:
        _remote_scp_to(host, port, user, local_path, rel_file)
    except subprocess.CalledProcessError as e:
        msg = (e.stderr or e.stdout or "").strip()
        raise RuntimeError(f"scp 失败: {msg or e.returncode}") from e
    finally:
        try:
            local_path.unlink()
        except OSError:
            pass

    remote_display = f"{user}@{host}:~/{rel_file}"
    return {
        "path": remote_display,
        "folder": f"~/{rel_folder}",
        "filename": filename,
        "host": host,
        "transport": "scp",
    }

eval_router = APIRouter()


def _make_res(code: int = 200, data=None, msg: str = ""):
    return {"code": code, "data": data, "msg": msg}


def _args_class_to_dict() -> dict[str, Any]:
    """从当前磁盘上的 args.py 加载 Args，将其可序列化字段打成字典。"""
    if str(BASE_DIR) not in sys.path:
        sys.path.insert(0, str(BASE_DIR))

    import importlib
    import args  # type: ignore

    importlib.reload(args)
    inst = args.Args()  # type: ignore
    out: dict[str, Any] = {}
    for name in dir(inst):
        if name.startswith("_"):
            continue
        val = getattr(inst, name)
        if callable(val):
            continue
        out[name] = val
    return out


class MetaDumpBody(BaseModel):
    task_id: int = Field(..., description="任务 ID")
    job_id: int = Field(..., description="实例 ID")


# ============================
# 功能 0：评测参数读写 (args.py)
# ============================

# 评测相关允许修改的字段
EVAL_PARAM_FIELDS = [
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
    "target_image_size_right_arm",
]


def _update_args_file_eval(new_params: dict):
    """
    评测专用的 args.py 更新逻辑：
    - 仅允许修改 EVAL_PARAM_FIELDS 中的字段
    - 引号/列表格式与旧版 server.py 保持一致：
        * host, prompt_type, object_name 强制使用双引号
        * 其余字段使用 Python 默认 repr

    备注（类型冲突说明）:
        - args.py 中 `host` 声明为 `host: str = ["10.119.100.16"]`，
          注解是 str，实际值是 List[str]。
          本接口**沿用旧版行为**，把 `host` 当作 List[str] 来处理，
          与现有 server.py 保持一致。
    """
    if not ARGS_FILE.exists():
        return False, "未找到 args.py 文件", {}

    with ARGS_FILE.open("r", encoding="utf-8") as f:
        lines = f.readlines()

    # 只保留允许的 key
    filtered_params = {k: v for k, v in new_params.items() if k in EVAL_PARAM_FIELDS}

    updated_keys = set()
    new_lines = []

    double_quote_keys = ["host", "prompt_type", "object_name"]

    for line in lines:
        matched_any_key = False
        for key, val in filtered_params.items():
            pattern = rf"^(\s+{key}(?::\s*[\w\[\], ]+)?\s*=\s*).*$"
            m = re.match(pattern, line)
            if not m:
                continue

            prefix = m.group(1)

            if key in double_quote_keys:
                if isinstance(val, str):
                    formatted_val = f'"{val}"'
                elif isinstance(val, list):
                    inner_items = [
                        f'"{item}"' if isinstance(item, str) else repr(item)
                        for item in val
                    ]
                    formatted_val = f"[{', '.join(inner_items)}]"
                else:
                    formatted_val = repr(val)
            else:
                formatted_val = repr(val)

            if val is None:
                formatted_val = "None"

            new_lines.append(f"{prefix}{formatted_val}\n")
            updated_keys.add(key)
            matched_any_key = True
            break

        if not matched_any_key:
            new_lines.append(line)

    missing_keys = set(filtered_params.keys()) - updated_keys
    # if missing_keys:
    #     return False, f"参数 {list(missing_keys)} 在 args.py 中未定义", {"missing": list(missing_keys)}

    with ARGS_FILE.open("w", encoding="utf-8") as f:
        f.writelines(new_lines)

    return True, "配置更新成功", {"updated": list(updated_keys)}


@eval_router.get("/api/eval/params")
async def get_eval_params():
    """
    读取评测相关的参数子集，来源为当前磁盘上的 args.py。
    字段列表见 EVAL_PARAM_FIELDS。
    """
    if str(BASE_DIR) not in sys.path:
        sys.path.insert(0, str(BASE_DIR))

    import importlib
    import args  # type: ignore

    importlib.reload(args)
    global_args = args.Args()  # type: ignore

    data = {field: getattr(global_args, field, None) for field in EVAL_PARAM_FIELDS}
    return _make_res(data=data, msg="评测参数加载成功。")


@eval_router.post("/api/eval/params")
async def set_eval_params(request: Request):
    """
    修改评测相关的高频参数，直接改写 args.py。
    仅会影响 EVAL_PARAM_FIELDS 中列出的字段。
    """
    body = await request.json()
    ok, msg, extra = _update_args_file_eval(body)
    if ok:
        return _make_res(data=extra, msg="评测参数已更新至 args.py。")
    return _make_res(code=500, data=extra, msg=msg)


@eval_router.post("/api/eval/meta/dump")
async def dump_args_meta_json(body: MetaDumpBody):
    """
    将当前 args.py 中 Args 类的全部字段序列化为 JSON 落盘。

    默认（EVAL_META_USE_REMOTE 非 0）：经 ssh/scp 推到远端 ~/mock_dump/{task_id}_{job_id}/{时间}_meta.json，
    需 1.88 已对 1.99 配好免密登录（ssh-copy-id）。

    本地模式（EVAL_META_USE_REMOTE=0）：写到 EVAL_META_DUMP_BASE 或 /tmp/eval_meta_dump/…
    """
    task_id = body.task_id
    job_id = body.job_id
    ts = datetime.now(BEIJING_TZ).strftime("%Y_%m_%d_%H_%M_%S")
    filename = f"{ts}_meta.json"
    payload = _args_class_to_dict()

    if _use_remote_dump():
        try:
            data = await asyncio.to_thread(_dump_via_ssh_scp, task_id, job_id, ts, payload)
        except OSError as e:
            server_logger.error(
                "[eval/meta/dump] 远端 mkdir 失败 task_id=%s job_id=%s err=%s",
                task_id,
                job_id,
                e,
            )
            return _make_res(
                code=500,
                data={"path": str(e)},
                msg=f"远端 mkdir 失败: {e}",
            )
        except RuntimeError as e:
            server_logger.error(
                "[eval/meta/dump] 远端 scp 失败 task_id=%s job_id=%s err=%s",
                task_id,
                job_id,
                e,
            )
            return _make_res(
                code=500,
                data={"hint": "请确认本机已 ssh-copy-id 到远端，且远端磁盘可写"},
                msg=str(e),
            )

        server_logger.info(
            "[eval/meta/dump] 已保存 meta.json(远端) task_id=%s job_id=%s path=%s",
            task_id,
            job_id,
            data.get("path"),
        )
        return _make_res(data=data, msg="meta.json 已保存到远端。")

    folder = _meta_dump_base() / f"{task_id}_{job_id}"
    out_path = folder / filename

    try:
        folder.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except OSError as e:
        hint = _dump_oserror_hint(e)
        server_logger.error(
            "[eval/meta/dump] 写入失败 task_id=%s job_id=%s path=%s err=%s %s",
            task_id,
            job_id,
            out_path,
            e,
            hint or "",
        )
        data_err: dict[str, Any] = {"path": str(out_path), "errno": e.errno}
        if hint:
            data_err["hint"] = hint
        msg = f"写入失败: {e}"
        if hint:
            msg = f"{msg} {hint}"
        return _make_res(code=500, data=data_err, msg=msg)

    server_logger.info(
        "[eval/meta/dump] 已保存 meta.json(本地) task_id=%s job_id=%s path=%s",
        task_id,
        job_id,
        out_path,
    )
    return _make_res(
        data={
            "path": str(out_path),
            "folder": str(folder),
            "filename": filename,
        },
        msg="meta.json 已保存。",
    )
