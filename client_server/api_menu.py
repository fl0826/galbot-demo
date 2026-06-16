"""
交互式接口测试脚本。

用法：
  python api_menu.py
  python api_menu.py --host 192.168.1.10
  python api_menu.py --timeout 3

启动后输入菜单编号，例如 1、2、3，即可发送对应 POST 请求。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class ApiAction:
    key: str
    title: str
    method: str
    port: int
    path: str
    group: str
    note: str = ""
    body: dict = field(default_factory=dict)


ACTIONS = [
    # ===== 套垃圾袋 =====
    ApiAction("1",  "套垃圾袋：正常启动（含复位）",       "POST", 9053, "/api/put_garbage_bag",        "套垃圾袋"),
    ApiAction("2",  "套垃圾袋：断点续推（跳过复位）",     "POST", 9053, "/api/put_garbage_bag_resume",  "套垃圾袋"),
    ApiAction("3",  "套垃圾袋：仅复位",                   "POST", 9053, "/api/reset",                  "套垃圾袋"),
    ApiAction("4",  "套垃圾袋：停止",                     "POST", 9053, "/api/stop",                   "套垃圾袋"),

    # ===== 打扫地面 =====
    ApiAction("5",  "打扫地面：启动推理（不复位）",        "POST", 9051, "/api/clean_floor",             "打扫地面"),
    ApiAction("6",  "打扫地面：停止",                     "POST", 9051, "/api/stop",                   "打扫地面"),
    ApiAction("7",  "打扫地面：仅复位",                   "POST", 9051, "/api/reset",                  "打扫地面"),

    # ===== 清理桌面 - 推理 =====
    ApiAction("8",  "清理桌面：升降取垃圾袋（含复位）",   "POST", 9052, "/api/pick_bag",               "清理桌面-推理"),
    ApiAction("9",  "清理桌面：桌面物品清理",             "POST", 9052, "/api/bag_large_items",        "清理桌面-推理"),
    ApiAction("10", "清理桌面：抹布清理",                 "POST", 9052, "/api/sweep_trash",            "清理桌面-推理"),
    ApiAction("11", "清理桌面：提起袋子",                 "POST", 9052, "/api/lift_bag",               "清理桌面-推理"),
    ApiAction("12", "清理桌面：停止",                     "POST", 9052, "/api/stop",                   "清理桌面-推理"),

    # ===== 清理桌面 - 复位 =====
    ApiAction("13", "清理桌面：复位（桌面默认）",          "POST", 9052, "/api/reset",                  "清理桌面-复位"),

    # ===== 清理桌面 - 夹爪 =====
    ApiAction("14", "清理桌面：松爪",                     "POST", 9052, "/api/open_gripper",           "清理桌面-夹爪"),
    ApiAction("15", "清理桌面：闭合夹爪",                 "POST", 9052, "/api/close_gripper",          "清理桌面-夹爪"),
]

ACTION_BY_KEY = {action.key: action for action in ACTIONS}


def build_url(host: str, action: ApiAction) -> str:
    return f"http://{host}:{action.port}{action.path}"


def post_json(url: str, timeout: float, body: dict | None = None) -> tuple[int, str]:
    request = urllib.request.Request(
        url,
        data=json.dumps(body or {}).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read().decode("utf-8", errors="replace")
        return response.status, body


def pretty_body(body: str) -> str:
    if not body:
        return "<empty>"
    try:
        parsed = json.loads(body)
    except json.JSONDecodeError:
        return body
    return json.dumps(parsed, ensure_ascii=False, indent=2)


def call_action(host: str, action: ApiAction, timeout: float) -> None:
    url = build_url(host, action)
    print(f"\n>>> {action.title}")
    print(f">>> {action.method} {url}")
    if action.body:
        print(f">>> body: {json.dumps(action.body, ensure_ascii=False)}")
    start = time.perf_counter()
    try:
        status, body = post_json(url, timeout, action.body)
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        print(f"<<< HTTP {exc.code} ({time.perf_counter() - start:.2f}s)")
        print(pretty_body(body))
        return
    except urllib.error.URLError as exc:
        print(f"<<< 请求失败：{exc.reason}")
        return
    except TimeoutError:
        print(f"<<< 请求超时：超过 {timeout}s")
        return
    except OSError as exc:
        print(f"<<< 请求异常：{exc}")
        return

    print(f"<<< HTTP {status} ({time.perf_counter() - start:.2f}s)")
    print(pretty_body(body))


def print_menu(host: str, timeout: float) -> None:
    print("\n" + "=" * 72)
    print("Galbot VLA 接口测试菜单")
    print(f"当前 host: {host}    timeout: {timeout}s")
    print("输入编号调用接口；输入 h 修改 host；输入 m 重打菜单；输入 q 退出。")
    print("-" * 72)

    current_group: Optional[str] = None
    for action in ACTIONS:
        if action.group != current_group:
            current_group = action.group
            print(f"\n[{current_group}]")
        print(f"  {action.key:>2}. {action.title:<28} -> :{action.port}{action.path}")

    print("\n[其他]")
    print("   h. 修改目标 host")
    print("   m. 重新显示菜单")
    print("   q. 退出")
    print("=" * 72)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Galbot VLA HTTP 接口交互式测试菜单")
    parser.add_argument("--host", default="localhost", help="服务所在机器 IP/域名，默认 localhost")
    parser.add_argument("--timeout", type=float, default=5.0, help="HTTP 请求超时时间，默认 5 秒")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    host = args.host
    timeout = args.timeout

    print_menu(host, timeout)

    while True:
        try:
            choice = input("\n请输入编号 > ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n退出。")
            return 0

        if not choice:
            continue

        lower_choice = choice.lower()
        if lower_choice in {"q", "quit", "exit"}:
            print("退出。")
            return 0
        if lower_choice in {"m", "menu"}:
            print_menu(host, timeout)
            continue
        if lower_choice in {"h", "host"}:
            new_host = input(f"请输入新的 host（当前 {host}）> ").strip()
            if new_host:
                host = new_host
                print(f"已切换 host: {host}")
            continue

        action = ACTION_BY_KEY.get(choice)
        if action is None:
            print(f"未知输入：{choice}。请输入 1-{len(ACTIONS)}，或 m/h/q。")            
            continue

        call_action(host, action, timeout)


if __name__ == "__main__":
    sys.exit(main())
