"""
任务导航脚本 - 支持多种任务点的导航
包含重新定位、路径检查和导航功能
支持中文和数字命令输入
"""
from galbot_sdk.g1 import GalbotNavigation, GalbotRobot, ControllerName, ControlStatus
import time
import json
import argparse
import os


def navigate_to_point(nav, target_pose, point_name="目标点"):
    """
    导航到指定点位

    Parameters:
        robot (GalbotRobot): 机器人实例
        nav (GalbotNavigation): 导航实例
        target_pose (list): 目标位姿 [x, y, z, qx, qy, qz, qw]
        point_name (str): 点位名称，用于日志输出

    Returns:
        bool: 是否成功到达目标点
    """
    try:
        current_pose = nav.get_current_pose()
        print(f"当前位姿: {current_pose}")
        print(f"开始导航到{point_name}: {target_pose}")

        # 检查路径可达性
        if nav.check_path_reachability(target_pose, current_pose):
            print(f"路径可达，开始导航到{point_name}")

            retry_count = 3
            while retry_count > 0:
                # 执行导航
                status = nav.navigate_to_goal(
                    target_pose,
                    enable_collision_check=True,
                    is_blocking=True,
                    timeout=30
                )

                time.sleep(0.5)

                # 检查是否到达目标
                if nav.check_goal_arrival():
                    print(f"✅ 成功到达{point_name}")
                    final_pose = nav.get_current_pose()
                    print(f"最终位姿: {final_pose}")
                    return True
                else:
                    retry_count -= 1
                    print(f"导航失败，剩余重试次数: {retry_count}")
                    print(f"导航状态: {status}")

        else:
            print(f"❌ 路径不可达或不安全，无法到达{point_name}")
            return False

    except Exception as e:
        print(f"导航到{point_name}过程中发生异常: {e}")
        return False

def load_task_locations(config_path):
    """从配置文件加载任务点位信息

    Parameters:
        config_path (str): 配置文件路径

    Returns:
        dict: 任务点位信息字典
    """
    config_path = os.path.expanduser(config_path)
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    with open(config_path, 'r', encoding='utf-8') as f:
        locations = json.load(f)

    print(f"✅ 成功加载配置文件: {config_path}")
    print(f"📍 共加载 {len(locations)} 个点位")

    return locations

def print_available_commands(locations):
    """打印可用命令"""
    print("\n📍 可用导航命令：")
    print("=" * 50)

    for key, info in locations.items():
        commands = info.get("commands", [str(info.get("id", ""))])
        commands_str = "、".join(commands)
        print(f"• {commands_str} -> {info['name']}")

    print("=" * 50)
    print("💡 提示：输入 'help' 查看命令列表，输入 'add' 添加新位置，输入 'quit' 或 'exit' 退出程序")

def relocalize(nav, locations):
    """交互式重定位：让用户选择当前所在位置，使用对应位姿进行重定位"""
    print("开始重新定位...")

    if nav.is_localized():
        print("✅ 已定位")
        return

    print("\n📍 请选择机器人当前所在位置：")
    for i, (key, info) in enumerate(locations.items()):
        print(f"  {i}: {info['name']}")

    while True:
        choice = input("请输入位置编号: ").strip()
        try:
            idx = int(choice)
            keys = list(locations.keys())
            if 0 <= idx < len(keys):
                selected_key = keys[idx]
                relocalize_pose = locations[selected_key]["pose"]
                print(f"使用 [{locations[selected_key]['name']}] 的位姿进行重定位")
                break
            else:
                print(f"请输入 0 到 {len(keys) - 1} 之间的数字")
        except ValueError:
            print("输入无效，请输入数字编号")

    max_retries = 10
    for attempt in range(max_retries):
        nav.relocalize(relocalize_pose)
        time.sleep(1.0)
        if nav.is_localized():
            print("✅ 重定位成功!")
            return
        print(f"重定位中，重试 ({attempt + 1}/{max_retries})...")

    print("❌ 重定位失败，已达最大重试次数")
    exit(1)

def parse_command(user_input, locations):
    """解析用户输入的命令"""
    user_input = user_input.strip()

    if user_input.lower() in ['quit', 'exit', 'q']:
        return "quit"

    if user_input.lower() in ['help', 'h']:
        return "help"

    if user_input.lower() == 'add':
        return "add"

    for key, info in locations.items():
        commands = info.get("commands", [info["name"], str(info.get("id", ""))])
        if user_input in commands:
            return key

    return None

def add_new_location(nav, locations, config_path):
    """添加新位置到配置文件

    Parameters:
        nav (GalbotNavigation): 导航实例
        locations (dict): 当前位置字典
        config_path (str): 配置文件路径

    Returns:
        dict: 更新后的位置字典
    """
    print("\n📍 添加新位置")
    print("=" * 50)
    print("请将机器人手动推到目标位置...")
    input("按回车键继续...")

    # 获取当前位姿
    current_pose = nav.get_current_pose()
    print(f"当前位姿: {current_pose}")

    # 输入位置名称
    while True:
        location_name = input("请输入位置名称: ").strip()
        if location_name:
            break
        print("位置名称不能为空，请重新输入")

    # 生成位置 key（使用拼音或简化名称）
    location_key = input("请输入位置标识符 (英文，用于内部标识，留空自动生成): ").strip()
    if not location_key:
        # 简单生成：使用时间戳
        location_key = f"location_{int(time.time())}"

    update_dict = {
        "name": location_name,
        "pose": current_pose.tolist() if hasattr(current_pose, 'tolist') else current_pose
    }

    if location_key in locations:
        locations[location_key].update(update_dict)
    else:
        max_id = max([info.get("id", 0) for info in locations.values()], default=0)
        new_id = max_id + 1
        new_loc = {
            "id": new_id
        }
        new_loc.update(update_dict)
        locations[location_key] = new_loc

    # 保存到配置文件
    config_path = os.path.expanduser(config_path)
    try:
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(locations, f, ensure_ascii=False, indent=4)
        print(f"✅ 新位置已保存到配置文件: {config_path}")
        print(f"   名称: {location_name}")
        print(f"   标识符: {location_key}")
    except Exception as e:
        print(f"❌ 保存配置文件失败: {e}")

    return locations

def main():
    """主函数：任务导航"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='机器人任务导航系统')
    parser.add_argument(
        '--config',
        type=str,
        default='/userdata/galbot_tree/galbot_g1/config/location_map.json',
        help='配置文件路径'
    )
    args = parser.parse_args()

    # 加载配置文件
    try:
        locations = load_task_locations(args.config)
    except FileNotFoundError as e:
        print(f"❌ {e}")
        return
    except json.JSONDecodeError as e:
        print(f"❌ 配置文件格式错误: {e}")
        return
    except Exception as e:
        print(f"❌ 加载配置文件失败: {e}")
        return

    # 初始化机器人和导航
    robot = GalbotRobot.get_instance()
    nav = GalbotNavigation.get_instance()

    try:
        # 初始化
        if robot.init():
            print("✅ 机器人初始化成功")
        else:
            print("❌ 机器人初始化失败")
            return

        if nav.init():
            print("✅ 导航系统初始化成功")
        else:
            print("❌ 导航系统初始化失败")
            return

        # 等待数据准备
        time.sleep(1)

        # 切换到位姿控制器
        res = robot.switch_controller(ControllerName.CHASSIS_POSE_CTRL)
        if res != ControlStatus.SUCCESS:
            print("❌ 切换控制器失败！")
            exit(1)
        else:
            print("✅ 切换控制器成功！")

        # 重新定位
        relocalize(nav, locations)
        current_pose = nav.get_current_pose()
        print(f"当前位姿: {current_pose}")

        # 显示可用命令
        print_available_commands(locations)

        # 主循环：等待用户输入命令
        print("\n🚀 任务导航系统已启动，请输入命令：")

        while True:
            try:
                user_input = input("\n请输入导航命令: ").strip()

                if not user_input:
                    continue

                command = parse_command(user_input, locations)

                if command == "quit":
                    print("👋 退出程序...")
                    break
                elif command == "help":
                    print_available_commands(locations)
                    continue
                elif command == "add":
                    locations = add_new_location(nav, locations, args.config)
                    continue
                elif command is None:
                    print(f"❌ 无法识别命令: '{user_input}'")
                    print("💡 输入 'help' 查看可用命令")
                    continue

                # 执行导航
                target_info = locations[command]

                print(f"\n🎯 目标: {target_info['name']}")
                print(f"📍 目标位姿: {target_info['pose']}")

                success = navigate_to_point(nav, target_info['pose'], target_info['name'])

                if success:
                    print(f"🎉 成功到达{target_info['name']}！")
                    print("✨ 任务完成，可以继续输入下一个命令")
                else:
                    print(f"⚠️ 未能到达{target_info['name']}，请检查路径或重试")

            except KeyboardInterrupt:
                print("\n\n⏹️  用户中断程序")
                break
            except Exception as e:
                print(f"\n❌ 程序执行过程中发生异常: {e}")

    except Exception as e:
        print(f"程序执行过程中发生异常: {e}")

    finally:
        # 停止导航
        nav.stop_navigation()
        print("导航已停止")

        # 释放资源
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        print("✅ 资源释放成功")

if __name__ == "__main__":
    main()
