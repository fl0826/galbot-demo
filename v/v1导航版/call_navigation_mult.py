#!/usr/bin/env python3
"""
导航控制脚本 - 通过编号选择点位进行导航
点位来自 location_map.json
"""

import sys
import time
sys.path.insert(0, "/home/galbot/vla_client/vla_tree")

from galbot_sdk.g1 import GalbotNavigation, GalbotRobot, ControllerName, ControlStatus


LOCATIONS = {
    "hotel": {
        "id": 1,
        "name": "垃圾站",
        "pose": [0.34035560488700867, -0.21991275250911713, -0.03312307223677635,
                 0.006063402390240334, 0.011393127441231209, -0.7401593484398675, 0.6723076458851504]
    },
    "trash_bag": {
        "id": 2,
        "name": "卧室",
        "pose": [0.22136326134204865, 1.9452053308486938, 0.0017818112391978502,
                 0.0008642711869409962, -0.002559838169778184, 0.6946860574868778, 0.7193079881366524]
    },
    "floor_cleaning": {
        "id": 3,
        "name": "客厅",
        "pose": [1.4839913845062256, -0.04077132046222687, -0.0101028922945261,
                 -0.0010384007529652185, 0.0015685286818179016, -0.017434208715043042, 0.9998462430834708]
    },
    "desk_cleaning": {
        "id": 4,
        "name": "餐厅",
        "pose": [4.849914073944092, -0.4569421112537384, -0.04835652559995651,
                 -0.006090641443757879, 0.0047637668949906935, -0.7231525229209531, 0.6906450891773822]
    },
    "location_1779959795": {
        "id": 5,
        "name": "酒店门口",
        "pose": [0.46595120429992676, 0.5645597577095032, -0.022063903510570526,
                 0.007339836193347319, -0.006462030344018104, 0.9996014751104988, 0.026481312758294864]
    },
    "location_1779963425": {
        "id": 6,
        "name": "床边",
        "pose": [3.91005539894104, -0.1382303684949875, -0.0417376272380352,
                 -0.0012733309843662465, -0.006097442178923918, 0.28388375346635264, 0.9588384714564462]
    },
    "location_1780293925": {
        "id": 7,
        "name": "灶具",
        "pose": [1.267265796661377, 0.2779254615306854, -0.03721865639090538,
                 0.00233109263562663, -0.0037630659897629295, 0.48519386099427714, 0.8743954040335223]
    },
    "location_1780293956": {
        "id": 8,
        "name": "洗衣机",
        "pose": [0.7360139489173889, 2.975029468536377, -0.023249110206961632,
                 0.0006226194538654948, -0.006411323044326261, 0.9726316388225444, 0.23226321801613026]
    }
}

# id -> key 反查表，用于编号输入
_ID_TO_KEY = {info["id"]: key for key, info in LOCATIONS.items()}


def print_menu():
    print("\n" + "=" * 50)
    print("  导航点位列表")
    print("=" * 50)
    for key, info in sorted(LOCATIONS.items(), key=lambda x: x[1]["id"]):
        print(f"  {info['id']:>2}.  {info['name']}")
    print("-" * 50)
    print("   m.  重新显示菜单")
    print("   q.  退出")
    print("=" * 50)


def navigate_to_location(nav, location_key: str) -> bool:
    location = LOCATIONS[location_key]
    pose = location["pose"]
    name = location["name"]

    print(f"\n>>> 导航到: [{location['id']}] {name}")
    print(f"    x={pose[0]:.4f}  y={pose[1]:.4f}  z={pose[2]:.4f}")
    print(f"    qx={pose[3]:.4f} qy={pose[4]:.4f} qz={pose[5]:.4f} qw={pose[6]:.4f}")

    try:
        current_pose = nav.get_current_pose()
        print(f"    当前位姿: {current_pose}")

        if not nav.check_path_reachability(pose, current_pose):
            print(f"<<< 路径不可达: {name}")
            return False

        print(f"    路径可达，开始导航...")

        retry_count = 1
        while retry_count > 0:
            nav.navigate_to_goal(
                pose,
                enable_collision_check=True,
                is_blocking=True,
                timeout=30
            )

            time.sleep(0.5)

            if nav.check_goal_arrival():
                print(f"<<< 成功到达: {name}")
                return True
            else:
                retry_count -= 1
                if retry_count > 0:
                    print(f"    导航失败，剩余重试: {retry_count}")

        print(f"<<< 导航失败: {name}")
        return False

    except Exception as e:
        print(f"<<< 导航异常: {name} - {e}")
        return False


def main():
    print("\n" + "="*60)
    print("初始化机器人导航系统...")
    print("="*60)
    
    robot = GalbotRobot.get_instance()
    nav = GalbotNavigation.get_instance()
    
    try:
        if not robot.init():
            print("❌ 机器人初始化失败")
            return
        print("✅ 机器人初始化成功")
        
        if not nav.init():
            print("❌ 导航系统初始化失败")
            return
        print("✅ 导航系统初始化成功")
        
        time.sleep(1)
        
        res = robot.switch_controller(ControllerName.CHASSIS_POSE_CTRL)
        if res != ControlStatus.SUCCESS:
            print("❌ 切换到位姿控制模式失败")
            return
        print("✅ 已切换到位姿控制模式")
        
        print_menu()
        
        while True:
            try:
                user_input = input("\n请输入点位编号 > ").strip()
            except (KeyboardInterrupt, EOFError):
                print("\n退出。")
                break
            
            if not user_input:
                continue
            
            lc = user_input.lower()
            if lc in {"q", "quit", "exit"}:
                print("退出。")
                break
            if lc in {"m", "menu"}:
                print_menu()
                continue
            
            try:
                loc_id = int(user_input)
            except ValueError:
                print(f"无效输入: '{user_input}'，请输入点位编号或 m/q")
                continue
            
            if loc_id not in _ID_TO_KEY:
                print(f"编号 {loc_id} 不存在，有效范围 1-{len(LOCATIONS)}")
                continue
            
            navigate_to_location(nav, _ID_TO_KEY[loc_id])
    
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
    
    finally:
        print("\n清理资源...")
        nav.stop_navigation()
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        print("✅ 资源释放完成")


if __name__ == "__main__":
    main()
