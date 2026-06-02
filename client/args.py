from typing import List, Optional

OBS_Head_IMG = "observation/image"
OBS_Right_WRIST_IMG = "observation/wrist_image"
OBS_Left_WRIST_IMG = "observation/left_wrist_image"
OBS_PREV_STATE = "observation/prev_state"
OBS_STATE = "observation/state"


class Args:
    # ==========================================================================
    # Global switch: which region below drives the model inputs + init pose.
    #   "general" → Region A (dataset-sourced contract)
    #   "grocery" → Region B (legacy prompt builder + configs_golf/*.json)
    # Region B is kept for back-compat; new deployments should prefer "general".
    # ==========================================================================
    input_source: str = "general"

    # ==========================================================================
    # Region A — general dataset-sourced inputs
    # --------------------------------------------------------------------------
    # Mirror the datacook format (see datacook/FORMAT.md):
    #   - task          : tasks.jsonl → "task" field, fed verbatim to the model
    #   - task_images   : tasks.jsonl → "task_images" (relative paths)
    #   - init_pose     : meta/init_pose.json → "init_pose" (23-dim wire layout)
    # Only consumed when input_source == "general".
    # ==========================================================================

    # 垃圾袋
    # task : str = "Put a garbage bag in the trash can."

    # 桌面
    task: str = "Pick up the bag and place it on the table."
    # task : str = "Put the large objects on the table into the bag."
    # task : str = "Sweep the remaining trash on the table into the white basin, then put it into the bag."
    # task : str = "Lift up the bag."

    # 地面
    # task : str = "Put the garbage on the ground one by one into the trash can until there is no more garbage on the ground."

    task_images: List[str] = []
    # Path (relative to project root) to a datacook init_pose.json artifact.
    # "" means "not set"; required in general mode.  See config/init_pose/.
    # init_pose_file: str = "config/init_pose/zhiyuan_pick_trash.json"          # 地面 垃圾袋
    init_pose_file: str = "config/init_pose/zhiyuan_pick_trash_stand.json"  # 桌面

    # ==========================================================================
    # Region B — grocery legacy prompt/pose builder
    # --------------------------------------------------------------------------
    # prompt_type + object_* + shelf_* get templated into the model prompt;
    # prompt_type also selects which configs_golf/*.json to read init_pose from.
    # Only consumed when input_source == "grocery".
    # ==========================================================================
    prompt_type: str = "shelf_stock"
    object_name: List[str] = ["dongfangshuyeqingganpuer"]
    object_location: List[int] = [1, 1]
    object_location_on_shelf: str = "left"  # "left" | "middle" | "right"
    hand_used: str = "left"  # "left" | "right"
    shelf_location = ["front"]  # "left" | "front" | "right"
    shelf_type: List[int] = [3]
    config_path: str = "configs_golf"

    # ==========================================================================
    # Region C — network / control / image / misc (common to both modes)
    # ==========================================================================
    # host = ["10.0.100.34"]
    host = ["192.168.1.99"]
    port = [6686]  # 桌面
    # port = [6688]  # 地面
    # port=[6687]    # 垃圾袋

    # None → fall back to init_pose[14] / init_pose[22] (i.e. whatever the
    # Region A init_pose or Region B config file carried).  Non-empty list
    # always overrides those two dims.
    gripper_init_width: Optional[List[float]] = None

    action_horizon: int = 30
    action_horizon_use: int = 30
    proprio_step: int = 3  # 包括当前，有几帧动作
    # blocking: bool = True  # 阻塞
    blocking: bool = False  # 非阻塞

    dt_model_control: float = 0.06
    enable_chassis: float = 1.0
    enable_takeover: float = 1.0
    network_latency_tolerance: float = 60.0  # seconds

    target_image_size_head: List[int] = [224, 224]  # [横向, 纵向]
    target_image_size_left_arm: List[int] = [224, 224]
    target_image_size_right_arm: List[int] = [224, 224]
    raw_image_size_head: List[int] = [1280, 960]

    raw_image_size_left_arm: List[int] = [1280, 720]  # 工作用大的
    raw_image_size_right_arm: List[int] = [1280, 720]
    # raw_image_size_left_arm:  List[int] = [640, 360]  # 数采用小的
    # raw_image_size_right_arm: List[int] = [640, 360]

    gripper_vel: float = 100  # 0-200, 负数表示按规划速度运动
    gripper_effort: float = 70
    lim_vel: float = 1.0
    lim_acc: float = 6.0
    damping_D: float = 5
    stiffness_K: float = 4
    lambda_para: List[float] = [0.1, 0]
    traj_filtering: float = -1.0
    para_chassis_p: List[float] = [1.0, 1.0, 1.0]

    vla_type: str = (
        "VLA"  # "VLA" | "VLA_training_time_RTC" | "PlanEndEffector_Vla_WeightingPosByAccCtrl"
    )
    control_mode: str = "joint"  # "eef" | "joint"
    has_init_action: bool = True
    auto_stop: bool = False
    auto_stop_distance: float = 0.15
    success_gripper_width: float = 0.02
    allow_retry: bool = True
    retry_fail_max_num: int = 3
    vla_max_cost_time: float = 40.0

    visualization: bool = False
    profile_log: bool = True
    websocket_backend: str = "auto"
    image_crop_and_resize_method: str = "auto"  # 没用的参数
    cal_T_count: int = 10

    robot_type: str = "galbot_golf_full"
    gripper_width: float = 0.129
