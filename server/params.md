1. host = [""]（模型ip）（支持输入）
  比如10.119.0.0/16
2. port = []（使用端口）（支持输入）
  从6666到8888
3. object_name: List[str] = ["sku名称"]（支持拼音检索）
  使用物体库中的名称，前端传来，后端别管
4. object_location: List[int] = [层, 列]（支持输入）
  层：1、2、3、4、11、12、13、14、15、21、22、23、24、25、
  列：1、2、3、4、5、6、
5. blocking: bool = （阻塞/非阻塞运动模式）（选项）
  False or True
6. action_horizon_use: int =（实际执行步长）（支持输入）
  数字：3、4、5、6、7、8、9、10（这只是可能的输入，不做限制）
7. dt_model_control: float = （模型时间间隔）（支持输入）
  浮点数：0.30~0.85
8. prompt_type: str = （提示词）（选项）
  1. fix_shelf_freezer（定台冷柜）
  2. fix_shelf_general（定台层板）
  3. wb_shelf_freezer（转身冷柜）
  4. wb_shelf_general（转身层板）
  5. shelf_stock（上货）
  6. czy（上货）
9. object_location_on_shelf: str = ' ' （sku在货架的左右分布）（选项）
  1. "left", "middle", "right"
10. hand_used: str = ' '（使用手）（选项）
  1. "left", "right"
新增
11. action_horizon: int = （模型推理步长）（支持输入）
  1. 3、5、6、7、8、10
12. shelf_location: str = [' '] （机器人与货架位置对应关系）（选项）
  1. "left", "front", "right"
13. gripper_vel: float = 100  # 0-200, 负数表示按规划速度运动 （夹具闭合速度）（支持输入）
14. gripper_effort: float = 70（夹具闭合力度）（支持输入）
15. gripper_init_width: List[float] = [0.075,0.090] （左右手初始夹抓宽度）（支持输入）
16. target_image_size_head:      List[int] = [1280,960] #[横向尺寸， 纵向尺寸] （头部相机分辨率）（选项）
    target_image_size_left_arm:  List[int] = [1280,720]#[横向尺寸， 纵向尺寸] （左腕部相机分辨率）
    target_image_size_right_arm: List[int] = [1280,720]#[横向尺寸， 纵向尺寸] （右腕部相机分辨率）
  1. List[int] = [640,480]
     List[int] = [398,224]
     List[int] = [398,224]
  2. List[int] = [224,224]
     List[int] = [224,224]
     List[int] = [224,224]