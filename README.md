# galbot-demo

银河 galbot 机器人 VLA demo 存档。

## 目录结构

```
galbot-demo/
├── client/          # 机器人端脚本（交互式 / 自动）
├── client_server/   # HTTP 服务 + 工具脚本
├── bash/            # 推理服务器：拉模型 / 启动模型服务
├── traj/            # 采集轨迹 parquet 文件
├── config/          # 初始位姿和高度配置
└── version/               # 历史版本存档（v0 / v1导航版）
```

## 任务与端口


| 任务                     | 端口   |
| ---------------------- | ---- |
| 清理桌面 (clear_table)     | 9052 |
| 清理地面 (clean_floor)     | 9051 |
| 套垃圾袋 (put_garbage_bag) | 9053 |


## 运行模式

**交互式**（终端按键手动控制）


| 脚本                | 按键                           |
| ----------------- | ---------------------------- |
| `client/table.py` | `1/2/3/4` 切子任务，`s` 停止，`q` 退出 |
| `client/floor.py` | `1` 启动，`2` 复位，`s` 停止，`q` 退出  |
| `client/bag.py`   | `1/2/3` 切阶段，`s` 停止，`q` 退出    |


**自动模式**：`client/auto_table_124.py`，自动依次执行桌面子任务 1→2→4。

**HTTP 服务模式**：修改 `start.sh` 顶部的 IP，一键启动三个服务 + 交互菜单。

```bash
bash start.sh
```

三个服务日志分别写入 `logs/`，菜单里输入编号调用接口。

## 工具脚本

以下脚本均在 `client_server/` 目录下运行。

**复位**

```bash
python reset.py                # 桌面位姿（默认）
python reset.py --pose floor   # 地面位姿
```

**位姿工具**

```bash
python pose_tool.py get --out my_pose.json          # 获取当前位姿  保存到 my_pose.json 
python pose_tool.py reset --pose-file my_pose.json  # 移动到指定位姿 my_pose.json 
```

**轨迹回放**（验证 `traj/` 下的采集数据）

```bash
python replay_downsample.py --parquet ../traj/导航垃圾桶.parquet --step 15 --speed 0.8
```

**相机预览**（浏览器访问 `http://<机器人IP>:8080`）

```bash
python camera_viewer_1.py   # 仅头部相机
python camera_viewer_3.py   # 头部 + 双臂三路相机
```

