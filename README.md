# galbot-demo

银河 galbot 机器人 VLA demo 存档。

## 目录结构

```
galbot-demo/
├── client/          # 机器人端核心 VLA 调用脚本
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


## HTTP 接口

以下接口均为 `POST`，状态查询和健康检查为通用接口：

- `GET /api/status`：任务状态
- `GET /api/health`：健康检查

### 套垃圾袋服务 `put_garbage_bag`（端口 `9053`）


| 接口                            | 功能              |
| ----------------------------- | --------------- |
| `/api/put_garbage_bag`        | 正常启动套垃圾袋任务，包含复位 |
| `/api/put_garbage_bag_resume` | 断点续推，跳过复位       |
| `/api/reset`                  | 仅复位，不推理         |
| `/api/stop`                   | 停止当前任务          |


### 打扫地面服务 `clean_floor`（端口 `9051`）


| 接口                 | 功能             |
| ------------------ | -------------- |
| `/api/clean_floor` | 启动打扫地面推理任务，不复位 |
| `/api/reset`       | 仅复位            |
| `/api/stop`        | 停止当前任务         |


### 清理桌面服务 `clear_table`（端口 `9052`）


| 接口                     | 功能          |
| ---------------------- | ----------- |
| `/api/pick_bag`        | 升降取垃圾袋，包含复位 |
| `/api/bag_large_items` | 桌面大物品清理     |
| `/api/sweep_trash`     | 抹布清理龙虾    |
| `/api/lift_bag`        | 提起袋子        |
| `/api/reset`           | 复位到桌面默认位姿   |
| `/api/open_gripper`    | 松开夹爪        |
| `/api/close_gripper`   | 闭合夹爪        |
| `/api/stop`            | 停止当前任务      |


## 运行模式

**HTTP 服务模式**：修改 `start.sh` 顶部的模型服务器 IP，一键启动三个服务 + 交互菜单。

```bash
bash start.sh
```

三个服务日志分别写入 `logs/`，菜单里输入编号调用接口。

## 工具脚本

以下脚本均在 `client_server/` 目录下运行。

**复位**

```bash
python reset.py              # 默认复位到桌面任务初始位姿
python reset.py --pose floor # 复位到地面/垃圾袋任务初始位姿
```

**位姿工具**

```bash
python pose_tool.py get --out my_pose.json          # 获取当前位姿，保存到 my_pose.json
python pose_tool.py reset --pose-file my_pose.json  # 移动到指定位姿 my_pose.json
```

**轨迹回放**（验证 `traj/` 下的采集数据）

```bash
python replay_downsample.py --parquet ../traj/导航垃圾桶.parquet --step 15 --speed 0.8
```

**日志拉取**

```bash
python pull_logs.py  # 按脚本顶部配置拉取机器人日志
```

**相机预览**（浏览器访问 `http://<机器人IP>:8080`）

```bash
python camera_viewer_1.py   # 仅头部相机
python camera_viewer_3.py   # 头部 + 双臂三路相机
```

