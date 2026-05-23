# galbot-demo
demo存档


| 文件 | 类型 | 用途 |
|------|------|------|
| `table.py` | 交互式 | 终端输入 1/2/3/4/5/s/q 手动切换桌面四个子任务 |
| `floor.py` | 交互式 | 终端输入 1/2/s/q 手动控制清理地面 |
| `bag.py` | 交互式 | 终端输入 1/2/3/s/q 手动控制套垃圾袋（含断点续推） |
| `auto_table.py` | 自动 | 启动后按顺序执行桌面四个子任务，根据 action delta 自动切换 |
| `robot_server_clear_table.py` | HTTP 服务 | curl 触发，端口 9052 |
| `robot_server_clean_floor.py` | HTTP 服务 | curl 触发，端口 9051 |
| `robot_server_put_garbage_bag.py` | HTTP 服务 | curl 触发，端口 9053 |

