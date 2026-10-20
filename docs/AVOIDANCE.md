# 与 safety 共用相机的无相机遥操

此模式不启动相机和录制器，Quest 使用现实透视。常规采集请用[双臂启动流程](QUICKSTART_COMMANDS.md#6-双臂-vr-遥操与录制)。

## 启动

先在 `../safety` 终端启动检测服务，独占 front/side：

```bash
cd ../safety
/home/mypc/anaconda3/envs/safety/bin/python \
  real_scripts/live_obstacle_detection.py --no-browser --no-lingbot-depth
```

再从本项目根目录启动遥操；实体运动前确认工作空间、活动 TCP、Remote Control 和急停：

```bash
real_scripts/start_dual_vr_teleop_no_cameras.sh --confirm MOVE_BOTH_ARMS
```

脚本启动 Quest 桥接、映射、RTDE 和独立 QP-CBF 计算进程，预检跳过相机检查。使用项目源码及 launch；已有 ROS 环境时无需为该脚本单独重建。

## 操作与行为

- 右 **A** 键切换避障，切换后松开并重新握住侧握把。头显显示状态；录制按钮禁用。
- 每 `50 ms` 从最新手柄目标与关节反馈生成一条受步长约束的指令，检查固定点、扫掠路径和双臂间距；通过则直接发送 `servoJ`。
- 有风险时暂停旧指令、重新读取输入并调用 QP-CBF；过时的目标或反馈对应的结果会丢弃。
- 优化失败、服务不可用或标定不一致时，本步不执行，避障保持开启；新输入会重试，只有手动按 A 才关闭。
- `obstacle_avoidance.ignore_snapshot_age: true`：本模式继续使用超过 `2.0 s` 的 OBB 快照，**可能依据旧障碍位置运动**；其他 safety 程序仍保留过期检查。

常规双臂脚本显式关闭此避障功能，使用普通 `servoL` 路径。两种脚本共用 RTDE 和 Quest 端口，不能同时运行。TCP 距离及软件避障不能替代实体急停和现场监护。

## 环境与退出

| 配置 | 默认值 |
| --- | --- |
| `SAFETY_PYTHON` | `/home/mypc/anaconda3/envs/safety/bin/python` |
| safety 检测服务 | `127.0.0.1:8766` |
| 避障计算进程 | `127.0.0.1:8768`；可设 `QUEST3_AVOIDANCE_PORT=8769` |

故障原因与堆栈见启动终端的 `pose_mapper` / `teleop_avoidance_server` 日志。更新后退出旧 VR 会话并刷新头显页面（`passthrough-v6`）。

`Ctrl-C` 结束遥操及其计算进程；独立检测服务由原终端管理。恢复相机采集前，停止专用脚本及 safety 对相机的占用。脚本的避障环境变量不写回调用终端。
