# Quest 3 + 双 UR7e 遥操作与数据采集

Quest 3 左右手柄分别控制两台 UR7e；同步保存双腕、front/side 四路 RGB 和机器人状态到 HDF5。深度按需开启。

| 文档 | 内容 |
| --- | --- |
| [安装](docs/INSTALLATION.md) | Ubuntu 24.04、ROS 2 Jazzy、构建与环境 |
| [命令快查](docs/QUICKSTART_COMMANDS.md) | 设备参数、预检、启动、录制、视频导出 |
| [配置与标定](docs/CALIBRATION.md) | TCP、双臂双相机外参、断点续做 |
| [系统与数据](docs/SYSTEM_REFERENCE.md) | 控制逻辑、HDF5 格式、丢帧诊断 |
| [无相机避障遥操](docs/AVOIDANCE.md) | 与独立 safety 服务共用相机 |

## 启动

安装并完成 TCP、四份相机外参及双臂换算标定后，在仓库根目录执行：

```bash
conda deactivate 2>/dev/null || true
source real_scripts/ros_environment.sh
ros2 run quest3_robot_data preflight --config config/hardware_dual_teleop.yaml
real_scripts/start_dual_vr_teleop.sh --confirm MOVE_BOTH_ARMS --task "pick place"
```

**实体运动前**：清空工作空间，确认急停可触达、UR 为 Remote Control/运行状态，且活动 TCP 为各自已标定的夹爪 TCP。先分别低速空载验证两臂。

Quest 通过 USB 连接并接受调试授权。启动脚本自动设置 ADB 转发；在 Quest Browser 打开 `http://localhost:8765`，点击“连接”→“进入现实透视”，等左右手柄均被追踪。

| 操作 | 功能 |
| --- | --- |
| 按住左/右侧握把 | 移动对应机械臂；松开停止 |
| 左/右扳机 | 闭合对应夹爪；松开张开 |
| 左 Y / 右 B | 开始录制 / 停止并保存 |
| 终端 `Ctrl-C` | 停止全部节点并结束当前录制 |

默认只采集 RGB 和机器人数据；需要 front/side 深度时，在启动命令末尾加 `--record-depth`。单臂使用 `start_left_vr_teleop.sh --confirm MOVE_LEFT_ARM --task "pick place"`，同样支持该选项。

## 检查与导出

双臂数据按任务分目录；`--task "pick place"` 对应 `data/dual_vr_episodes/pick_place/`。

```bash
# 检查帧间隔和数据对齐
ros2 run quest3_robot_data inspect_episode \
  data/dual_vr_episodes/pick_place/episode_00000.hdf5

# 批量导出该任务的四路 H.264 MP4（默认 30 FPS）
ros2 run quest3_robot_data export_task_episode_videos \
  data/dual_vr_episodes/pick_place
```

视频位于各 episode 同级的 `episode_00000_videos/`；已完整导出的 H.264 视频自动跳过。单个 episode 导出及旧文件夹数据转换见[命令快查](docs/QUICKSTART_COMMANDS.md#7-数据目录与视频导出)。

## 采集性能

默认 `rgb_storage: raw`，无损、无压缩；图像转换和写盘在后台完成，front 参考帧按顺序处理。四路 `640×480 RGB @30 FPS` 约需 **111 MB/s、6.6 GB/分钟**。使用持续写入性能足够的磁盘；可选 `raw_lzf` 无损压缩或 `jpeg` 有损压缩以节省空间。

运行 `ros2 topic echo /quest3/system_status` 查看 `writer_queue`、`dropped_frames` 和 `drop_reasons`。排查方法见[丢帧诊断](docs/SYSTEM_REFERENCE.md#丢帧诊断)；其余连接、追踪和停机问题见[运行诊断](docs/QUICKSTART_COMMANDS.md#8-常用运行时诊断)。
