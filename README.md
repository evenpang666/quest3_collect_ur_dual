# Quest 3 + 双 UR7e 遥操作与数据采集

通过 Quest 3 的 WebXR 手柄控制两台 UR7e，并同步采集两台腕部 D435i、front/side D455 与机器人状态，保存为 HDF5 episode。左、右手柄分别控制左、右机械臂；侧握把负责运动离合，扳机控制对应夹爪。

> **实体运动前**：清空工作空间，确认急停可触达、两台 UR 处于 Remote Control/运行状态，且示教器的活动 TCP 为各自已标定的夹爪 TCP。先完成标定和只读预检，再分别低速验证左右臂。松开侧握把可停止对应臂；遇到碰撞或线缆风险，使用实体急停。

| 从这里开始 | 文档 |
| --- | --- |
| 当前现场的完整命令、设备参数与诊断 | [命令快查](docs/QUICKSTART_COMMANDS.md) |
| ROS 2 Jazzy 安装与构建细节 | [安装说明](docs/INSTALLATION.md) |
| 硬件配置、TCP 与双相机标定 | [配置与标定](docs/CALIBRATION.md) |
| 系统结构、安全机制与 HDF5 格式 | [系统与数据参考](docs/SYSTEM_REFERENCE.md) |
| 早期左臂只读调试记录 | [调试归档](docs/LEFT_ARM_DEBUG_HISTORY.md) |

## 安装

当前主机使用 **Ubuntu 24.04 + ROS 2 Jazzy + 系统 Python 3.12**。在新终端中按 [ROS 2 Jazzy 官方安装步骤](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)配置 Noble 软件源；若系统是从 Jammy 升级来的，先按[完整安装说明](docs/INSTALLATION.md)处理旧 ROS 软件源。不要加载 Humble 或 Conda 环境。

```bash
sudo apt install -y ros-jazzy-ros-base ros-jazzy-realsense2-camera \
  ros-jazzy-cv-bridge ros-jazzy-launch-ros \
  python3-opencv python3-aiohttp python3-h5py python3-numpy \
  python3-venv python3-colcon-common-extensions python3-rosdep

/usr/bin/python3 -m venv --system-site-packages ~/ros2_ws/pydeps_jazzy
~/ros2_ws/pydeps_jazzy/bin/python -m pip install --no-deps ur-rtde pyrealsense2
```

从仓库根目录构建 Jazzy 工作区：

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
test -e ~/ros2_ws/src/quest3_robot_data || \
  ln -s "$PWD" ~/ros2_ws/src/quest3_robot_data
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon --log-base log_jazzy build --symlink-install --packages-select quest3_robot_data \
  --build-base build_jazzy --install-base install_jazzy
source /home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/ros_environment.sh
ros2 pkg prefix quest3_robot_data
```

## 快速使用

以下命令默认从仓库根目录执行。每个新终端先加载环境：

```bash
conda deactivate 2>/dev/null || true
source real_scripts/ros_environment.sh
```

1. 按[配置与标定](docs/CALIBRATION.md)检查硬件身份、活动 TCP、左右臂 front/side 四份外参与双臂换算汇总。当前现场的标定命令及验收条件见[命令快查](docs/QUICKSTART_COMMANDS.md#3-标定文件状态)。
2. 连接 Quest USB，接受头显中的调试授权；执行 `adb devices` 和 `adb reverse tcp:8765 tcp:8765`。
3. 先运行只读预检，并确认输出 `PREFLIGHT PASS`。实体双臂配置会显示 `MOTION GATE: ENABLED`，因此预检通过后也须先确认现场安全条件。

```bash
ros2 run quest3_robot_data preflight \
  --config config/hardware_dual_teleop.yaml
```

完成标定、左右手验证和低速空载测试后，启动双臂遥操作及录制：

```bash
real_scripts/start_dual_vr_teleop.sh --confirm MOVE_BOTH_ARMS \
  --task "pick place"
```

只使用左臂时：

```bash
real_scripts/start_left_vr_teleop.sh --confirm MOVE_LEFT_ARM \
  --task "pick place"
```

在 Quest Browser 打开 `http://localhost:8765`，点击“连接”→“进入现实透视”。按住侧握把移动对应机械臂；左手 **Y** 或右手 **B** 切换录制。按 `Ctrl-C` 结束。episode 保存路径、视频导出和手柄操作细节见[命令快查](docs/QUICKSTART_COMMANDS.md#6-双臂-vr-遥操与录制)。

### 录制数据转视频

录制结束后，在仓库根目录执行以下命令。`--task "pick place"` 的数据位于 `data/dual_vr_episodes/pick_place/`。

```bash
# 转换单个 HDF5 episode
ros2 run quest3_robot_data export_episode_videos \
  data/dual_vr_episodes/pick_place/episode_00000.hdf5

# 转换该任务目录下的全部 HDF5 episode
ros2 run quest3_robot_data export_task_episode_videos \
  data/dual_vr_episodes/pick_place
```

每个 episode 的视频保存在同目录的 `episode_00000_videos/`，包含 front、side、左腕和右腕四路 MP4。默认帧率为 30 FPS；需要覆盖已有视频时，在命令末尾加 `--overwrite`。其他任务请将 `pick_place` 换成对应任务目录名。

若与独立的 `../safety` 检测服务共用相机，可使用[无相机双臂遥操作流程](docs/QUICKSTART_COMMANDS.md#与-safety-同时运行无相机双臂遥操)；该模式不启动相机或录制器。

## 常见问题

| 现象 | 先检查 |
| --- | --- |
| `rclpy` / `rtde_control` / `pyrealsense2` 导入失败 | 在新终端重新加载 `real_scripts/ros_environment.sh`；确认使用 Jazzy 系统 Python 与独立 `pydeps_jazzy`。见[安装说明](docs/INSTALLATION.md)。 |
| Quest 未被 ADB 检测到 | 确认头显已开机并解锁、USB 数据线已接到当前主机；运行 `lsusb` 和 `adb devices -l`。若两处均无 Quest，检查线缆、接口和头显连接。 |
| Quest ADB 显示 `unauthorized`、`offline` 或连接超时 | 在头显中接受 USB 调试授权，必要时重新插拔线缆，运行 `adb kill-server && adb start-server && adb devices -l`；设备状态应为 `device`。 |
| Quest 页面无法进入现实透视 | 确认 `adb reverse --list` 含 `tcp:8765`，退出旧沉浸式会话并刷新 Quest Browser 页面。 |
| 相机打不开或预检缺少序列号 | 停止其他占用 RealSense 的程序，执行 `real_scripts/view_front_cam.sh` 检查已连接相机。预览结束后按 `q` 退出，再启动遥操作。 |
| 预检拒绝双臂启动 | 检查四份外参及双臂换算汇总均为 `calibrated: true`，并确认相机、底座与 TCP 未在标定后移动。见[标定命令](docs/QUICKSTART_COMMANDS.md#4-双臂双相机-charuco-外参标定先左臂再右臂)。 |
| 机械臂不动或录制中断 | 查看示教器安全/保护停机提示及 `ros2 topic echo /quest3/system_status`。追踪或 RTDE 恢复后，先松开再按住侧握把重新锚定。见[运行诊断](docs/QUICKSTART_COMMANDS.md#8-常用运行时诊断)。 |

完整排障命令与紧急停止步骤见[命令快查](docs/QUICKSTART_COMMANDS.md#8-常用运行时诊断)。
