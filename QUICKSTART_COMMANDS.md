# Quest 3 + 双 UR7e 命令快查（当前现场）

本文档只保留当前已接入、已验证的现场流程。详细实现和历史只读调试见 [`real_scripts/README.md`](real_scripts/README.md)。

| 设备 | 标识 |
|---|---|
| 左 UR7e | `192.168.124.58` |
| 右 UR7e | `192.168.124.60` |
| 主机有线口 | `192.168.124.59/24`，无网关 |
| front D455 | `260922305390` |
| 左腕 D435i | `420222071502` |
| 右腕 D435i | `405622072940` |
| Quest 3 ADB | `2G97C5ZJ2L05V1` |

> 实体运动前：清空双臂工作空间、急停可触达、两台 UR 处于 Remote Control/运行状态，并确认每台示教器中活动 TCP 均为对应的 Robotiq 2F-85 合爪尖端 TCP。

## 1. 每个新终端先加载环境

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

代码修改后重建：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select quest3_robot_data
source install/setup.bash
```

## 2. 连接检查

Quest 与 USB 转发：

```bash
adb devices
adb reverse tcp:8765 tcp:8765
adb reverse --list
```

预期 Quest 状态为 `device`。若 ADB 卡住：

```bash
adb kill-server
adb start-server
adb devices
```

查看 RealSense 序列号：

```bash
/usr/bin/python3 - <<'PY'
import pyrealsense2 as rs
for d in rs.context().query_devices():
    print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number))
PY
```

检查两台机器人网络与 Robotiq URCap socket：

```bash
nc -zvw 1 192.168.124.58 30004
nc -zvw 1 192.168.124.58 63352
nc -zvw 1 192.168.124.60 30004
nc -zvw 1 192.168.124.60 63352
```

## 3. 标定文件状态

检查左右 front 外参：

```bash
grep -E 'calibrated|touch_rms_mm|final_board_motion' \
  config/calibration/left_base_to_front_camera.yaml \
  config/calibration/right_base_to_front_camera.yaml
```

只有两份文件都显示 `calibrated: true` 才启动双臂。当前系统使用示教器的活动 TCP，不需要由脚本再次设置 TCP 偏移。

## 4. 右臂 ChArUco 外参标定（重做时使用）

先停止所有占用 front D455 的 RealSense 节点。标定板可以与左臂标定时放在不同位置，但在本次“拍相机样本→触四点”的过程中必须固定不动。

```bash
ros2 run quest3_robot_data calibrate_camera_charuco \
  --robot-ip 192.168.124.60 \
  --camera-serial 260922305390 \
  --base-frame right_base \
  --camera-frame front_camera_color_optical_frame \
  --squares-x 10 --squares-y 7 \
  --square-size-m 0.037 --marker-size-m 0.027 \
  --dictionary DICT_5X5_100 \
  --camera-samples 30 --max-touch-rms-mm 2.5 --recapture-camera \
  --output /home/mypc/Documents/pyw/projects/quest3_collect/config/calibration/right_base_to_front_camera.yaml
```

操作顺序：空格锁定相机样本 → 按命令行实际检测到的四个点逐一触碰并回车 → 移开机械臂后回车做标定板位移检查。必须得到 `calibrated: true` 与 `PASS`。

左臂重标定时仅替换：

```text
--robot-ip 192.168.124.58
--base-frame left_base
--output .../left_base_to_front_camera.yaml
```

## 5. 左臂 VR 遥操

```bash
real_scripts/start_left_vr_teleop.sh --confirm MOVE_LEFT_ARM
```

头显中打开 `http://localhost:8765`，点击“连接”→“进入 VR”。左手柄侧握把为左臂位姿离合；按住后先保持静止完成锚定，再移动/旋转手柄。左扳机独立控制左夹爪：按住缓慢合拢，松开缓慢张开。

停止：终端按 `Ctrl-C`。

## 6. 双臂 VR 遥操与录制

启动前只读预检：

```bash
ros2 run quest3_robot_data preflight \
  --config /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_dual_teleop.yaml
```

必须看到：

```text
PREFLIGHT PASS
MOTION GATE: ENABLED
```

启动：

```bash
real_scripts/start_dual_vr_teleop.sh --confirm MOVE_BOTH_ARMS
```

- 左手柄侧握把：左臂相对位姿遥操；左扳机：左夹爪。
- 右手柄侧握把：右臂相对位姿遥操；右扳机：右夹爪。
- 两臂均为 1:1 相对位姿、最大平移约 `0.25 m/s`、最大角速度约 `1.5 rad/s`。
- 两个 TCP 在 front 相机坐标中的距离小于 `0.20 m` 时，双臂同时停止；松开再按住侧握把重新锚定后恢复。

录制：左手柄 **Y 键**、右手柄 **B 键**，或网页“开始录制/停止并保存”按钮均可切换。退出 VR、网页断开或终端 `Ctrl-C` 会停止并保存当前 episode。

当前未接 side D455，双臂 episode 保存 front RGB-D、左右腕 RGB、双臂关节状态、commanded action 和夹爪状态/命令；side 相机视频会在导出时生成全黑占位。

Quest VR 中央保持 front D455 的原始主视角；左、右两侧分别显示 left/right wrist RGB（未连接的一侧为黑屏）。front 预览为 `1280×720@30` 源图、目标 `15 FPS` 传输，两个腕部预览目标各 `10 FPS`，以优先保证遥操主视角稳定。D455 深度为 `640×480@30`；录制器仍将 front RGB 与深度写入 `640×480` HDF5 数据格式，腕部数据不变。

导出视频默认每个已保存的同步样本写一帧、固定 `30 FPS`；若录制期间同步采样掉帧，视频会更短而不会插入重复帧。覆盖已有导出视频时追加 `--overwrite`。

当前实时采集配置默认使用 HDF5 后端：新 episode 为 `episode_00001.hdf5`，所有 RGB、深度、关节状态和 action 均按同一个样本索引顺序追加到单个文件中。这样避免每帧创建多个文件带来的写盘延迟。

历史文件夹 episode 导出视频：

```bash
ros2 run quest3_robot_data export_folder_episode_videos \
  data/left_vr_episodes/episode_00003
```

覆盖已有视频时追加 `--overwrite`。

## 7. 数据目录与视频导出

双臂 episode：

```text
data/dual_vr_episodes/episode_00000.hdf5
data/dual_vr_episodes/episode_00001.hdf5
...
```

左臂 episode：

```text
data/left_vr_episodes/episode_00000.hdf5
...
```

查看 HDF5 结构和时间戳质量：

```bash
ros2 run quest3_robot_data inspect_episode \
  data/dual_vr_episodes/episode_00000.hdf5
```

导出固定四路 MP4：

```bash
ros2 run quest3_robot_data export_episode_videos \
  data/dual_vr_episodes/episode_00000.hdf5
```

默认输出：

```text
data/dual_vr_episodes/episode_00000_videos/
├── front_cam.mp4
├── side_cam.mp4
├── wrist_left_cam.mp4
└── wrist_right_cam.mp4
```

覆盖旧视频并固定帧率：

```bash
ros2 run quest3_robot_data export_episode_videos \
  data/dual_vr_episodes/episode_00000.hdf5 \
  --fps 30 --overwrite
```

## 8. 常用运行时诊断

查看系统状态（重点查看 `left_arm_rtde` / `right_arm_rtde` 是否为 `enabled`，以及 `*_gripper`）：

```bash
ros2 topic echo /quest3/system_status
```

查看 Quest 手柄数据：

```bash
ros2 topic echo /quest3/controllers
```

查看右臂是否收到目标、是否使能：

```bash
ros2 topic echo /right_arm/teleop_enabled
ros2 topic echo /right_arm/target_tcp_pose
```

查看左右关节状态：

```bash
ros2 topic hz /left_arm/joint_states
ros2 topic hz /right_arm/joint_states
```

查看右夹爪命令（扳机按住为 `1.0`，松开为 `0.0`）：

```bash
ros2 topic echo /right_arm/gripper_command
```

若 Quest 页面黑屏：退出沉浸式会话、关闭旧浏览器标签，确认脚本日志中有 `front_camera.camera: RealSense Node Is Up!`，再在 Quest Browser 打开 `http://localhost:8765`。

若预检提示 `cannot check Quest ADB ... timed out`：这通常是首次启动 ADB daemon 超过旧的 3 秒窗口。当前版本已改为先启动 daemon 并等待最多 12 秒。仍失败时，退出所有头显上的 USB 调试授权提示后，在主机执行：

```bash
adb kill-server
adb start-server
adb devices -l
```

输出中 Quest 3 对应行必须是 `device`（不是 `unauthorized` 或 `no permissions`）；若是 `unauthorized`，戴上头显并在 USB 调试提示中选择允许。

当前遥操配置不设置软件 Cartesian 工作空间；实际运动边界由 UR 控制柜中的安全平面、关节限制、速度限制及急停负责。若任一臂不动，先查看系统状态和示教器的安全/保护停机提示。

发生保护停机或急停后，先在示教器解除故障并重新使能机械臂。运行中的遥操程序会每秒自动尝试重建 RTDE 连接；状态显示 `reconnected; release and re-press clutch` 后，必须先松开再重新按住对应手柄侧握把，才会建立新的相对位姿锚点并允许该臂运动。

## 9. 紧急停止

- 立刻松开对应手柄侧握把：该臂目标停止。
- 终端按 `Ctrl-C`：调用 `servoStop` 并停止所有节点。
- 任何异常、碰撞风险或线缆拉扯风险：使用 UR 示教器急停/保护停止，确认原因后再恢复。
