# 左臂遥操作现场调试（无标定、只读）

本文档对应 2026-09-08 已接入的最小硬件：左 UR7e、左腕相机、front 相机和 Quest 3。

## 当前已确认参数

| 设备 | 参数 | 状态 |
|---|---|---|
| UR7e | `192.168.124.61/24` | ping、30004、63352 已通过 |
| 主机有线口 | `enp4s0`，`192.168.124.59/24` | 永久静态、无网关 |
| Quest 3 | `2G97C5ZJ2L05V1` | ADB authorized |
| front | D455，`260922305390` | 更换接线后重新识别 |
| left_wrist | D435i，`420222071502` | 更换腕部相机后重新识别 |

当前在线设备为一台 D435i 和一台 D455；旧腕部序列号 `405622072940` 已不在线。

使用的配置：

- `config/hardware_left_debug.yaml`
- `config/single_arm_debug.yaml`

调试配置允许暂时缺少 front/side/TCP 标定，但只在 `enable_motion: false` 时有效。
如果把它改成 `true`，节点会拒绝启动。此配置只能调通相机、Quest、手柄和机器人状态，不能用于实体运动。

## 0. 一次性补齐 Python 依赖

不要在 Conda 环境中安装 ROS 依赖：

```bash
conda deactivate
conda deactivate  # 若提示符仍显示 (base)，再执行一次

sudo apt install -y python3-pip
/usr/bin/python3 -m pip install --user ur-rtde
```

预检会在缺少 `pyrealsense2` 时回退使用 `rs-enumerate-devices`；但独立 front 相机预览工具需要 `pyrealsense2`。

验证：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -c 'import rtde_receive, rtde_control; print("ur_rtde OK")'
```

## 1. 每个终端加载环境

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 2. 只读预检

```bash
ros2 run quest3_robot_data preflight \
  --config /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_left_debug.yaml
```

必须看到：

```text
PREFLIGHT PASS
MOTION GATE: receive-only; robot cannot move
```

## 3. 终端 A：启动两台相机

```bash
ros2 launch quest3_robot_data realsense_four.launch.py \
  hardware_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_left_debug.yaml
```

另开终端检查：

```bash
ros2 topic hz /left_wrist_camera/camera/color/image_raw
ros2 topic hz /front_camera/camera/color/image_raw
ros2 topic hz /front_camera/camera/aligned_depth_to_color/image_raw
```

## 4. 终端 B：启动 Quest 主视角与手柄桥接

```bash
ros2 run quest3_robot_data webxr_bridge \
  --front-topic /front_camera/camera/color/image_raw
```

另开终端配置 USB 转发：

```bash
adb reverse tcp:8765 tcp:8765
adb reverse --list
```

Quest Browser 打开 `http://localhost:8765`，依次点击“连接”和“进入 VR”。此时应在头显里看到 front 画面。

如果刚更新过网页代码，请先退出沉浸式 VR、关闭该浏览器标签页，再重新打开网址进入 VR。
当前页面应显示版本 `orientation-v2`；若没有该文字，说明仍在使用旧页面。

验证左手柄（先不要按侧握把）：

```bash
ros2 topic echo --once /quest3/controllers
```

消息中应同时包含 `"left"` 和左手柄 `position/orientation`。

## 5. 终端 C：只读启动 UR 状态节点

确保示教器已开机、机器人处于可通信状态，但无需启用运动：

```bash
ros2 run quest3_robot_data rtde_servo --config \
  /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_left_debug.yaml
```

检查状态：

```bash
ros2 topic hz /left_arm/joint_states
ros2 topic echo --once /left_arm/tcp_pose
ros2 topic echo /quest3/system_status
```

期望状态包含：

```text
left_arm_rtde: receive-only (enable_motion=false)
```

## 6. 终端 D：只读验证手柄映射

```bash
ros2 run quest3_robot_data pose_mapper --config \
  /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_left_debug.yaml
```

按住左侧握把并小幅移动手柄，然后观察诊断目标：

```bash
ros2 topic echo /left_arm/teleop_enabled
ros2 topic echo /left_arm/target_tcp_pose
```

此时目标话题会变化，但 `enable_motion:false` 保证目标不会发送到机械臂控制接口。

## 7. RTDE 关节微动测试（不依赖相机/TCP 标定）

先运行 dry-run；默认只计算 `wrist_3` 的 `+0.5°` 目标，不发送命令：

```bash
ros2 run quest3_robot_data joint_motion_probe --ip 192.168.124.61
```

确认示教器为自动模式、机器人周围无人无障碍、急停可触达，并确认腕部线缆允许微量转动后，才执行：

```bash
ros2 run quest3_robot_data joint_motion_probe --ip 192.168.124.61 \
  --execute --confirm ROBOT_AREA_CLEAR
```

脚本以不超过 `0.05 rad/s` 的速度让第 6 轴移动 `+0.5°`，保持 0.5 秒，再回到启动位置。参数有硬上限：位移不超过 `1°`、速度不超过 `0.10 rad/s`、加速度不超过 `0.20 rad/s²`。这只验证 RTDE 运动通道，不代表 VR 坐标映射已经安全。

## 8. 桌面固定 ChArUco 的 front_cam 外参标定

此流程适用于“标定板固定在桌面上”，与把单个 ArUco 刚性安装到末端的旧流程不同。必须先完成 TCP 标定，并在示教器中录入、激活该 TCP。

当前标定板在板坐标中为横向 10 列 × 7 行（方格 37 mm、marker 27 mm）。按空格锁定画面时，程序从该帧实际检测成功的内角中自动选择总互距最大的四点，并在命令行输出 ID、板网格行列、图像像素坐标和画面方位。运行前暂停 RealSense ROS 节点，使程序能以 1280×720 独占 front_cam：

```bash
ros2 run quest3_robot_data calibrate_camera_charuco \
  --robot-ip 192.168.124.61 \
  --camera-serial 260922305390 \
  --base-frame left_base \
  --camera-frame front_camera_color_optical_frame \
  --squares-x 10 --squares-y 7 \
  --square-size-m 0.037 --marker-size-m 0.027 \
  --dictionary DICT_5X5_100 \
  --recapture-camera \
  --output /home/mypc/Documents/pyw/projects/quest3_collect/config/calibration/left_base_to_front_camera.yaml
```

上面的字典名必须与印板时使用的完整 OpenCV 字典一致；可选值为 `DICT_5X5_50/100/250/1000`。图像检测稳定后只按一次空格，程序自动采集一组帧并立即保存相机观测检查点；随后依次触碰四个角点，每个 TCP 坐标也会立即保存。中断后重新运行会自动续采。若相机或标定板移动过，增加 `--recapture-camera`，并删除对应的 `_touches.yaml` 后重采。最终输出只有通过触点 RMS 和相机观测稳定性门限时才标记为 `calibrated:true`。

若质量报告指出个别触点误差较大，可在原命令后增加 `--retouch-corner-ids 8 31`。程序会复用相机观测和其他触点，只要求重新触碰指定点。

## 暂不执行

在 front 外参和 TCP 标定完成前：

- 不得把 `enable_motion` 改为 `true`；
- 不得使用调试配置进行实体移动；
- 不得根据当前单位变换判断真实机械臂运动方向。

下一阶段应完成 TCP、front 外参和工作空间边界确认，再创建低速运动配置。

## 9. 已标定左臂的 VR 实体遥操（当前现场）

已通过 front 外参、并已在 UR 示教器激活“Robotiq 合爪尖端”TCP 后，使用
`config/hardware_left_teleop.yaml`。该配置只连接左腕 D435i 与 front D455，使用
**左手柄→左臂**，平移/旋转均为 `1.0×` 手柄相对位姿，`0.45 m/s` 限速。RTDE 不会重新设置 TCP，
因此必须确认示教器当前活动 TCP 就是已标定的 Robotiq 合爪尖端。

启动脚本会先启动两台 RealSense 并等待 front D455 出现图像；若 25 秒内没有图像，
它会停止相机节点且不会启动 RTDE 控制。单臂调试录制结果只包含 left_wrist RGB 与
front RGB-D；接入 side D455 后再使用双相机数据采集配置。

先确认：现场清空、急停可触达、UR 为 Remote Control/运行、末端没有夹持会因误操作掉落的物体。
随后执行：

```bash
conda deactivate 2>/dev/null || true
/home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/start_left_vr_teleop.sh \
  --confirm MOVE_LEFT_ARM \
  --task "将红色积木放入蓝色容器"
```

Quest Browser 打开 `http://localhost:8765`，点击“连接”“进入 VR”。只有按住**左手柄侧握把**时才会给左臂发目标；先按住不移动，确认机械臂不动后，再以毫米级缓慢移动。运行中按 `Ctrl-C` 会调用 `servoStop` 并停止全部节点。

`--task` 必填，且在整个脚本运行期间不可变。每个保存的 HDF5 根属性 `task_description` 都会记录该文本；要录制新任务，退出脚本后以新的 `--task` 参数启动。

夹爪与机械臂离合独立：左手柄扳机按下（阈值 `0.15`）时 2F-85 以低速合拢，松开时以低速张开。当前 Robotiq 寄存器设置为速度 `20/255`、力度 `25/255`。不握侧握把也可测试夹爪，机械臂不会移动；Quest 或手柄追踪丢失时保持夹爪最后位置。

## 10. 双臂双相机标定与双臂遥操

现场 front D455 为 `260922305390`，side D455 为 `260922303231`。两臂均使用各自示教器已标定并激活的“2F-85 合爪尖端”TCP。

按 [快速启动第 4 节](../QUICKSTART_COMMANDS.md#4-双臂双相机-charuco-外参标定先左臂再右臂) 执行 `calibrate_dual_camera_charuco`：先左臂 front/side 观测、触四个共同内角、双相机位移检查；左臂通过后可移动板，再重新采集并标定右臂。每个臂内部必须保持板固定，两台相机和机器人底座始终保持固定。只有四份外参都为 `calibrated: true` 才启动双臂。

```bash
/home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/start_dual_vr_teleop.sh \
  --confirm MOVE_BOTH_ARMS \
  --task "双臂协作将物体放入指定区域"
```

左右臂各自使用可达的四个点，点 ID 和板位置可完全不同；可以分别用 `--arm left` / `--arm right` 运行。两臂都完成后才通过固定相机换算，生成 `dual_arm_camera_transforms.yaml`，一致性检查通过后才允许双臂启动。

双臂启动使用 `hardware_dual_teleop.yaml + dual_arm.yaml`，录制双腕 RGB、front/side RGB-D 及双臂状态/action，保存四份外参与换算汇总元数据。

双臂启动脚本会用 ADB 模拟接近传感器闭合，保持 Quest 在摘下后仍运行；退出脚本时恢复物理接近传感器。先戴头显启动 Quest Browser 的 WebXR 现实透视会话，再把头显稳放在摄像头可持续追踪左右手柄、扬声器能听到的位置。左手 Y / 右手 B 切换录制；两声升调表示录制器确认开始，两声降调表示确认停止，三声低调表示录制中断或请求被拒绝。异常退出后执行 `bash real_scripts/quest_keep_awake.sh off` 恢复接近传感器。

双臂 episode 保存到 `data/dual_vr_episodes/task_name/episode_00000.hdf5`。`task_name` 由 `--task` 转成小写、将空格替换为下划线；例如 `Pick Up Cube` 对应 `pick_up_cube`。每个任务目录独立编号。

左右手柄分别控制左右 UR：`1:1` 相对位姿、`0.45 m/s`、约 `3.0 rad/s`、双臂夹爪速度 `255/255`、力度 `20/255`。握住侧握把后，各手柄 `gripSpace` 相对偏移按 X 同向、Y/Z 反向映射到各自当时的 Tool/TCP 坐标系；头显和 front 相机朝向不参与运动方向换算。双臂目标在 front 相机坐标中小于 `0.20 m` 时会同时停止；分别松开并重新按住侧握把后才会恢复。单臂脚本仍使用 `hardware_left_teleop.yaml` 中的夹爪速度 `20/255`。

双臂模式要求左右手柄同时被 WebXR 追踪。缺少任一手柄时，两臂都不能运动，也不能开始录制；录制中追踪丢失会停止并保存已有有效帧。手柄恢复后须先松开两侧握把，再重新按住。

启动前预检会读取两臂当前 TCP，确认其位于各自的虚拟工作空间内；若显示
当前 `hardware_left_teleop.yaml` 与 `hardware_dual_teleop.yaml` 不设置软件 Cartesian
工作空间；运动范围由 UR 控制柜的安全平面、关节限制、速度限制及急停负责。若自定义配置
设置 `workspace_min/workspace_max`，两项必须同时设置，预检仍会检查当前 TCP 是否在范围内。

## 11. 将一个 episode 导出为四个相机视频

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run quest3_robot_data export_episode_videos \
  /home/mypc/Documents/pyw/projects/quest3_collect/data/dual_vr_episodes/task_name/episode_00000.hdf5
```

默认输出目录是 HDF5 同级的 `episode_00000_videos/`，其中固定包含
`front_cam.mp4`、`side_cam.mp4`、`wrist_left_cam.mp4`、`wrist_right_cam.mp4`。
导出历史 episode 时，如果某个相机没有 RGB 数据集，仍会生成与 episode 等长的纯黑
MP4。根据 `timestamps_ns` 自动估算帧率；需要强制帧率或覆盖旧文件时，例如：

```bash
ros2 run quest3_robot_data export_episode_videos \
  /home/mypc/Documents/pyw/projects/quest3_collect/data/dual_vr_episodes/task_name/episode_00000.hdf5 \
  --fps 30 --overwrite
```
