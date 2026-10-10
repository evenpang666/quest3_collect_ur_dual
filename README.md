# Quest 3 + 双 UR7e 实体遥操作与数据采集

本项目使用 Quest Browser 的 WebXR 获取左右手柄位姿，通过 USB `adb reverse` 连接主机；头显通过 WebXR `immersive-ar` 显示现实透视，仅叠加录制文字提示，暂不显示三路相机画面；RealSense 图像仍正常采集和保存。左手柄固定控制左 UR7e，右手柄固定控制右 UR7e。每次按下侧握把时，记录手柄 `gripSpace` 与该臂当前 Tool/TCP 的位姿；后续手柄相对位姿偏移按 X 同向、Y/Z 反向映射到 TCP，头显朝向和 front 相机朝向均不参与运动方向换算。笛卡尔目标通过 RTDE `servoL` 发送，由每台 UR 控制器内部完成 IK。

默认配置支持双臂；`hardware_single.yaml + single_arm.yaml` 支持只启动左臂。

现场部署时可直接使用 [快速启动命令快查](QUICKSTART_COMMANDS.md)，其中集中列出了安装、标定、预检、单/双臂启动、录制和诊断命令。

## 系统结构

```text
Quest 3 (USB / adb reverse)
  └─ WebXR 左右手柄 ──> webxr_bridge ──> pose_mapper
       ▲ front RGB                         ├─ 左臂绝对 TCP 目标
       │                                   └─ 右臂绝对 TCP 目标
       └──────────── D455 front       rtde_servo ──以太网──> 两台 UR7e
                                                └─TCP 63352──> 两个 2F-85

2×D435i wrist RGB + D455 front/side RGB-D + 双臂状态/action
  └──────────────────────── recorder ──> episode_XXXXX.hdf5
```

## 安全边界

配置中的 `rtde.enable_motion` 默认为 `false`。此时系统只读取机器人状态，不会移动机械臂或夹爪。完成全部标定、左右手验证和低速空载检查后，才可手动改为 `true`。

- 每只手的侧握把是对应机械臂的独立死手开关；松开立即停止并清除锚点。
- WebXR/目标数据超过 0.25 秒未更新会执行 `servoStop()`。
- 目标超出各机械臂 `workspace_min/max` 会停止。
- 单次目标位置/姿态变化会被 `max_target_jump_m/rad` 限制。
- 双臂目标 TCP 距离小于 `min_tcp_separation_m` 时两臂同时撤销使能。
- 软件限制不能替代实体急停、安全平面、减速模式、双臂防碰撞和现场监护。

TCP 距离保护不覆盖机械臂连杆和场景，**尚不能证明两臂之间或机械臂与场景之间无碰撞**。首次运动必须低速、空载，并有人手持急停；正式使用建议再接入 MoveIt Planning Scene 或独立安全 PLC。

## 1. 安装

建议 Ubuntu 22.04 + ROS 2 Humble，使用 ROS 对应的系统 Python，不要使用无法导入 `rclpy` 的 Conda Python。

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge \
  ros-humble-launch-ros python3-opencv python3-aiohttp python3-h5py
python3 -m pip install --user ur_rtde pyrealsense2
```

构建：

```bash
mkdir -p ~/ros2_ws/src
ln -s /home/mypc/Documents/pyw/projects/quest3_collect ~/ros2_ws/src/quest3_robot_data
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select quest3_robot_data
source install/setup.bash
```

## 2. 填写硬件身份

双臂修改 [config/hardware.yaml](config/hardware.yaml)，单臂修改 [config/hardware_single.yaml](config/hardware_single.yaml)：

1. 明确记录哪台实体机器人是 `left_arm`，为两台 UR 设置不同静态 IP；
2. 填入两台机器人 IP；
3. 用 `rs-enumerate-devices` 查出四台相机序列号并填入；
4. 根据现场测量缩小左右臂的 `workspace_min/max`；
5. 保持 `enable_motion: false`。

四台 Realsense 和 Quest 都走 USB，建议把相机分散到不同 USB 3 root hub。不要通过无源 Hub 同时连接四路视频设备。两台 UR 可通过千兆交换机接主机网口，确保 IP 不冲突且 RTDE 端口 30004 可达。

## 3. TCP 标定（先做）

Pivot 标定只能求 TCP 平移。TCP 朝向来自夹爪 CAD/安装方向，通过 `--tcp-rotation-rotvec` 指定；默认认为和法兰方向一致。

在 UR 示教器中先把当前 TCP 设为全零。将夹爪工具尖端固定在同一机械点，至少采集 12 个差异明显的法兰姿态：

```bash
ros2 run quest3_robot_data calibrate_tcp \
  --robot-ip 192.168.1.10 \
  --base-frame left_base --flange-frame left_tool0 --tcp-frame left_tcp \
  --samples 12 \
  --output /home/mypc/Documents/pyw/projects/quest3_collect/config/calibration/left_tcp.yaml
```

右臂重复执行并输出 `right_tcp.yaml`。只有 RMS 误差通过时文件才会标记 `calibrated: true`。随后将 `tcp_offset_xyz_rotvec` 填入对应 UR Installation 的 TCP 设置并设为活动 TCP。

## 4. 双臂 front/side ChArUco 相机外参标定

现场流程使用固定在桌面的 ChArUco 板，按**先左臂、再右臂**的顺序标定。每个臂先采集 front/side 两台相机，再用该臂的活动 TCP 触碰四个共同可见内角，分别求解两份 base-to-camera 外参。无需将标记刚性安装在 TCP 上。

```bash
ros2 run quest3_robot_data calibrate_dual_camera_charuco \
  --config /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_dual_teleop.yaml \
  --squares-x 10 --squares-y 7 \
  --square-size-m 0.037 --marker-size-m 0.027 \
  --dictionary DICT_5X5_100 \
  --camera-samples 30 --max-touch-rms-mm 2.5
```

front D455 为 `260922305390`，side D455 为 `260922303231`。板参数必须与实物一致。停止所有占用相机的节点后运行；每个相机窗口稳定时按空格，按标注的点 ID 触碰并回车，最后移开机械臂并回车检查两台相机中的板位移。

**左臂的两份外参均通过后允许移动标定板**，重新固定在右臂可达、两台相机均可见的位置。右臂独立重新采集板观测及 TCP 触点。每个臂内部，从拍摄第一台相机到最终位移检查结束，板必须固定；相机和机器人底座始终保持固定。

左右臂的四点独立选择，允许不同点 ID 和不同板位置，不要求两臂工作空间重叠。可分两次运行：先追加 `--arm left` 完成左臂，再移动板并追加 `--arm right` 完成右臂。两臂都通过后，程序才利用固定相机换算基座关系：`left_T_right = left_T_camera × inverse(right_T_camera)`。通过 front/side 两条路径及两臂相机相对位姿的一致性检查后，生成 `dual_arm_camera_transforms.yaml`，含左右基座和 front/side 相机之间的变换。默认一致性阈值为 `10 mm / 3°`，汇总未通过或来源外参改变时，预检会阻止双臂启动。

| 机器人 | 相机 | 输出文件 |
|---|---|---|
| left_arm | front | `left_base_to_front_camera.yaml` |
| left_arm | side | `left_base_to_side_camera.yaml` |
| right_arm | front | `right_base_to_front_camera.yaml` |
| right_arm | side | `right_base_to_side_camera.yaml` |

四份外参和换算汇总均须为 `calibrated: true`。front 外参用于将 WebXR 偏移映射到各 UR base；side 外参随 episode 保存，用于三维数据处理。移动相机、机械臂底座或改变 TCP 后必须重新标定。独立标定、断点续做及重做已通过结果的方法见 [快速启动第 4 节](QUICKSTART_COMMANDS.md#4-双臂双相机-charuco-外参标定先左臂再右臂)。原有单臂单相机 `calibrate_camera_charuco` 和 TCP 刚性 ArUco `calibrate_camera` 命令继续保留。

## 5. 启动四台相机

```bash
ros2 launch quest3_robot_data realsense_four.launch.py \
  hardware_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/hardware.yaml
```

腕部 D435i 只启用 RGB；front/side D455 保存 RGB 和对齐到彩色图的深度。默认 640×480、30 Hz。检查：

```bash
ros2 topic hz /front_camera/camera/color/image_raw
ros2 topic hz /front_camera/camera/aligned_depth_to_color/image_raw
```

若实际 realsense-ros 版本生成的话题不同，修改 `dual_arm.yaml`/`single_arm.yaml` 和 `hardware*.yaml` 中的话题。

## 6. 只读预检和左右手验证

Quest 开启 Developer Mode，通过 USB 接入并在头显中接受 RSA 调试授权：

```bash
adb devices
adb reverse tcp:8765 tcp:8765
ros2 run quest3_robot_data preflight \
  --config /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware.yaml
```

预检会只读检查：左右角色、每个已配置场景相机对应的外参、TCP 来源、四个序列号、两台机器人 RTDE 连接和 Quest ADB。使用 `robot_active_tcp` 的现场配置无需额外 TCP YAML。全部通过后，以 `enable_motion:false` 启动完整系统：

```bash
ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/hardware.yaml \
  recorder_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/dual_arm.yaml \
  output_dir:=/data/quest3_ur7e
```

在 Quest Browser 打开 `http://localhost:8765`，点击“连接”→“进入现实透视”。头显内应能看到真实场景，上方显示录制状态与操作提示（页面版本 `passthrough-v2`）。需使用支持 `immersive-ar` 的 Quest Browser；若不支持，页面会提示错误。确认：

- 左手柄状态只对应 `left_arm`，右手柄只对应 `right_arm`；
- 按住握把时建立锚点，第一帧不会跳动；
- 松开握把或拔掉 USB 后状态立即变成 stopped；
- 现实透视可见，Y/B 能切换录制，提示面板计时与中断原因正常显示；
- 在小幅移动手柄时，实际机械臂方向与预期一致。

手柄 `gripSpace` 的 X 对应 Tool +X，Y 对应 Tool -Y，Z 对应 Tool -Z；因此手柄前方（-Z）对应 Tool +Z。如果现场方向仍不符，检查示教器中当前活动 TCP 的坐标轴。front 外参只用于双臂 TCP 距离换算，不决定遥操方向。

## 7. 低速启用实体运动

确认两台 UR 处于允许 RTDE 控制的 Remote/运行状态，急停可用。停止系统，将 `hardware.yaml` 的 `rtde.enable_motion` 改成 `true`，重新预检并启动。

先只启动单臂配置验证，再启用双臂。按住左/右握把才会移动对应机械臂；扳机闭合对应 2F-85。右手 B 键开始/结束一个 episode，右/左摇杆分别重新锚定对应机械臂。

正式连续采集建议给 RTDE 进程配置实时调度/实时内核，并让数采目录使用 NVMe。四路图像写盘与 RTDE 控制已经分在不同进程，但普通内核在高 I/O 压力下仍可能产生控制周期抖动。

## 8. 数据格式与同步

以每个新的 front RGB 时间戳生成一帧：其他相机选择最近帧，关节状态插值到 front 时间。任一数据源超过 `max_skew_sec` 则丢弃整帧，不会写入错配数据。

```text
observations/images/{left_wrist,right_wrist,front,side}/rgb
observations/images/{front,side}/depth
observations/qpos                 # 6/12维，rad
observations/qvel                 # 6/12维，rad/s
observations/gripper_position     # 1/2维，0开到1闭
action                            # UR内部目标q - 当前q，6/12维
executed_action                   # q[t+1] - q[t]
gripper_action                    # 1/2维，0开到1闭
action_valid                      # 最后一帧为false
is_terminal
timestamps_ns
source_timestamps_ns/*
```

HDF5 会嵌入 hardware 配置、front/side 外参和 TCP 标定。临时写入 `episode_XXXXX.inprogress`，成功停止后原子改名；已有临时文件不会被覆盖。

RGB 默认以 `raw_lzf` 无损保存。若 NVMe 写入仍跟不上，可在 recorder YAML 中改成 `rgb_storage: jpeg`；此时每帧保存为 HDF5 可变长 JPEG 字节，需要训练加载器先解码，深度图仍保持无损原始值。

检查数据：

```bash
ros2 run quest3_robot_data inspect_episode /data/quest3_ur7e/episode_00000.hdf5
```

## 单臂模式

完成左臂标定后，使用：

```bash
ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_single.yaml \
  recorder_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/single_arm.yaml \
  output_dir:=/data/quest3_ur7e_single
```

单臂模式只订阅左腕相机和左 UR/Robotiq，但仍保存 front/side RGB-D。
