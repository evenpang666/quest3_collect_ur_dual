# 系统、安全与数据参考

以 [README](../README.md) 为操作入口；当前现场参数及脚本命令见 [命令快查](QUICKSTART_COMMANDS.md)。

本项目使用 Quest Browser 的 WebXR 获取左右手柄位姿，通过 USB `adb reverse` 连接主机；头显通过 WebXR `immersive-ar` 显示现实透视，仅叠加录制文字提示，暂不显示三路相机画面；RealSense 图像仍正常采集和保存。左手柄固定控制左 UR7e，右手柄固定控制右 UR7e。每次按下侧握把时，记录手柄 `gripSpace` 与该臂当前 Tool/TCP 的位姿；后续手柄相对位姿偏移按 X 同向、Y/Z 反向映射到 TCP，头显朝向和 front 相机朝向均不参与运动方向换算。笛卡尔目标通过 RTDE `servoL` 发送，由每台 UR 控制器内部完成 IK。

当前实体双臂脚本使用 `hardware_dual_teleop.yaml + dual_arm.yaml`；`hardware.yaml` 为只读配置。单臂脚本使用 `hardware_left_teleop.yaml + single_arm_debug.yaml`。
与独立运行的 `../safety` 检测服务共用相机时，可运行
[`start_dual_vr_teleop_no_cameras.sh`](../real_scripts/start_dual_vr_teleop_no_cameras.sh)：
它不启动相机或录制器，右手柄 A 键可开启 QP-CBF 避障。避障模式先检查下一条
RTDE 单步指令的固定点与扫掠路径；安全时直接使用 `servoJ` 发送，只有碰撞风险时
才对本步指令做 QP-CBF 优化，并只处理最新手柄目标；普通启动脚本
仍使用上面的 `servoL` 路径，不启用 A 键避障。

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

只读配置 `hardware.yaml` 的 `rtde.enable_motion` 为 `false`；实体遥操作脚本使用的 `hardware_dual_teleop.yaml` 为 `true`。运行实体脚本前应完成标定、预检、左右手验证和低速空载检查。

- 每只手的侧握把是对应机械臂的独立死手开关；松开立即停止并清除锚点。
- WebXR/目标数据超过 0.25 秒未更新会执行 `servoStop()`。
- 自定义配置若同时设置 `workspace_min/max`，超出范围的目标会停止；当前实体双臂配置未设置软件 Cartesian 工作空间，由 UR 控制柜安全设置负责运动边界。
- 单次目标位置/姿态变化会被 `max_target_jump_m/rad` 限制。
- 双臂目标 TCP 距离小于 `min_tcp_separation_m` 时两臂同时撤销使能。
- 软件限制不能替代实体急停、安全平面、减速模式、双臂防碰撞和现场监护。

TCP 距离保护不覆盖机械臂连杆和场景，**尚不能证明两臂之间或机械臂与场景之间无碰撞**。首次运动必须低速、空载，并有人手持急停；正式使用建议再接入 MoveIt Planning Scene 或独立安全 PLC。

## 启动四台相机

```bash
ros2 launch quest3_robot_data realsense_four.launch.py \
  hardware_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/hardware.yaml
```

腕部 D435i 只启用 RGB；front/side D455 保存 RGB 和对齐到彩色图的深度。实际分辨率和帧率以相机配置及启动日志为准。检查：

```bash
ros2 topic hz /front_camera/camera/color/image_raw
ros2 topic hz /front_camera/camera/aligned_depth_to_color/image_raw
```

若实际 realsense-ros 版本生成的话题不同，修改 `dual_arm.yaml`/`single_arm.yaml` 和 `hardware*.yaml` 中的话题。

## 只读预检和左右手验证

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

在 Quest Browser 打开 `http://localhost:8765`，点击“连接”→“进入现实透视”。头显内应能看到真实场景，上方显示录制状态与操作提示（页面版本以实际页面为准）。需使用支持 `immersive-ar` 的 Quest Browser；若不支持，页面会提示错误。确认：

- 左手柄状态只对应 `left_arm`，右手柄只对应 `right_arm`；
- 按住握把时建立锚点，第一帧不会跳动；
- 松开握把或拔掉 USB 后状态立即变成 stopped；
- 现实透视可见，Y/B 能切换录制，提示面板计时与中断原因正常显示；
- 在小幅移动手柄时，目标话题的变化方向与预期一致；实体方向在后续低速空载验证。

手柄 `gripSpace` 的 X 对应 Tool +X，Y 对应 Tool -Y，Z 对应 Tool -Z；因此手柄前方（-Z）对应 Tool +Z。如果现场方向仍不符，检查示教器中当前活动 TCP 的坐标轴。front 外参只用于双臂 TCP 距离换算，不决定遥操方向。

## 低速启用实体运动

确认两台 UR 处于允许 RTDE 控制的 Remote/运行状态，急停可用。先用只读配置验证；实体运动使用 `hardware_dual_teleop.yaml` 与[现场启动脚本](QUICKSTART_COMMANDS.md#6-双臂-vr-遥操与录制)，重新预检后启动。

先只启动单臂配置验证，再启用双臂。按住左/右握把才会移动对应机械臂；扳机闭合对应 2F-85。右手 B 键开始/结束一个 episode，右/左摇杆分别重新锚定对应机械臂。

正式连续采集建议给 RTDE 进程配置实时调度/实时内核，并让数采目录使用 NVMe。四路图像写盘与 RTDE 控制已经分在不同进程，但普通内核在高 I/O 压力下仍可能产生控制周期抖动。

## 数据格式与同步

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

下列配置的 `enable_motion: false`，用于左臂只读验证：

```bash
ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_single.yaml \
  recorder_config:=/home/mypc/Documents/pyw/projects/quest3_collect/config/single_arm.yaml \
  output_dir:=/data/quest3_ur7e_single
```

当前现场需要左臂实体遥操作及录制时，按[命令快查的左臂流程](QUICKSTART_COMMANDS.md#5-左臂-vr-遥操)使用 `start_left_vr_teleop.sh`；该脚本使用 `hardware_left_teleop.yaml` 和 `single_arm_debug.yaml`，采集左腕 RGB 与 front RGB-D。

