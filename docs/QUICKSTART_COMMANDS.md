# 命令快查

除构建命令外，均从仓库根目录执行。首次部署见[安装说明](INSTALLATION.md)，标定细节见[配置与标定](CALIBRATION.md)。

| 设备 | 标识 |
| --- | --- |
| 左 / 右 UR7e | `192.168.124.61` / `192.168.124.60` |
| 主机有线口 | `192.168.124.59/24`，无网关 |
| front / side D455 | `260922305390` / `260922303231` |
| 左 / 右腕 D435i | `420222071502` / `405622072940` |
| Quest 3 ADB | `2G97C5ZJ2L05V1` |

## 1. 每个新终端先加载环境

```bash
conda deactivate 2>/dev/null || true
source /home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/ros_environment.sh
```

修改代码后重建并重启相关节点：

```bash
cd ~/ros2_ws
colcon --log-base log_jazzy build --symlink-install --packages-select quest3_robot_data \
  --build-base build_jazzy --install-base install_jazzy
source install_jazzy/local_setup.bash
```

## 2. 连接检查

```bash
adb devices -l                         # Quest 状态应为 device
adb reverse tcp:8765 tcp:8765
adb reverse --list
rs-enumerate-devices                   # 核对相机序列号
nc -zvw 1 192.168.124.61 30004         # 左臂 RTDE
nc -zvw 1 192.168.124.61 63352         # 左夹爪
nc -zvw 1 192.168.124.60 30004         # 右臂 RTDE
nc -zvw 1 192.168.124.60 63352         # 右夹爪
```

单独预览全部 RealSense RGB：`real_scripts/view_front_cam.sh`。指定相机可加 `--serial 260922305390`；按 `q` / `Esc` 退出。预览会独占相机，先停止其他相机程序，退出预览后再启动遥操。

## 3. 标定文件状态

```bash
rg 'calibrated|camera_serial|touch_rms_mm|final_board_motion' config/calibration/*.yaml
ros2 run quest3_robot_data preflight --config config/hardware_dual_teleop.yaml
```

左右臂到 front/side 的四份外参及 `dual_arm_camera_transforms.yaml` 均须为 `calibrated: true`；预检还会检查来源是否过期。现场配置使用示教器活动 TCP，不会再次设置 TCP 偏移。

## 4. 双臂双相机 ChArUco 外参标定（先左臂、再右臂）

停止所有占用相机的程序，激活两臂各自已标定的 TCP。相机和底座全程固定。

```bash
ros2 run quest3_robot_data calibrate_dual_camera_charuco \
  --config config/hardware_dual_teleop.yaml \
  --squares-x 11 --squares-y 8 \
  --square-size-m 0.070 --marker-size-m 0.052 \
  --dictionary DICT_5X5_100 \
  --camera-samples 30 --max-touch-rms-mm 2.5
```

1. 在左臂可达、两台相机均可见处固定板。
2. front、side 窗口分别按空格采样，按标注 ID 触碰四点并回车。不可达点输入 `id=N` 替换，四点不得重复或共线。
3. 移开机械臂，板保持不动，回车完成两台相机的板位移检查。
4. 左臂 `PASS` 后可移动板，在右臂侧重新固定并重复采样、触点和检查。
5. 四份外参及双臂换算均通过后再启动遥操。

两臂可用不同板位置和角点，无需工作空间重叠。分次标定加 `--arm left` / `--arm right`；板仍在上次位置时才可加 `--resume`。重做、角点行列和一致性检查见[标定说明](CALIBRATION.md#双臂-frontside-charuco-相机外参标定)。

## 5. 左臂 VR 遥操

```bash
real_scripts/start_left_vr_teleop.sh --confirm MOVE_LEFT_ARM --task "pick place"
```

使用 `hardware_left_teleop.yaml + single_arm_debug.yaml`，保存左腕/front RGB；加 `--record-depth` 额外保存 front 深度。夹爪速度 `20/255`。头显操作同下一节，左手柄控制左臂。

## 6. 双臂 VR 遥操与录制

**实体运动前**：清空双臂工作空间、急停可触达；两台 UR 为 Remote Control/运行状态，活动 TCP 均为对应的 Robotiq 合爪尖端 TCP。先低速空载验证方向。

```bash
real_scripts/start_dual_vr_teleop.sh --confirm MOVE_BOTH_ARMS --task "pick place"
# 需要 front/side 深度时，在上面命令末尾加 --record-depth
```

脚本自动预检、设置 USB 转发并保持 Quest 唤醒。预检应显示 `PREFLIGHT PASS` 和 `MOTION GATE: ENABLED`。使用 `hardware_dual_teleop.yaml + dual_arm.yaml`，默认录制四路 RGB 与双臂数据。

在 Quest Browser 打开 `http://localhost:8765`，点击“连接”→“进入现实透视”。左右手柄均被追踪后松开握把，再将头显稳放在能持续追踪两只手柄的位置，保持浏览器会话运行。

| 操作 / 提示 | 含义 |
| --- | --- |
| 左 / 右侧握把 | 按住移动对应臂；每次重新握住建立相对位姿锚点 |
| 左 / 右扳机 | 对应夹爪闭合，松开张开；双臂速度 `255/255`、力度 `20/255` |
| 左 Y / 右 B / 网页录制按钮 | 开始录制或停止保存 |
| 两声升调 / 两声降调 | 录制器确认开始 / 确认停止 |
| 三声低调 | 录制中断或开始请求被拒绝 |
| `Ctrl-C` | 停止所有节点，结束当前录制 |

`--task` 在本次运行中固定；新任务需退出后重新启动。手柄相对位姿按 1:1 映射，轴关系为 Tool X 同向、Y/Z 反向；头显和 front 相机朝向不决定运动方向。实际响应及运动范围受 UR 控制柜安全设置约束，详见[系统参考](SYSTEM_REFERENCE.md)。

任一手柄追踪丢失会停止两臂并结束录制。恢复后先松开两侧握把再重新按住；录制需重新按 Y/B 开始。退出 VR、网页断开也会停止录制。脚本退出时恢复接近传感器；异常退出后执行：

```bash
bash real_scripts/quest_keep_awake.sh off
```

头显使用现实透视与文字提示，不请求相机预览。front 源图 `1280×720@30`，录制中心裁剪缩放至 `640×480`；可选 front 深度同样处理。更新后退出旧会话、刷新页面，版本应为 `passthrough-v6`。

### 与 safety 同时运行：无相机双臂遥操

使用[专用避障流程](AVOIDANCE.md)。该模式由 safety 独占相机，不启动录制器；右 A 键切换避障，不能与常规遥操同时运行。

## 7. 数据目录与视频导出

| 模式 | 保存路径 |
| --- | --- |
| 双臂，`--task "pick place"` | `data/dual_vr_episodes/pick_place/episode_00000.hdf5` |
| 左臂 | `data/left_vr_episodes/episode_00000.hdf5` |

双臂每个任务独立编号；任务名转小写、空格替换为下划线。检查与导出：

```bash
ros2 run quest3_robot_data inspect_episode \
  data/dual_vr_episodes/pick_place/episode_00000.hdf5
ros2 run quest3_robot_data export_episode_videos \
  data/dual_vr_episodes/pick_place/episode_00000.hdf5
ros2 run quest3_robot_data export_task_episode_videos \
  data/dual_vr_episodes/pick_place
# 历史文件夹 episode
ros2 run quest3_robot_data export_folder_episode_videos data/left_vr_episodes/episode_00003
```

每个 episode 输出到同级 `episode_00000_videos/`：`front_cam.mp4`、`side_cam.mp4`、`wrist_left_cam.mp4`、`wrist_right_cam.mp4`；缺少的相机输出黑帧。

默认 H.264（`avc1`）、30 FPS，每个样本写一帧；采集有缺口时视频会缩短，不会自动补帧。可加 `--fps 30 --overwrite` 覆盖；批量导出会跳过完整 H.264 视频，自动重做旧 `mp4v` 或中断结果。导出不依赖 ROS 时可运行 `/usr/bin/python3 scripts/export_task_episode_videos.py <任务目录>`。

## 8. 常用运行时诊断

```bash
ros2 topic echo /quest3/system_status       # RTDE、夹爪、录制及丢帧原因
ros2 topic echo /quest3/controllers         # 手柄追踪
ros2 topic hz /left_arm/joint_states
ros2 topic hz /right_arm/joint_states
ros2 topic echo /right_arm/teleop_enabled
ros2 topic echo /right_arm/target_tcp_pose
ros2 topic echo /right_arm/gripper_command  # 按扳机 1.0，松开 0.0
```

| 问题 | 处理 |
| --- | --- |
| Python / ROS 模块导入失败 | 新终端退出 Conda，加载第 1 节环境；见[安装说明](INSTALLATION.md) |
| ADB `unauthorized` / `offline` / 超时 | 在头显接受调试授权，执行 `adb kill-server`、`adb start-server`、`adb devices -l` |
| ADB 无设备 | 确认头显开机解锁，用 `lsusb` 检查 USB 数据线和接口；`no permissions` 检查 udev 权限 |
| 现实透视未进入 | 检查 USB 转发，退出旧 WebXR 会话并刷新 Quest Browser；允许 `immersive-ar` |
| 相机打不开 | 停止其他占用相机的程序，再用 `view_front_cam.sh` 检查 |
| 机械臂不动 | 查看系统状态与示教器的安全/保护停机提示；现场配置未设置软件 Cartesian 工作空间 |
| 录制掉帧 | 查看 `drop_reasons`、队列和源话题频率；见[丢帧诊断](SYSTEM_REFERENCE.md#丢帧诊断) |

急停、保护停机或 RTDE 失联会中断当前录制。先在示教器排除故障并使能；程序每秒重试 RTDE。显示 `reconnected; release and re-press clutch` 后松开再握住重新锚定，并开始新 episode。

## 9. 紧急停止

- 松开对应侧握把：停止该臂。
- 终端 `Ctrl-C`：执行 `servoStop` 并停止节点。
- 碰撞或线缆风险：立即使用实体急停，排除原因后再恢复。
