# Quest 3 + 双 UR7e 命令快查（当前现场）

本文档记录当前现场配置与操作流程；双臂启动前须完成四份相机外参标定。详细实现和历史只读调试见 [`real_scripts/README.md`](real_scripts/README.md)。

| 设备 | 标识 |
|---|---|
| 左 UR7e | `192.168.124.61` |
| 右 UR7e | `192.168.124.60` |
| 主机有线口 | `192.168.124.59/24`，无网关 |
| front D455 | `260922305390` |
| side D455 | `260922303231` |
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

单独预览当前接入主机的全部 RealSense RGB 图像（不会启动 ROS 节点或机械臂）：

```bash
real_scripts/view_front_cam.sh
```

默认枚举并打开所有当前连接的 RealSense（包括 front/side D455 与腕部 D435i），以网格显示 `640×480@30` RGB；在图像窗口按 `q` 或 `Esc` 退出。该工具会直接占用相机，因此运行前必须停止 VR 遥操脚本、`realsense_four.launch.py` 或其他正在使用这些相机的程序。若只需检查某一台，可追加 `--serial 260922305390`。

检查两台机器人网络与 Robotiq URCap socket：

```bash
nc -zvw 1 192.168.124.61 30004
nc -zvw 1 192.168.124.61 63352
nc -zvw 1 192.168.124.60 30004
nc -zvw 1 192.168.124.60 63352
```

## 3. 标定文件状态

检查左右臂到 front/side 的四份外参：

```bash
grep -E 'calibrated|camera_serial|touch_rms_mm|final_board_motion' \
  config/calibration/left_base_to_front_camera.yaml \
  config/calibration/left_base_to_side_camera.yaml \
  config/calibration/right_base_to_front_camera.yaml \
  config/calibration/right_base_to_side_camera.yaml \
  config/calibration/dual_arm_camera_transforms.yaml
```

四份外参和一份换算汇总都必须显示 `calibrated: true`。当前遥操配置使用示教器的活动 TCP，不需要脚本再次设置 TCP 偏移。

## 4. 双臂双相机 ChArUco 外参标定（先左臂、再右臂）

先停止 VR 遥操、相机预览及所有占用 front/side D455 的 RealSense 节点。确认左右示教器均已激活各自标定过的 TCP。两台相机和机械臂底座在整个流程中保持固定。

```bash
ros2 run quest3_robot_data calibrate_dual_camera_charuco \
  --config /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_dual_teleop.yaml \
  --squares-x 11 --squares-y 8 \
  --square-size-m 0.070 --marker-size-m 0.052 \
  --dictionary DICT_5X5_100 \
  --camera-samples 30 --max-touch-rms-mm 2.5
```

相机序列号从配置读取：front 为 `260922305390`，side 为 `260922303231`。板参数为 11×8、70 mm 格子、52 mm 标记；必须与实物一致。

标定会枚举每台相机实际支持的 RGB 格式，优先 `1280×720@30`，其次 `1280×720@15`，必要时使用 `640×480`。当前 side 通过 USB 2.1 连接，不支持 `1280×720@30`，会使用 `1280×720@15`；以启动时打印的 `RGB ready` 为准。采样和最终位移检查均读取所用格式的实际内参，无需修改标定命令。

1. 将板放在左臂可达、两台相机均可见的位置，固定后回车。
2. front 窗口稳定后按一次空格采集，再在 side 窗口按一次空格采集。两次观测之间保持板固定。
3. 程序从两台相机都识别到的内角中选择四个分散的初始触点，并保存两张标注图。触碰提示直接给出从板坐标原点起的第几行、第几列及 corner ID；行列从 1 开始，仅数棋盘格内部交点，不数板外边缘。当前 11×8 方格板有 10×7 个内角（ID 0–69），`corner ID 19` 为第 2 行、第 10 列，可对照图中的 `P19`。左臂 TCP 依次触碰指定角点，每点回车记录。不可达时可输入 `id=N`，替换点可以是同一块固定板上任意有效内角，但四点不能重复或共线。替换提示会给出行列；若未在两台锁定图中共同识别，需按行列确认实物位置，标注图仍显示初始推荐点。无效输入会重新提示，保留已记录触点。
4. 移开左臂，保持板固定，回车检查两台相机中的标定板位移。只有两份结果均通过，才显示左臂 `PASS`。
5. **左臂通过后允许移动标定板**，将其移至右臂可达、两台相机均可见的位置并重新固定。右臂重新拍摄两台相机、触四点并检查位移；不会复用左臂的板位姿或触点数据。

每个臂内部，从第一台相机拍摄到两台相机的最终位移检查结束，标定板必须固定。两台相机共同可见的内角不足四个或共线时，调整板的位置后重新开始该臂。

**左右臂各自选择四点，点 ID 可以完全不同，不要求两臂工作空间重叠。** 上面的“共同触点”只指本臂当前板位置下两台相机都看见的点。两臂也可以分两次运行：先在上面的命令追加 `--arm left`，完成并移动板后，再用 `--arm right` 运行；每次可各自指定 `--touch-corner-ids`。

两臂都完成后，程序才通过固定相机换算，生成 `config/calibration/dual_arm_camera_transforms.yaml`，包含左右基座之间的变换、front 到 side 的变换及两条路径的一致性误差。例如 `left_T_right = left_T_front × inverse(right_T_front)`，将右基座坐标转换到左基座；板位置和触点无需相同。front 和 side 两条换算路径以及两臂算出的相机相对位姿须一致，默认允许误差为 `10 mm / 3°`，可通过 `--max-cross-translation-mm` / `--max-cross-rotation-deg` 调整。

若最终一致性检查失败，四份独立结果保留，汇总标记 `calibrated: false` 并阻止双臂启动。检查相机/底座是否移动及触点是否正确，再备份并重做受影响臂的两份外参。任一外参文件改变后，预检会提示汇总过期；重新运行上面的命令即可保留已通过的两臂结果并重新换算。

新会话默认重新采集相机和触点；已经通过且设备/坐标系匹配的完整相机对会保留并跳过。单独开始右臂可在上面的命令追加 `--arm right`（左臂为 `--arm left`）。仅在板仍固定于上次位置时，追加 `--arm right --resume` 可续做右臂检查点；板已移动时不要使用 `--resume`。

要重做已通过的标定，先把对应臂的 front/side 两份最终 YAML 移至备份目录，再执行新会话。已有的通过结果不会被直接覆盖；本次任一相机未通过质量门时，该臂两份结果都标记为 `calibrated: false`，可重试。

## 5. 左臂 VR 遥操

```bash
real_scripts/start_left_vr_teleop.sh --confirm MOVE_LEFT_ARM \
  --task "将红色积木放入蓝色容器"
```

头显中打开 `http://localhost:8765`，点击“连接”→“进入现实透视”。左手柄侧握把为左臂位姿离合；按住后先保持静止完成锚定，再移动/旋转手柄。左扳机独立控制左夹爪：按住缓慢合拢，松开缓慢张开。

左手柄位移/旋转相对握住时的 `gripSpace` 锚点计算，再按 X 同向、Y/Z 反向映射到左臂当时的 Tool/TCP 坐标系；头显与 front 相机朝向不参与运动方向换算。

`--task` 为本次运行固定的自然语言任务描述。每个录制出的 HDF5 都会在根属性 `task_description` 中保存此文本；若任务改变，退出当前脚本后带新文本重新启动。

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
real_scripts/start_dual_vr_teleop.sh --confirm MOVE_BOTH_ARMS \
  --task "双臂协作将物体放入指定区域"
```

- 左手柄侧握把：左臂相对位姿遥操；左扳机：左夹爪。
- 右手柄侧握把：右臂相对位姿遥操；右扳机：右夹爪。
- 两臂均为 1:1 相对位姿、最大平移约 `0.45 m/s`、最大角速度约 `3.0 rad/s`。
- 双臂夹爪速度均为 `255/255`，力度为 `20/255`；单臂启动脚本仍使用速度 `20/255`。
- 两个 TCP 在 front 相机坐标中的距离小于 `0.20 m` 时，双臂同时停止；松开再按住侧握把重新锚定后恢复。

每次握住侧握把时，记录各手柄 `gripSpace` 与对应机械臂当时的 Tool/TCP 位姿；之后的位移和旋转按 X 同向、Y/Z 反向的轴关系换算。头显和 front 相机朝向不决定运动方向。首次实体运动前，低速空载分别核对两臂 Tool 的 X/Y/Z 正负方向；“手柄前方”（-Z）对应 Tool 的正 Z 轴，实际桌面方向取决于活动 TCP 的姿态。

进入现实透视后，必须等左右手柄都被追踪，双臂遥操和录制才会就绪。任一手柄失联或追踪超时，两臂立即撤销运动使能、当前录制停止；恢复追踪后先松开两侧握把，再重新按住锚定。网页与头显会提示手柄未就绪，录制器会拒绝缺手柄的开始请求。

录制：左手柄 **Y 键**、右手柄 **B 键**，或网页“开始录制/停止并保存”按钮均可切换。退出 VR、网页断开或终端 `Ctrl-C` 会停止并保存当前 episode。

双臂启动脚本使用 `hardware_dual_teleop.yaml + dual_arm.yaml`，episode 保存 front/side RGB-D、左右腕 RGB、双臂关节状态、commanded action 和夹爪状态/命令，并嵌入四份外参与双臂换算汇总。`dual_arm_front_only.yaml` 仅保留为无 side 相机时的临时配置。

双臂 episode 按 `--task` 分目录保存：任务名称转为小写，空格替换为下划线；例如 `--task "Pick Up Cube"` 保存到 `data/dual_vr_episodes/pick_up_cube/`。每个任务目录独立编号。

Quest 头显使用 WebXR `immersive-ar` 现实透视：背景透明，可看到真实工作场景；视野上方保留随头部移动的录制状态、计时、Y/B 操作提示和录制中断原因。front / left wrist / right wrist 三路相机画面暂不显示，头显页面不再请求预览 JPEG。相机采集与 episode 保存不受影响：front RGB 源图为 `1280×720@30`，深度为 `640×480@30`；录制器仍将 front RGB 与深度写入 `640×480` HDF5 数据格式，腕部数据不变。

需在 Quest Browser 中通过 `http://localhost:8765`（ADB 转发）或 HTTPS 打开，并允许进入沉浸式会话。若浏览器不支持现实透视，页面会显示错误提示。更新代码后按第 1 节重建并重启遥操脚本，退出旧会话、刷新头显页面；页面版本应为 `passthrough-v2`。

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
data/dual_vr_episodes/task_name/episode_00000.hdf5
data/dual_vr_episodes/task_name/episode_00001.hdf5
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
  data/dual_vr_episodes/task_name/episode_00000.hdf5
```

导出固定四路 MP4：

```bash
ros2 run quest3_robot_data export_episode_videos \
  data/dual_vr_episodes/task_name/episode_00000.hdf5
```

默认输出：

```text
data/dual_vr_episodes/task_name/episode_00000_videos/
├── front_cam.mp4
├── side_cam.mp4
├── wrist_left_cam.mp4
└── wrist_right_cam.mp4
```

覆盖旧视频并固定帧率：

```bash
ros2 run quest3_robot_data export_episode_videos \
  data/dual_vr_episodes/task_name/episode_00000.hdf5 \
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

若 Quest 现实透视未显示：退出沉浸式会话、刷新头显页面，确认页面版本为 `passthrough-v2`，在 Quest Browser 中重新点击“连接”→“进入现实透视”并允许进入会话；检查页面错误提示和 Quest Browser / 系统版本。现实背景由 Quest 自身透视提供，不依赖 RealSense 预览。

若预检提示 `cannot check Quest ADB ... timed out`：这通常是首次启动 ADB daemon 超过旧的 3 秒窗口。当前版本已改为先启动 daemon 并等待最多 12 秒。仍失败时，退出所有头显上的 USB 调试授权提示后，在主机执行：

```bash
adb kill-server
adb start-server
adb devices -l
```

输出中 Quest 3 对应行必须是 `device`（不是 `unauthorized` 或 `no permissions`）；若是 `unauthorized`，戴上头显并在 USB 调试提示中选择允许。

当前遥操配置不设置软件 Cartesian 工作空间；实际运动边界由 UR 控制柜中的安全平面、关节限制、速度限制及急停负责。若任一臂不动，先查看系统状态和示教器的安全/保护停机提示。

发生保护停机或急停后，若当前正在录制，系统会立即停止并保存该 episode；头显录制提示面板会显示橙色“录制已安全中断并保存”及原因。先在示教器解除故障并重新使能机械臂。运行中的遥操程序会每秒自动尝试重建 RTDE 连接；状态显示 `reconnected; release and re-press clutch` 后，必须先松开再重新按住对应手柄侧握把，才会建立新的相对位姿锚点并允许该臂运动。恢复运动后请用 Y/B 或网页按钮开始一个新的 episode，勿继续使用被中断的录制。

## 9. 紧急停止

- 立刻松开对应手柄侧握把：该臂目标停止。
- 终端按 `Ctrl-C`：调用 `servoStop` 并停止所有节点。
- 任何异常、碰撞风险或线缆拉扯风险：使用 UR 示教器急停/保护停止，确认原因后再恢复。
