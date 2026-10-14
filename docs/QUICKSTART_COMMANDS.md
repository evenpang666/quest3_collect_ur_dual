# Quest 3 + 双 UR7e 命令快查（当前现场）

以下命令默认在仓库根目录执行。本文档记录当前现场配置与操作流程；双臂启动前须完成四份相机外参标定。详细实现和历史只读调试见 [左臂只读调试归档](LEFT_ARM_DEBUG_HISTORY.md)。

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
source /home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/ros_environment.sh
```

代码修改后重建：

```bash
cd ~/ros2_ws
colcon --log-base log_jazzy build --symlink-install --packages-select quest3_robot_data \
  --build-base build_jazzy --install-base install_jazzy
source install_jazzy/local_setup.bash
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
  --task "pick place"
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
  --task "pick place"
```

启动脚本通过 ADB 保持 Quest 唤醒，并在脚本退出时恢复接近传感器。戴头显在 Quest Browser 打开 `http://localhost:8765`，点击“连接”→“进入现实透视”；确认左右手柄均被追踪后，先松开两侧握把，再摘下头显并把它稳放在能看见两只手柄的位置。头显仍须通电，浏览器会话须保持运行。若手柄失去追踪，双臂会停止，录制会中断。Quest 扬声器需在操作者听得到的位置。

摘下后直接用手柄遥操；左手 **Y** 或右手 **B** 切换录制。录制器确认开始时播放两声升调，确认停止时播放两声降调；录制中断或开始被拒绝时播放三声低调。提示音从 Quest 扬声器播放，依据录制器回传状态，不以按键动作代替确认。若启动脚本异常终止，可执行 `bash real_scripts/quest_keep_awake.sh off` 恢复接近传感器；手动保持唤醒可执行 `bash real_scripts/quest_keep_awake.sh on`。两者均要求 Quest 已通过 ADB 连接。

### 与 safety 同时运行：无相机双臂遥操

先在 `../safety` 独立启动检测服务，由它独占 front/side：

```bash
cd ../safety
/home/mypc/anaconda3/envs/safety/bin/python \
  real_scripts/live_obstacle_detection.py --no-browser --no-lingbot-depth
```

随后在本项目另一个终端运行专用脚本：

```bash
real_scripts/start_dual_vr_teleop_no_cameras.sh --confirm MOVE_BOTH_ARMS
```

该脚本启动 Quest WebXR、位姿映射、双臂 RTDE 控制以及 safety Python 环境中的
QP-CBF 计算进程；预检不会查询 RealSense，也不会启动任何相机节点或 episode
录制器。Quest 使用现实透视，录制按钮会禁用。右手柄 **A 键**按一次开启避障，
再按一次关闭；切换后松开侧握把再按住。头显显示开关状态和提示。
避障模式仅保留最新手柄目标；每轮从当前机械臂反馈生成一条受 RTDE 单步限制的
指令，使用 safety 的固定点与扫掠路径检查。未侵入 OBB 安全边界、双臂间距也安全时，
直接发送该单步关节指令，不运行 QP，也不逐步暂停；检测到风险时才暂停旧指令，
重新读取机械臂反馈和最新手柄目标，并调用 safety 的 QP-CBF 优化本步指令。
结果返回时会检查最新手柄指令是否仍与该单步指令一致、关节反馈是否漂移；
过时结果直接丢弃，再从最新输入继续判断。
避障默认每 `50 ms` 计划一条动作；手柄目标的平移/旋转限幅按这个周期换算，
不再把 `125 Hz` 的单次 `servoL` 限幅直接用于整条避障动作。最终关节步长仍受
`safety` 的关节速度限制，并逐步执行固定点与扫掠路径检查。
更新脚本后请退出旧 WebXR 会话并刷新 Quest Browser 页面，确认显示 `passthrough-v6`。
专用遥操设置了 `obstacle_avoidance.ignore_snapshot_age: true`：超过 `2.0 s` 的快照
仍会用于 OBB 检查和优化，**不会仅因采集时间过期而停住动作**。如果检测服务持续
发布旧场景，机械臂也会依据旧障碍物位置运动；其他 safety 程序仍保留 `2.0 s`
限时。优化失败、服务不可用、标定不一致等运行错误会
丢弃本次动作并保持避障开启；下一次新手柄输入会重新检查、必要时重新优化。
恢复前不会发送未经验证的动作，也不会切回普通遥操；手动按 A 才关闭避障。
头显显示暂停及恢复提示。普通双臂脚本不启用 A 键避障。
避障启用或计算失败的具体原因和堆栈会出现在专用脚本所在终端的 `pose_mapper` 或
`teleop_avoidance_server` 日志中，便于排查。
专用脚本直接使用当前项目的 Python 源码及 launch 文件；已有 ROS 安装包时，
无需为了新增的 `--skip-camera-check` 参数先重新构建。若 safety 环境不在默认路径，
启动前设置 `SAFETY_PYTHON` 为其 Python 可执行文件。避障计算进程默认使用
`127.0.0.1:8768`；若该端口已被占用，可在启动命令前设置 `QUEST3_AVOIDANCE_PORT=8769`，
脚本会让计算进程与遥操客户端使用同一端口。safety 检测服务仍使用 `8766`。
退出或终止专用遥操脚本时，它会停止自己启动的避障计算进程；独立运行的 safety
检测服务仍由其原终端管理。
需要相机和录制时，停止此脚本及 safety 的相机占用后再使用上面的常规启动脚本。
两种遥操脚本共用机器人 RTDE 和 Quest 的 `8765` 端口，不能同时运行；第二个脚本
会在启动前报错退出。常规采集脚本会显式关闭避障模式，照常启动四路相机与录制器；
专用避障脚本不会启动相机或录制器，且其环境变量不会写回调用它的终端。

- 左手柄侧握把：左臂相对位姿遥操；左扳机：左夹爪。
- 右手柄侧握把：右臂相对位姿遥操；右扳机：右夹爪。
- 两臂均为 1:1 相对位姿、最大平移约 `0.45 m/s`、最大角速度约 `3.0 rad/s`。
- 双臂夹爪速度均为 `255/255`，力度为 `20/255`；单臂启动脚本仍使用速度 `20/255`。
- 两个 TCP 在 front 相机坐标中的距离小于 `0.20 m` 时，双臂同时停止；松开再按住侧握把重新锚定后恢复。

每次握住侧握把时，记录各手柄 `gripSpace` 与对应机械臂当时的 Tool/TCP 位姿；之后的位移和旋转按 X 同向、Y/Z 反向的轴关系换算。头显和 front 相机朝向不决定运动方向。首次实体运动前，低速空载分别核对两臂 Tool 的 X/Y/Z 正负方向；“手柄前方”（-Z）对应 Tool 的正 Z 轴，实际桌面方向取决于活动 TCP 的姿态。

进入现实透视后，必须等左右手柄都被追踪，双臂遥操和录制才会就绪。任一手柄失联或追踪超时，两臂立即撤销运动使能、当前录制停止；恢复追踪后先松开两侧握把，再重新按住锚定。网页与头显会提示手柄未就绪，录制器会拒绝缺手柄的开始请求。

录制：左手柄 **Y 键**、右手柄 **B 键**，或网页“开始录制/停止并保存”按钮均可切换。头显摘下后用声音辨认已确认的录制状态；退出 VR、网页断开或终端 `Ctrl-C` 会停止并保存当前 episode。

双臂启动脚本使用 `hardware_dual_teleop.yaml + dual_arm.yaml`，episode 保存 front/side RGB-D、左右腕 RGB、双臂关节状态、commanded action 和夹爪状态/命令，并嵌入四份外参与双臂换算汇总。`dual_arm_front_only.yaml` 仅保留为无 side 相机时的临时配置。

双臂 episode 按 `--task` 分目录保存：任务名称转为小写，空格替换为下划线；例如 `--task "Pick Up Cube"` 保存到 `data/dual_vr_episodes/pick_up_cube/`。每个任务目录独立编号。

Quest 头显使用 WebXR `immersive-ar` 现实透视：背景透明，可看到真实工作场景；视野上方保留随头部移动的录制状态、计时、Y/B 操作提示和录制中断原因。front / left wrist / right wrist 三路相机画面暂不显示，头显页面不再请求预览 JPEG。相机采集与 episode 保存不受影响：front RGB 源图为 `1280×720@30`，深度为 `640×480@30`；录制器仍将 front RGB 与深度写入 `640×480` HDF5 数据格式，腕部数据不变。

需在 Quest Browser 中通过 `http://localhost:8765`（ADB 转发）或 HTTPS 打开，并允许进入沉浸式会话。若浏览器不支持现实透视，页面会显示错误提示。更新代码后按第 1 节重建并重启遥操脚本，退出旧会话、刷新头显页面；页面版本应为 `passthrough-v6`。

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

一次导出某个任务目录下的全部 HDF5（每个 episode 各生成四路 MP4）：

```bash
/usr/bin/python3 scripts/export_task_episode_videos.py \
  data/dual_vr_episodes/task_name
```

默认跳过四路视频均存在且帧数正确的 episode；中断后直接重跑会重新导出未完成的 episode。需要强制重新导出全部时追加 `--overwrite`。修改代码并重建 ROS 包后，也可使用 `ros2 run quest3_robot_data export_task_episode_videos <任务目录>`。

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

若 Quest 现实透视未显示：退出沉浸式会话、刷新头显页面，确认页面版本为 `passthrough-v6`，在 Quest Browser 中重新点击“连接”→“进入现实透视”并允许进入会话；检查页面错误提示和 Quest Browser / 系统版本。现实背景由 Quest 自身透视提供，不依赖 RealSense 预览。

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
