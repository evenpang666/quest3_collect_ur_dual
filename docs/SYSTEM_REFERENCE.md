# 系统与数据参考

启动命令见[命令快查](QUICKSTART_COMMANDS.md)，设备身份与外参见[配置与标定](CALIBRATION.md)。

## 系统结构

```text
Quest WebXR → webxr_bridge → pose_mapper → rtde_servo → UR7e / Robotiq
四路 RealSense + 机器人状态 → recorder 同步缓冲 → 后台图像转换 / HDF5 写盘
```

Quest 通过 USB `adb reverse` 连接主机；头显显示现实透视与录制状态，当前不请求相机预览。控制、相机、录制运行在不同进程。

| 用途 | 硬件配置 | 录制配置 |
| --- | --- | --- |
| 实体双臂 | `hardware_dual_teleop.yaml` | `dual_arm.yaml` |
| 实体左臂 | `hardware_left_teleop.yaml` | `single_arm_debug.yaml` |
| 只读双臂 / 单臂 | `hardware.yaml` / `hardware_single.yaml` | `dual_arm.yaml` / `single_arm.yaml` |

手动启动只读系统（另行启动相机）：

```bash
ros2 launch quest3_robot_data realsense_four.launch.py hardware_config:="$PWD/config/hardware.yaml"
# 另一个已加载环境的终端
ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:="$PWD/config/hardware.yaml" \
  recorder_config:="$PWD/config/dual_arm.yaml" \
  output_dir:="$PWD/data/readonly" task_description:="read-only check"
```

深度默认关闭。手动启动时，相机设置 `enable_depth:=true`，系统设置 `record_depth:=true`；实体脚本的 `--record-depth` 同时设置二者。独立录制器也接受 `--record-depth`。

## 控制与安全边界

- 左右手柄分别控制对应机械臂。侧握把为独立死手开关；松开停止并清除锚点，扳机独立控制夹爪。
- 每次握住时记录手柄 `gripSpace` 和该臂 Tool/TCP 位姿；后续相对位姿按 X 同向、Y/Z 反向换算。手柄前方（-Z）对应 Tool +Z，实际方向取决于活动 TCP 姿态。
- 普通双臂配置 `rtde.limit_cartesian_targets: false`，完整 1:1 目标直接发往 `servoL`，由 UR 内部完成 IK。实际运动受控制柜安全设置、伺服响应及延迟影响；`ur_rtde 1.6.5` 的 `servoL/servoJ` 不使用 `speed/acceleration` 参数。
- 其他配置默认保留 `max_target_jump_m/rad` 限幅；[专用避障](AVOIDANCE.md)另有步长约束。
- 目标 / 手柄超过 `0.25 s` 未更新会停止；双臂缺任一手柄时均不能运动或开始录制。恢复后须松开再握住。
- 双臂目标 TCP 在 front 坐标中距离小于 `0.20 m` 时同时停止。此保护不覆盖连杆或场景，不能证明无碰撞。
- 当前实体配置未设置软件 Cartesian 工作空间；自定义时 `workspace_min/max` 必须成对设置。运动边界由 UR 安全平面、关节/速度限制和急停负责。

只读配置 `enable_motion: false`；实体配置为 `true`。首次运动前完成标定、预检、左右映射和低速空载检查。软件保护不能替代实体急停和现场监护。急停、保护停机或 RTDE 失联会结束当前录制；恢复步骤见[运行诊断](QUICKSTART_COMMANDS.md#8-常用运行时诊断)。

## 数据格式与同步

front RGB 为参考时钟，按时间戳从旧到新处理；每次回调最多处理 `aggregate_batch_size` 帧，让订阅回调继续接收消息。其他图像和 action / 夹爪选择最近时间戳，关节优先插值，无法插值时回退到容差内的最近状态。

缺少来源时，保留当前参考帧最多等待 `max_skew_sec`（主机单调时钟计时），不会因新 front 帧到达而跳过；仍无法匹配则丢弃整帧。相机帧可能被多个相邻样本复用，所有实际来源时间戳均保存。

回调只选择消息引用和状态；图像转换、中心裁剪/缩放、编码及写盘由后台线程执行。样本队列有容量限制，满时计数并丢弃本帧。停止命令按同一 FIFO 排序，提交时不等待磁盘；后台写完已接收样本后才完成文件。

```text
observations/images/{left_wrist,right_wrist,front,side}/rgb
observations/images/{front,side}/depth  # 仅 record_depth=true
observations/qpos                     # 6/12维，rad
observations/qvel                     # 6/12维，rad/s
observations/gripper_position         # 1/2维，0开至1闭
action                                # 目标 q - 实际 q，rad
executed_action                       # qpos[t+1] - qpos[t]，rad
gripper_action                        # 1/2维，0开至1闭
action_valid                          # 末帧 false，executed_action 为零
is_terminal                           # 末帧 true
timestamps_ns
source_timestamps_ns/*
```

所有数据集首维是相同样本索引；关闭时检查长度。文件嵌入任务描述、硬件配置和标定内容，深度选择由 `record_depth` 属性记录。写入 `episode_XXXXX.inprogress`，成功关闭后原子改名为 `.hdf5`；异常保留临时文件。

## 存储参数

修改对应 `config/*arm*.yaml` 后重启录制器。

| 参数 | 默认值 / 含义 |
| --- | --- |
| `rgb_storage` | `raw`：无损无压缩、`N×H×W×3 uint8`，优先保证吞吐 |
| `raw_lzf` | 可选无损 LZF，数组结构相同，增加压缩 CPU 开销 |
| `jpeg` | 可选有损 JPEG；保存可变长字节，读取时先解码 |
| `jpeg_quality` | `90`，仅 JPEG 生效 |
| `record_depth` | `false`；开启时深度保持原值并以 LZF 无损保存 |
| `source_buffer_size` | `30`，各来源缓存及图像/关节 DDS 订阅深度 |
| `writer_queue_size` | `30`，后台最多排队的样本数 |
| `aggregate_batch_size` | `4`，单次同步回调处理上限 |
| `max_skew_sec` | `0.050`，最大来源偏差和缺帧重试时间 |
| `max_joint_interp_gap_sec` | `0.100`，关节插值跨度上限 |

四路 `640×480 RGB @30 FPS` 原始图像约 **111 MB/s、6.6 GB/分钟**，深度另计。使用持续写入足够快的本地磁盘；增大队列只吸收短时波动，不能弥补长期吞吐不足。旧 `folder_npy` 后端仍支持 RGB JPEG、深度 NPY 和 JSONL 状态。

## 丢帧诊断

```bash
ros2 topic echo /quest3/system_status
ros2 topic hz /front_camera/camera/color/image_raw
ros2 run quest3_robot_data inspect_episode data/dual_vr_episodes/pick_place/episode_00000.hdf5
```

计数为录制器进程累计值：`synced_frames` 为匹配成功数，`enqueued_frames` 为成功排队数，`dropped_frames` 为已观察到的丢帧总数。`drop_reasons` 分类如下：

| 原因 | 含义与检查方向 |
| --- | --- |
| `reference_overflow` | front 缓冲覆盖了未处理帧；检查 CPU 负载与订阅积压 |
| `sync_timeout` | 来源缺失或时间戳偏差超限；查看 `last_sync_error` 指出的来源，检查相应话题 |
| `writer_queue_full` | 磁盘/压缩消费过慢；检查持续写速、存储模式和并行 I/O |
| `invalid_sample` | 状态维度等异常；查看录制器错误日志 |

这些计数不包含相机或 DDS 在回调前丢失的消息。`inspect_episode` 会输出实际采样率、长间隔及估算缺帧数；默认参考 30 FPS，其他源帧率可加 `--expected-fps 15`，估算值并非驱动层精确计数。`writer_error` 非空表示后台失败，检查磁盘空间和 `.inprogress`。仅提高 `max_skew_sec` 会降低同步质量，不应用来掩盖源数据缺失。RTDE 周期仍抖动时再评估实时内核/调度配置。
