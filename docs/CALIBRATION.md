# 硬件配置与标定

日常启动见[命令快查](QUICKSTART_COMMANDS.md)。先完成 TCP，再标定相机外参；相机、底座或 TCP 改变后须重新标定。

## 硬件身份

核对 `hardware*.yaml` 的左右臂角色、独立静态 IP、相机序列号和活动 TCP 来源。只读使用 `enable_motion: false`；实体双臂使用 `hardware_dual_teleop.yaml`，实体左臂使用 `hardware_left_teleop.yaml`。

四台 RealSense 与 Quest 分散接入 USB 3 root hub，避免共用无源 Hub。两臂通过以太网连接，RTDE 端口为 `30004`。自定义软件工作空间须同时设置 `workspace_min/max`。

## TCP 标定（先做）

若使用示教器已标定并激活的 TCP，配置 `robot_active_tcp`，脚本不会重设偏移。需要项目 Pivot 标定时：示教器先将 TCP 清零，固定工具尖端，采集至少 12 个差异明显的法兰姿态。

```bash
ros2 run quest3_robot_data calibrate_tcp \
  --robot-ip 192.168.124.61 \
  --base-frame left_base --flange-frame left_tool0 --tcp-frame left_tcp \
  --samples 12 --output config/calibration/left_tcp.yaml
```

右臂替换 IP 和坐标系，输出 `right_tcp.yaml`。Pivot 只求平移；旋转通过 `--tcp-rotation-rotvec` 指定，默认与法兰一致。RMS 通过后，将 `tcp_offset_xyz_rotvec` 写入对应示教器并激活。

## 双臂 front/side ChArUco 相机外参标定

停止所有相机占用程序，保持两台相机与底座固定。使用当前现场标定板：11×8 方格、格长 70 mm、marker 52 mm、`DICT_5X5_100`、`legacy` 排列（左上方格内有 marker）。参数必须与实物一致。

```bash
ros2 run quest3_robot_data calibrate_dual_camera_charuco \
  --config config/hardware_dual_teleop.yaml \
  --squares-x 11 --squares-y 8 \
  --square-size-m 0.070 --marker-size-m 0.052 \
  --dictionary DICT_5X5_100 \
  --board-pattern legacy \
  --camera-samples 30 --max-touch-rms-mm 2.5
```

每臂流程为：固定板 → front / side 各按空格采样 → TCP 触四个角点 → 移开机械臂、检查板位移。**本臂全程板不得移动；左臂通过后可移动板，再独立标定右臂。** 两臂可选不同点和板位置，不要求工作空间重叠。

程序优先选择两台相机共同看见且分散的角点，提示 ID、板网格行列并保存标注图。ID 从 0 开始，行列从板左上角起、从 1 开始，只数内部交点；11×8 方格有 10×7 个内角。第1行第1列为 ID0，第1行第9列为 ID8，第1行第10列为 ID9；图中 `ID9 R1C10` 表示 ID9、第1行第10列，ID19 则为第2行第10列。板旋转后，以板自身方向为准。不可达时输入 `id=N` 替换，可选任意有效内角；四点不可重复或共线，替换点未共同检测时需按行列核对实物。

`--board-pattern` 默认 `legacy`，兼容现场实物与 OpenCV 4.6；若使用新版 OpenCV 默认生成的板（左上方格为纯黑），指定 `--board-pattern opencv`。程序显式定义 marker 坐标，不依赖 OpenCV 版本的默认布局。需识别至少6个 marker，覆盖奇偶两类方格行；角点必须由相邻两个已识别 marker 支持。全部已识别的板内 marker 都参与几何检查，布局不符时中止，不再丢弃交替行来拟合。

**旧版曾出现图中第一内角标为 P1 的整列偏移。修正前保存的观测、触点和最终结果不得直接复用。** 检查点与最终结果包含板排列和模型版本，缺失或不匹配时拒绝续做/跳过。备份移走待重做臂的 front/side 两份最终 YAML，运行上述命令重新采集并重触四点；不要加 `--resume`。旧观测和触点会在新会话中重新写入，旧标注图不会自动变成有效观测。偶数行布局兼容问题见 [OpenCV 官方记录](https://github.com/opencv/opencv/issues/23152)。

RGB 格式按设备支持情况选择，优先 `1280×720@30`，其次 `@15` 或 `640×480`；使用实际内参，以 `RGB ready` 日志为准。现场 side 曾通过 USB 2.1 使用 `1280×720@15`，接线变化后重新核对。

### 分次、续做与重做

| 需求 | 方法 |
| --- | --- |
| 只标定左 / 右臂 | 在命令后加 `--arm left` / `--arm right` |
| 中断后继续 | 板仍在原位时加 `--arm right --resume`；移动过则重新采集 |
| 指定触点 | 加 `--touch-corner-ids`，两臂可不同 |
| 重做已通过结果 | 先备份移走该臂 front/side 两份最终 YAML，再运行新会话 |
| 来源改变、重算汇总 | 重跑原命令，保留匹配且通过的相机对，重新换算 |

新会话重新采集未完成臂的观测和触点；已通过且设备/坐标系匹配的完整结果会跳过。任一相机未过质量门时，该臂两份结果均标记 `calibrated: false`。

### 验收结果

输出至 `config/calibration/`：

| 机器人 | front | side |
| --- | --- | --- |
| 左臂 | `left_base_to_front_camera.yaml` | `left_base_to_side_camera.yaml` |
| 右臂 | `right_base_to_front_camera.yaml` | `right_base_to_side_camera.yaml` |

四份外参通过后，通过固定相机换算 `left_T_right = left_T_camera × inverse(right_T_camera)`，生成 `dual_arm_camera_transforms.yaml`。front/side 两条路径及两臂相机相对位姿须一致，默认阈值 `10 mm / 3°`，可用 `--max-cross-translation-mm` / `--max-cross-rotation-deg` 调整。

汇总失败保留四份结果，检查相机/底座移动和触点错误后重做受影响臂。四份外参与汇总均为 `calibrated: true` 且来源未过期，预检才允许启动双臂。front 外参用于 TCP 距离换算，不决定手柄映射方向；所有外参随 episode 保存。

旧单臂工具 `calibrate_camera_charuco`（桌面板）和 `calibrate_camera`（TCP 刚性 ArUco）仍可用，但不可混用其标定流程。早期现场记录见[调试归档](LEFT_ARM_DEBUG_HISTORY.md)。
