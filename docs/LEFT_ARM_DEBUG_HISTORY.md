# 左臂现场调试归档

历史记录，起于 **2026-09-08**。当前参数、显示模式和录制格式以[命令快查](QUICKSTART_COMMANDS.md)为准。

## 早期只读现场

最小设备为左 UR7e、左腕 D435i、front D455 和 Quest 3。

| 设备 | 当时参数 |
| --- | --- |
| 左 UR7e | `192.168.124.61/24`，RTDE `30004`、夹爪 `63352` |
| 主机有线口 | `enp4s0`，`192.168.124.59/24`，无网关 |
| Quest ADB | `2G97C5ZJ2L05V1` |
| front D455 | `260922305390` |
| 左腕 D435i | `420222071502`，当时替换了 `405622072940` |

使用 `hardware_left_debug.yaml + single_arm_debug.yaml`。调试硬件配置仅允许 `enable_motion: false`，可在缺少标定时验证相机、手柄与机器人状态；打开运动会被拒绝。历史上“未在线”的序列号不代表当前设备状态。

当时头显显示 front 画面、页面版本为 `orientation-v2`；当前改用 `passthrough-v6` 现实透视。旧的 HDF5 自动估算视频帧率说明也已过时，当前导出默认固定 30 FPS。

## 保留的专项工具

RTDE 微动测试默认 dry-run，仅计算第六轴 `+0.5°` 目标：

```bash
ros2 run quest3_robot_data joint_motion_probe --ip 192.168.124.61
```

确认 Remote Control、现场清空、急停可触达及腕部线缆允许运动后，才可加 `--execute --confirm ROBOT_AREA_CLEAR`。工具限制位移不超过 `1°`、速度 `0.10 rad/s`、加速度 `0.20 rad/s²`；仅验证 RTDE 通道，不代表 VR 映射已验证。

旧桌面板工具 `calibrate_camera_charuco` 当时使用 10×7、37 mm 格子、27 mm marker、`DICT_5X5_100`；此板规格不同于当前双臂标定板。相机观测和触点即时保存；板或相机移动后用 `--recapture-camera` 并重采对应 `_touches.yaml`。局部重触可用 `--retouch-corner-ids 8 31`。

TCP、活动工具坐标与外参未确认时，不得用调试配置执行实体运动。当前双臂标定、实体遥操、录制及恢复流程均已集中到[配置与标定](CALIBRATION.md)和[命令快查](QUICKSTART_COMMANDS.md)。
