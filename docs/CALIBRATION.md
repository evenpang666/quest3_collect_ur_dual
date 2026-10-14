# 硬件配置与标定

当前现场的配置文件、板尺寸和操作命令以 [命令快查](QUICKSTART_COMMANDS.md) 为准。以下说明配置项和标定原理。

## 填写硬件身份

只读双臂配置为 [hardware.yaml](../config/hardware.yaml)，实体双臂脚本使用 [hardware_dual_teleop.yaml](../config/hardware_dual_teleop.yaml)，单臂只读配置为 [hardware_single.yaml](../config/hardware_single.yaml)。先核对各配置中的：

1. 明确记录哪台实体机器人是 `left_arm`，为两台 UR 设置不同静态 IP；
2. 填入两台机器人 IP；
3. 用 `rs-enumerate-devices` 查出四台相机序列号并填入；
4. 如使用软件工作空间限制，根据现场测量同时设置 `workspace_min/max`；
5. 只读验证使用 `enable_motion: false` 的配置。实体遥操作配置会启用运动，须先完成标定和预检。

四台 Realsense 和 Quest 都走 USB，建议把相机分散到不同 USB 3 root hub。不要通过无源 Hub 同时连接四路视频设备。两台 UR 可通过千兆交换机接主机网口，确保 IP 不冲突且 RTDE 端口 30004 可达。

## TCP 标定（先做）

Pivot 标定只能求 TCP 平移。TCP 朝向来自夹爪 CAD/安装方向，通过 `--tcp-rotation-rotvec` 指定；默认认为和法兰方向一致。

在 UR 示教器中先把当前 TCP 设为全零。将夹爪工具尖端固定在同一机械点，至少采集 12 个差异明显的法兰姿态：

```bash
ros2 run quest3_robot_data calibrate_tcp \
  --robot-ip 192.168.124.61 \
  --base-frame left_base --flange-frame left_tool0 --tcp-frame left_tcp \
  --samples 12 \
  --output /home/mypc/Documents/pyw/projects/quest3_collect/config/calibration/left_tcp.yaml
```

右臂重复执行并输出 `right_tcp.yaml`。只有 RMS 误差通过时文件才会标记 `calibrated: true`。随后将 `tcp_offset_xyz_rotvec` 填入对应 UR Installation 的 TCP 设置并设为活动 TCP。

## 双臂 front/side ChArUco 相机外参标定

现场流程使用固定在桌面的 ChArUco 板，按**先左臂、再右臂**的顺序标定。每个臂先采集 front/side 两台相机，再用该臂的活动 TCP 触碰四个共同可见内角，分别求解两份 base-to-camera 外参。无需将标记刚性安装在 TCP 上。

```bash
ros2 run quest3_robot_data calibrate_dual_camera_charuco \
  --config /home/mypc/Documents/pyw/projects/quest3_collect/config/hardware_dual_teleop.yaml \
  --squares-x 11 --squares-y 8 \
  --square-size-m 0.070 --marker-size-m 0.052 \
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

四份外参和换算汇总均须为 `calibrated: true`。front 外参用于双臂基座关系与 TCP 距离检查；手柄相对偏移按当前 Tool/TCP 坐标轴映射，不由 front 相机朝向决定。side 外参随 episode 保存，用于三维数据处理。移动相机、机械臂底座或改变 TCP 后必须重新标定。独立标定、断点续做及重做已通过结果的方法见 [快速启动第 4 节](QUICKSTART_COMMANDS.md#4-双臂双相机-charuco-外参标定先左臂再右臂)。原有单臂单相机 `calibrate_camera_charuco` 和 TCP 刚性 ArUco `calibrate_camera` 命令继续保留。

