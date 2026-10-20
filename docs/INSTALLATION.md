# 安装与构建

环境：**Ubuntu 24.04 + ROS 2 Jazzy + 系统 Python 3.12**。使用新终端，不加载 Humble 或 Conda。

## 安装依赖

先按 [ROS 2 Jazzy 官方说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)配置 Noble 软件源；从 Ubuntu 22.04 升级的主机需先停用旧 Jammy ROS 源。可用 `apt-cache policy ros-jazzy-ros-base` 检查是否存在候选版本。

```bash
conda deactivate 2>/dev/null || true
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-realsense2-camera \
  ros-jazzy-cv-bridge ros-jazzy-launch-ros \
  python3-opencv python3-aiohttp python3-h5py python3-numpy \
  python3-venv python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
```

非 ROS Python 依赖装入独立目录；`--no-deps` 保留 APT 提供的 NumPy 等二进制依赖：

```bash
/usr/bin/python3 -m venv --system-site-packages ~/ros2_ws/pydeps_jazzy
~/ros2_ws/pydeps_jazzy/bin/python -m pip install --no-deps ur-rtde pyrealsense2
```

## 构建

在仓库根目录执行。使用独立 Jazzy 构建目录，不加载旧 Humble 工作区：

```bash
mkdir -p ~/ros2_ws/src
test -e ~/ros2_ws/src/quest3_robot_data || \
  ln -s "$PWD" ~/ros2_ws/src/quest3_robot_data
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon --log-base log_jazzy build --symlink-install --packages-select quest3_robot_data \
  --build-base build_jazzy --install-base install_jazzy
```

## 验证环境

```bash
source /home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/ros_environment.sh
ros2 pkg prefix quest3_robot_data
/usr/bin/python3 -c 'import rclpy, rtde_control, pyrealsense2; print("ROS and hardware Python modules OK")'
```

日后每个终端只需加载 `real_scripts/ros_environment.sh`，它会加载 Jazzy、项目工作区和 `pydeps_jazzy`。实际运动前继续完成[标定](CALIBRATION.md)和[只读预检](QUICKSTART_COMMANDS.md#3-标定文件状态)。
