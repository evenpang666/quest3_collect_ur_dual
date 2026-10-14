# 安装与环境配置

此文档保留完整安装步骤。日常启动请返回 [README](../README.md)。

## 现场安装步骤

当前主机已升级到 Ubuntu 24.04（系统 Python 3.12），应使用 ROS 2 Jazzy。旧的 Humble 是 Ubuntu 22.04 / Python 3.10 构建，无法在当前系统 Python 下导入 `rclpy`。以下安装步骤在**新终端**中执行；不要加载 Humble 或 Conda 环境。官方说明：[Jazzy Ubuntu deb 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

先停用升级后遗留的 Jammy ROS 软件源（当前机器位于 `/etc/apt/sources.list.d/ros2.sources`），再按官方方式安装 Noble 软件源：

```bash
conda deactivate 2>/dev/null || true
sudo mv /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.sources.jammy.disabled
sudo apt update
sudo apt install -y curl software-properties-common
sudo add-apt-repository universe
curl -fL --retry 3 -o /tmp/ros2-apt-source.deb \
  'https://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source/ros2-apt-source_1.3.0~noble_all.deb'
dpkg-deb --info /tmp/ros2-apt-source.deb
sudo dpkg -i /tmp/ros2-apt-source.deb
grep '^Suites:' /etc/apt/sources.list.d/ros2.sources  # 应显示 noble
sudo apt update
apt-cache policy ros-jazzy-ros-base  # 应显示 Candidate 版本
```

`curl -f` 会在 404 等 HTTP 错误时直接报错；`dpkg-deb --info` 必须能读出包信息后再执行安装。此前不带 `-f` 的命令可能把 `Not Found` 保存成 `.deb` 文件。

安装 Jazzy 及本项目依赖：

```bash
sudo apt install -y ros-jazzy-ros-base ros-jazzy-realsense2-camera \
  ros-jazzy-cv-bridge ros-jazzy-launch-ros \
  python3-opencv python3-aiohttp python3-h5py python3-numpy \
  python3-venv python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c 'import rclpy; print("Jazzy rclpy OK")'
ros2 --help >/dev/null
```

`ur-rtde` 和 `pyrealsense2` 装进独立的 Python 3.12 目录，供系统 Python 的 ROS 命令通过 `PYTHONPATH` 使用。`--no-deps` 保留 APT 提供的 NumPy，避免 pip 覆盖 ROS 的二进制依赖：

```bash
/usr/bin/python3 -m venv --system-site-packages ~/ros2_ws/pydeps_jazzy
~/ros2_ws/pydeps_jazzy/bin/python -m pip install --no-deps ur-rtde pyrealsense2
```

使用新的 Jazzy 构建目录；旧 `~/ros2_ws/install` 属于 Humble，不能继续加载：

```bash
mkdir -p ~/ros2_ws/src
test -e ~/ros2_ws/src/quest3_robot_data || \
  ln -s /home/mypc/Documents/pyw/projects/quest3_collect ~/ros2_ws/src/quest3_robot_data
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon --log-base log_jazzy build --symlink-install --packages-select quest3_robot_data \
  --build-base build_jazzy --install-base install_jazzy
source /home/mypc/Documents/pyw/projects/quest3_collect/real_scripts/ros_environment.sh
ros2 pkg prefix quest3_robot_data
/usr/bin/python3 -c 'import rclpy, rtde_control, pyrealsense2; print("ROS and hardware Python modules OK")'
```

此后每个新终端按[快速命令第 1 节](QUICKSTART_COMMANDS.md#1-每个新终端先加载环境)加载环境。实际机械臂启动前仍需完成标定和只读预检。

