#!/usr/bin/env bash
# Source from a fresh shell before running this project's ROS commands.

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "ERROR: ROS ${ROS_DISTRO} is already loaded. Open a fresh terminal for Jazzy." >&2
  return 1
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy is missing. Follow the Ubuntu 24.04 installation steps in README.md." >&2
  return 1
fi

if [[ ! -f "$HOME/ros2_ws/install_jazzy/local_setup.bash" ]]; then
  echo "ERROR: the Jazzy workspace is missing. Build it with the commands in README.md." >&2
  return 1
fi

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install_jazzy/local_setup.bash"

# The system-Python ROS entry points also need these two non-ROS Python wheels.
PY_DEPS="$HOME/ros2_ws/pydeps_jazzy/lib/python3.12/site-packages"
if [[ -d "$PY_DEPS" ]]; then
  export PYTHONPATH="$PY_DEPS${PYTHONPATH:+:$PYTHONPATH}"
fi
unset PY_DEPS

if ! /usr/bin/python3 -c 'import rclpy' >/dev/null 2>&1; then
  echo "ERROR: system Python cannot import Jazzy rclpy; check the ROS installation." >&2
  return 1
fi
