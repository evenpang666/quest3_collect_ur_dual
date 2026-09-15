#!/usr/bin/env bash
# Standalone front D455 preview.  This script does not start ROS nodes or robots.
set -Eeo pipefail

PROJECT_DIR="/home/mypc/Documents/pyw/projects/quest3_collect"
HARDWARE_CONFIG="$PROJECT_DIR/config/hardware_left_teleop.yaml"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "ERROR: leave Conda first (conda deactivate), then rerun this script." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

exec ros2 run quest3_robot_data view_front_camera \
  --hardware-config "$HARDWARE_CONFIG" "$@"
