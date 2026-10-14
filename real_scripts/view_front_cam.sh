#!/usr/bin/env bash
# Standalone preview of every connected RealSense RGB stream.
# This script does not start ROS nodes or robots.
set -Eeo pipefail

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "ERROR: leave Conda first (conda deactivate), then rerun this script." >&2
  exit 2
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_DIR/real_scripts/ros_environment.sh"

exec ros2 run quest3_robot_data view_realsense_cameras "$@"
