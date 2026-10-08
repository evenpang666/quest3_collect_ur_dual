#!/usr/bin/env bash
# Start real, 6-DoF Quest teleoperation for both calibrated UR7e arms.
# ROS setup runs before nounset because Humble setup scripts use optional vars.
set -Eeo pipefail

PROJECT_DIR="/home/mypc/Documents/pyw/projects/quest3_collect"
HARDWARE_CONFIG="$PROJECT_DIR/config/hardware_dual_teleop.yaml"
RECORDER_CONFIG="$PROJECT_DIR/config/dual_arm.yaml"
OUTPUT_DIR="$PROJECT_DIR/data/dual_vr_episodes"
CONFIRMATION="MOVE_BOTH_ARMS"
TASK_DESCRIPTION=""

if [[ "${1:-}" != "--confirm" || "${2:-}" != "$CONFIRMATION" || "${3:-}" != "--task" || -z "${4:-}" || $# -ne 4 ]]; then
  cat <<EOF
This starts REAL motion for BOTH UR7e arms.
Before continuing: clear both workspaces, make E-stops reachable, set both URs
to Remote Control / running, and verify each pendant's active TCP is its own
calibrated closed Robotiq fingertip TCP.

Run exactly:
  $0 --confirm $CONFIRMATION --task "Describe the task in natural language"
EOF
  exit 2
fi
TASK_DESCRIPTION="$4"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "ERROR: leave Conda first (conda deactivate), then rerun this script." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u

mkdir -p "$OUTPUT_DIR"
ros2 run quest3_robot_data preflight --config "$HARDWARE_CONFIG"
adb reverse tcp:8765 tcp:8765

cleanup() {
  trap - INT TERM EXIT
  for task_pid in "${SYSTEM_PID:-}" "${CAMERA_PID:-}"; do
    [[ -n "$task_pid" ]] && kill -INT "$task_pid" 2>/dev/null || true
  done
  for task_pid in "${SYSTEM_PID:-}" "${CAMERA_PID:-}"; do
    [[ -n "$task_pid" ]] && wait "$task_pid" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

echo "Starting both wrist cameras and front/side D455..."
ros2 launch quest3_robot_data realsense_four.launch.py \
  hardware_config:="$HARDWARE_CONFIG" &
CAMERA_PID=$!
echo "Waiting 4 seconds for RealSense initialization..."
sleep 4

echo "Quest: open http://localhost:8765, connect, then enter VR."
echo "Left controller -> left UR; right controller -> right UR."
echo "Hold a controller side-grip to move that arm; each trigger operates its gripper."
echo "Ctrl-C stops both servo loops and all nodes."
echo "Task label for every episode in this run: $TASK_DESCRIPTION"
ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:="$HARDWARE_CONFIG" \
  recorder_config:="$RECORDER_CONFIG" \
  output_dir:="$OUTPUT_DIR" \
  task_description:="$TASK_DESCRIPTION" \
  front_topic:=/front_camera/camera/color/image_raw &
SYSTEM_PID=$!
wait "$SYSTEM_PID"
