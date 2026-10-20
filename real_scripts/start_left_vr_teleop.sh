#!/usr/bin/env bash
# Start the calibrated, slow, position-only left-arm Quest 3 teleoperation.
# Some ROS setup scripts reference optional environment variables before
# defining them, so enable nounset only after sourcing ROS below.
set -Eeo pipefail

PROJECT_DIR="/home/mypc/Documents/pyw/projects/quest3_collect"
HARDWARE_CONFIG="$PROJECT_DIR/config/hardware_left_teleop.yaml"
RECORDER_CONFIG="$PROJECT_DIR/config/single_arm_debug.yaml"
OUTPUT_DIR="$PROJECT_DIR/data/left_vr_episodes"
CONFIRMATION="MOVE_LEFT_ARM"
TASK_DESCRIPTION=""
RECORD_DEPTH=false

if [[ "${1:-}" != "--confirm" || "${2:-}" != "$CONFIRMATION" || "${3:-}" != "--task" || -z "${4:-}" || $# -lt 4 || $# -gt 5 || ( $# -eq 5 && "${5:-}" != "--record-depth" ) ]]; then
  cat <<EOF
This starts REAL left-UR motion via RTDE (slow, position-only commissioning mode).
Before continuing: clear the workspace, keep the E-stop reachable, set the UR to
Remote Control / running, and verify the pendant's active TCP is the calibrated
closed Robotiq fingertip TCP.

Run exactly:
  $0 --confirm $CONFIRMATION --task "Describe the task in natural language" [--record-depth]

Default: RGB and robot data only. Add --record-depth to include front depth.
EOF
  exit 2
fi
TASK_DESCRIPTION="$4"
if [[ $# -eq 5 ]]; then
  RECORD_DEPTH=true
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "ERROR: leave Conda first (conda deactivate), then rerun this script." >&2
  exit 2
fi

source "$PROJECT_DIR/real_scripts/ros_environment.sh"
set -u

mkdir -p "$OUTPUT_DIR"
ros2 run quest3_robot_data preflight --config "$HARDWARE_CONFIG"
adb reverse tcp:8765 tcp:8765

cleanup() {
  trap - INT TERM EXIT
  for task_pid in "${LAUNCH_PID:-}" "${CAMERA_PID:-}"; do
    [[ -n "$task_pid" ]] && kill -INT "$task_pid" 2>/dev/null || true
  done
  for task_pid in "${LAUNCH_PID:-}" "${CAMERA_PID:-}"; do
    [[ -n "$task_pid" ]] && wait "$task_pid" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

echo "Starting left wrist and front RealSense cameras..."
echo "Depth capture and recording: $RECORD_DEPTH"
ros2 launch quest3_robot_data realsense_four.launch.py \
  hardware_config:="$HARDWARE_CONFIG" \
  enable_depth:="$RECORD_DEPTH" &
CAMERA_PID=$!

echo "Waiting 4 seconds for RealSense initialization..."
# Do not use `ros2 topic echo --once` as a launch gate here.  On a previous
# installation it can wait indefinitely despite the D455 already publishing
# best-effort images, leaving the WebXR server unable to start.  The bridge
# itself subscribes with sensor-data QoS and continually retries `/front.jpg`.
sleep 4

echo "Quest: open http://localhost:8765 in Quest Browser, click connect, then enter VR."
echo "Teleop: LEFT controller only. Hold the side-grip clutch before moving."
echo "Gripper: left trigger held = slow close; released = slow open (independent of clutch)."
echo "Recording: use the VR page record button. Ctrl-C stops robot motion and all nodes."
echo "Task label for every episode in this run: $TASK_DESCRIPTION"

ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:="$HARDWARE_CONFIG" \
  recorder_config:="$RECORDER_CONFIG" \
  output_dir:="$OUTPUT_DIR" \
  task_description:="$TASK_DESCRIPTION" \
  record_depth:="$RECORD_DEPTH" \
  front_topic:=/front_camera/camera/color/image_raw &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
