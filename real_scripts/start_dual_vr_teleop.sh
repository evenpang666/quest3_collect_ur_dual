#!/usr/bin/env bash
# Start real, 6-DoF Quest teleoperation for both calibrated UR7e arms.
# ROS setup runs before nounset because setup scripts use optional vars.
set -Eeo pipefail

PROJECT_DIR="/home/mypc/Documents/pyw/projects/quest3_collect"
HARDWARE_CONFIG="$PROJECT_DIR/config/hardware_dual_teleop.yaml"
RECORDER_CONFIG="$PROJECT_DIR/config/dual_arm.yaml"
OUTPUT_DIR="$PROJECT_DIR/data/dual_vr_episodes"
CONFIRMATION="MOVE_BOTH_ARMS"
TASK_DESCRIPTION=""
RECORD_DEPTH=false

if [[ "${1:-}" != "--confirm" || "${2:-}" != "$CONFIRMATION" || "${3:-}" != "--task" || -z "${4:-}" || $# -lt 4 || $# -gt 5 || ( $# -eq 5 && "${5:-}" != "--record-depth" ) ]]; then
  cat <<EOF
This starts REAL motion for BOTH UR7e arms.
Before continuing: clear both workspaces, make E-stops reachable, set both URs
to Remote Control / running, and verify each pendant's active TCP is its own
calibrated closed Robotiq fingertip TCP.

Run exactly:
  $0 --confirm $CONFIRMATION --task "Describe the task in natural language" [--record-depth]

Default: RGB and robot data only. Add --record-depth to include front/side depth.
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

# Both launch modes control the same robots and Quest bridge. Hold the lock for
# this launch (and its children) so a second mode cannot start concurrently.
exec {TELEOP_LOCK_FD}>/tmp/quest3_dual_vr_teleop.lock
if ! flock -n "$TELEOP_LOCK_FD"; then
  echo "ERROR: another dual-arm Quest teleoperation script is already running." >&2
  exit 2
fi

# Keep the collection path ordinary even if avoidance variables were exported
# in the caller's shell. The dedicated script sets these only for its children.
export QUEST3_ENABLE_AVOIDANCE=0
unset QUEST3_AVOIDANCE_RUNTIME_URL QUEST3_AVOIDANCE_PARENT_PID

source "$PROJECT_DIR/real_scripts/ros_environment.sh"
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
  if [[ "${QUEST_KEEP_AWAKE:-0}" == 1 ]]; then
    bash "$PROJECT_DIR/real_scripts/quest_keep_awake.sh" off || \
      echo "WARNING: could not restore Quest proximity sensor; run quest_keep_awake.sh off." >&2
  fi
}
trap cleanup INT TERM EXIT

# Quest must keep its WebXR session and controller tracking active off-face.
bash "$PROJECT_DIR/real_scripts/quest_keep_awake.sh" on
QUEST_KEEP_AWAKE=1
echo "Quest proximity sensor overridden for this teleoperation session."

echo "Starting both wrist cameras and front/side D455..."
echo "Depth capture and recording: $RECORD_DEPTH"
ros2 launch quest3_robot_data realsense_four.launch.py \
  hardware_config:="$HARDWARE_CONFIG" \
  enable_depth:="$RECORD_DEPTH" &
CAMERA_PID=$!
echo "Waiting 4 seconds for RealSense initialization..."
sleep 4

echo "Quest: open http://localhost:8765, connect, then enter VR."
echo "After both controllers track, remove the headset and place it facing the controllers."
echo "Y/B toggles recording: rising tones=start, falling tones=stop, low tones=interrupted."
echo "Left controller -> left UR; right controller -> right UR."
echo "Hold a controller side-grip to move that arm; each trigger operates its gripper."
echo "Ctrl-C stops both servo loops and all nodes."
echo "Task label for every episode in this run: $TASK_DESCRIPTION"
ros2 launch quest3_robot_data quest3_system.launch.py \
  hardware_config:="$HARDWARE_CONFIG" \
  recorder_config:="$RECORDER_CONFIG" \
  output_dir:="$OUTPUT_DIR" \
  task_description:="$TASK_DESCRIPTION" \
  record_depth:="$RECORD_DEPTH" \
  front_topic:=/front_camera/camera/color/image_raw &
SYSTEM_PID=$!
wait "$SYSTEM_PID"
