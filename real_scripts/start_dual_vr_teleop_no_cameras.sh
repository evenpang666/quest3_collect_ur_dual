#!/usr/bin/env bash
# Run the dual-arm Quest teleoperator alongside a separately running safety service.
# This script starts no RealSense pipeline and does not record episodes.
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HARDWARE_CONFIG="$PROJECT_DIR/config/hardware_dual_teleop.yaml"
RECORDER_CONFIG="$PROJECT_DIR/config/dual_arm.yaml"
CONFIRMATION="MOVE_BOTH_ARMS"
SAFETY_PYTHON="${SAFETY_PYTHON:-/home/mypc/anaconda3/envs/safety/bin/python}"
AVOIDANCE_PORT="${QUEST3_AVOIDANCE_PORT:-8768}"

if [[ $# -ne 2 || "${1:-}" != "--confirm" || "${2:-}" != "$CONFIRMATION" ]]; then
  cat <<EOF
This starts REAL motion for BOTH UR7e arms without starting cameras or recording.
The separately running safety detector must be available on localhost:8766.
Right-controller A toggles QP-CBF obstacle avoidance.
Before continuing: clear both workspaces, make E-stops reachable, set both URs
to Remote Control / running, and verify each pendant's active TCP.

Run exactly:
  $0 --confirm $CONFIRMATION
EOF
  exit 2
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "ERROR: leave Conda first (conda deactivate), then rerun this script." >&2
  exit 2
fi

# The camera/recording launch uses the same robots and Quest bridge.
exec {TELEOP_LOCK_FD}>/tmp/quest3_dual_vr_teleop.lock
if ! flock -n "$TELEOP_LOCK_FD"; then
  echo "ERROR: another dual-arm Quest teleoperation script is already running." >&2
  exit 2
fi

source "$PROJECT_DIR/real_scripts/ros_environment.sh"
set -u
# The installed ROS package may predate this script. Use the checked-out
# Python modules and launch file so its no-camera flags work without a rebuild.
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
export QUEST3_ENABLE_AVOIDANCE=1
if [[ ! "$AVOIDANCE_PORT" =~ ^[0-9]+$ ]] || ((10#$AVOIDANCE_PORT < 1 || 10#$AVOIDANCE_PORT > 65535)); then
  echo "ERROR: QUEST3_AVOIDANCE_PORT must be a TCP port from 1 to 65535." >&2
  exit 2
fi
export QUEST3_AVOIDANCE_RUNTIME_URL="http://127.0.0.1:$AVOIDANCE_PORT"

worker_ready() {
  /usr/bin/python3 -c 'from urllib.request import urlopen; import sys; urlopen(sys.argv[1] + "/health", timeout=0.3).close()' \
    "$QUEST3_AVOIDANCE_RUNTIME_URL" >/dev/null 2>&1
}

if [[ ! -x "$SAFETY_PYTHON" ]]; then
  echo "ERROR: safety Python is unavailable: $SAFETY_PYTHON" >&2
  exit 2
fi
if ! /usr/bin/python3 -c 'from urllib.request import urlopen; urlopen("http://127.0.0.1:8766/snapshot.json?compact=1", timeout=2).close()' >/dev/null 2>&1; then
  echo "ERROR: start the safety live_obstacle_detection service on 127.0.0.1:8766 first." >&2
  exit 2
fi
if worker_ready; then
  echo "ERROR: an avoidance worker is already using $QUEST3_AVOIDANCE_RUNTIME_URL; choose another QUEST3_AVOIDANCE_PORT." >&2
  exit 2
fi

# Check robot/Quest connectivity and calibration without querying the cameras
# already owned by the independent safety process.
/usr/bin/python3 -m quest3_robot_data.preflight --config "$HARDWARE_CONFIG" --skip-camera-check
adb reverse tcp:8765 tcp:8765

cleanup() {
  trap - EXIT
  trap '' INT TERM
  local task_pid attempt running
  echo "Stopping teleoperation and its avoidance worker."
  for task_pid in "${SYSTEM_PID:-}" "${SAFETY_PID:-}"; do
    [[ -n "$task_pid" ]] && kill -TERM "$task_pid" 2>/dev/null || true
  done
  for ((attempt=0; attempt<100; attempt++)); do
    running=false
    for task_pid in "${SYSTEM_PID:-}" "${SAFETY_PID:-}"; do
      if [[ -n "$task_pid" ]] && kill -0 "$task_pid" 2>/dev/null; then
        running=true
      fi
    done
    [[ "$running" == false ]] && break
    sleep 0.1
  done
  for task_pid in "${SYSTEM_PID:-}" "${SAFETY_PID:-}"; do
    [[ -n "$task_pid" ]] && kill -KILL "$task_pid" 2>/dev/null || true
  done
  for task_pid in "${SYSTEM_PID:-}" "${SAFETY_PID:-}"; do
    [[ -n "$task_pid" ]] && wait "$task_pid" 2>/dev/null || true
  done
  if [[ "${QUEST_KEEP_AWAKE:-0}" == 1 ]]; then
    bash "$PROJECT_DIR/real_scripts/quest_keep_awake.sh" off || \
      echo "WARNING: could not restore Quest proximity sensor; run quest_keep_awake.sh off." >&2
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

bash "$PROJECT_DIR/real_scripts/quest_keep_awake.sh" on
QUEST_KEEP_AWAKE=1
echo "Quest proximity sensor overridden for this teleoperation session."

QUEST3_AVOIDANCE_PARENT_PID="$$" PYTHONPATH="$PROJECT_DIR" "$SAFETY_PYTHON" \
  -m quest3_robot_data.teleop_avoidance_server --config "$HARDWARE_CONFIG" --port "$AVOIDANCE_PORT" &
SAFETY_PID=$!
SAFETY_READY=false
for ((attempt=0; attempt<60; attempt++)); do
  if ! kill -0 "$SAFETY_PID" 2>/dev/null; then
    echo "ERROR: safety avoidance worker exited before becoming ready." >&2
    exit 2
  fi
  if worker_ready; then
    SAFETY_READY=true
    break
  fi
  sleep 0.5
done
if [[ "$SAFETY_READY" != true ]]; then
  echo "ERROR: safety avoidance worker did not become ready on $QUEST3_AVOIDANCE_RUNTIME_URL." >&2
  exit 2
fi

echo "Starting dual-arm Quest teleoperation without camera nodes or episode recording."
echo "Quest: open http://localhost:8765, connect, then enter VR."
echo "After both controllers track, remove the headset and place it facing the controllers."
echo "Right-controller A toggles avoidance; failed checks hold that action and retry while avoidance stays on."
ros2 launch "$PROJECT_DIR/launch/quest3_system.launch.py" \
  hardware_config:="$HARDWARE_CONFIG" \
  recorder_config:="$RECORDER_CONFIG" \
  output_dir:="$PROJECT_DIR/data/dual_vr_episodes" \
  task_description:=teleop_only \
  start_recorder:=false &
SYSTEM_PID=$!
wait "$SYSTEM_PID"
