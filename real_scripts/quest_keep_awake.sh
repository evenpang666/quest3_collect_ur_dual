#!/usr/bin/env bash
# Keep Quest awake while it is off the operator's face; restore on exit.
set -euo pipefail

case "${1:-}" in
  on) action=com.oculus.vrpowermanager.prox_close ;;
  off) action=com.oculus.vrpowermanager.automation_disable ;;
  *) echo "Usage: $0 on|off" >&2; exit 2 ;;
esac
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 on|off" >&2
  exit 2
fi

adb shell am broadcast -a "$action"
