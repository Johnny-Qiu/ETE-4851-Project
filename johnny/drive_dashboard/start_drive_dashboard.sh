#!/bin/bash
# Combined sensor + WASD dashboard (johnny/dev). Starts DISARMED.
#   bash start_drive_dashboard.sh            live motors (arm from the page)
#   bash start_drive_dashboard.sh --demo     simulated board, never opens the Nucleo port
#   bash start_drive_dashboard.sh --sensors  receive-only, no drive card
# Extra options pass through, e.g. --http-port 8010
set -e
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$HOME/camera_ws/install/setup.bash" ]; then
  source "$HOME/camera_ws/install/setup.bash"
fi
ARGS=(--enable-drive)
case "${1:-}" in
  --demo) ARGS=(--demo); shift ;;
  --sensors) ARGS=(); shift ;;
esac
exec python3 -u "$HERE/drive_dashboard.py" "${ARGS[@]}" "$@"
