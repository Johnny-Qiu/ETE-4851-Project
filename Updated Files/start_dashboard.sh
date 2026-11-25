#!/bin/bash
set -e
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$HOME/camera_ws/install/setup.bash" ]; then
  source "$HOME/camera_ws/install/setup.bash"
fi
DRIVE_ARGS=()
if [ "${1:-}" = "--drive" ]; then
  DRIVE_ARGS=(--enable-drive --drive-dir "$PACKAGE_DIR/wasd_demo")
  shift
fi
exec python3 -u "$PACKAGE_DIR/sensor_viewer/all_sensors.py" "${DRIVE_ARGS[@]}" "$@"
