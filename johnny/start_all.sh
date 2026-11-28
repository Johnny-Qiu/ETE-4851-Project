#!/bin/bash
# Start cameras, LiDAR and the drive dashboard detached from SSH (survives disconnects).
#   bash ~/ETE-4851-Project/johnny/start_all.sh             live motors (arm from the page)
#   bash ~/ETE-4851-Project/johnny/start_all.sh --demo      simulated board
#   bash ~/ETE-4851-Project/johnny/start_all.sh --sensors   receive-only, no drive card
# Extra options go to the dashboard, e.g. --http-port 8010.
# Skip parts with NO_CAMERAS=1 or NO_LIDAR=1. Logs and PIDs: ~/logs. Stop with stop_all.sh.
# Connect from Windows first: ssh -L 8006:127.0.0.1:8006 swerve@PI_ADDRESS
R="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOGS="$HOME/logs"; mkdir -p "$LOGS"

running() {  # name pattern -> true if already up
  local pid
  if [ -f "$LOGS/$1.pid" ]; then
    pid=$(cat "$LOGS/$1.pid")
    kill -0 "$pid" 2>/dev/null && return 0
  fi
  pgrep -f "$2" >/dev/null
}

launch() {  # name pattern script [args...]
  local name=$1 pattern=$2; shift 2
  if running "$name" "$pattern"; then
    echo "$name: already running (stop_all.sh first to restart)"; return
  fi
  if [ ! -f "$1" ]; then echo "$name: MISSING $1"; return; fi
  # Background setsid in a script does not fork, so $! is the new session/group leader.
  setsid bash "$@" > "$LOGS/$name.log" 2>&1 < /dev/null &
  echo $! > "$LOGS/$name.pid"
  echo "$name: started (pid $!, log $LOGS/$name.log)"
}

[ -z "$NO_CAMERAS" ] && launch cameras usb_cam_node_exe "$R/johnny/cameras/start_cameras_fast.sh"
[ -z "$NO_LIDAR" ]   && launch lidar   sllidar_node     "$R/Updated Files/start_lidar.sh"
DASH_STARTED=0
if running dashboard drive_dashboard.py; then
  echo "dashboard: already running (stop_all.sh first to restart)"
else
  launch dashboard drive_dashboard.py "$R/johnny/drive_dashboard/start_drive_dashboard.sh" "$@"
  DASH_STARTED=1
fi

# Print the dashboard link (new token every dashboard start).
for _ in $(seq 1 30); do
  URL=$(grep -o 'http://127.0.0.1:[0-9]*/#token=[^ ]*' "$LOGS/dashboard.log" 2>/dev/null | tail -n 1)
  [ -n "$URL" ] && break
  [ "$DASH_STARTED" = 1 ] && ! kill -0 "$(cat "$LOGS/dashboard.pid")" 2>/dev/null && break
  sleep 1
done
echo
if [ -n "$URL" ]; then
  echo "Open in your browser (through the SSH tunnel):"
  echo "  $URL"
  echo "Setup resets on restart: motor1 -1, motor2 +1, front +1, rear -1, travel 430."
else
  echo "No dashboard link yet. Last lines of $LOGS/dashboard.log:"
  tail -n 15 "$LOGS/dashboard.log" 2>/dev/null
fi
sleep 2
for n in cameras lidar; do
  if [ -f "$LOGS/$n.pid" ] && ! kill -0 "$(cat "$LOGS/$n.pid")" 2>/dev/null; then
    echo "WARNING: $n exited. Last lines of $LOGS/$n.log:"; tail -n 10 "$LOGS/$n.log"
  fi
done
