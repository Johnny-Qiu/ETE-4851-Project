#!/bin/bash
# Stop everything start_all.sh started (dashboard first, so motors stop before sensors go away).
#   bash ~/ETE-4851-Project/johnny/stop_all.sh
# Also catches copies started by hand in other terminals.
LOGS="$HOME/logs"

stop_one() {  # name pattern
  local name=$1 pattern=$2 pid
  if [ -f "$LOGS/$name.pid" ]; then
    pid=$(cat "$LOGS/$name.pid")
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null          # Ctrl+C to the whole group
      for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
      kill -0 "$pid" 2>/dev/null && kill -TERM -- "-$pid" 2>/dev/null && sleep 1
      kill -0 "$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null
    fi
    rm -f "$LOGS/$name.pid"
  fi
  if pgrep -f "$pattern" >/dev/null; then       # leftovers / started by hand
    pkill -INT -f "$pattern"; sleep 2
    pkill -TERM -f "$pattern" 2>/dev/null; sleep 0.5
    pkill -KILL -f "$pattern" 2>/dev/null
  fi
  if pgrep -f "$pattern" >/dev/null; then echo "$name: STILL RUNNING"; else echo "$name: stopped"; fi
}

stop_one dashboard drive_dashboard.py
stop_one lidar     sllidar_node
stop_one cameras   usb_cam_node_exe
