#!/bin/bash
# Measure delivered ROS frame rate and Pi load while the cameras run (johnny/dev). Read-only.
#   bash measure_cameras.sh [seconds]
SECS=${1:-15}
source /opt/ros/lyrical/setup.bash
source ~/camera_ws/install/setup.bash 2>/dev/null
echo "== ROS topic rate over ${SECS}s"
for t in /camera/image_raw /camera/rear/image_raw; do
  ( r=$(timeout -s INT -k 3 "$SECS" ros2 topic hz "$t" 2>/dev/null < /dev/null | grep 'average rate' | tail -1); echo "$t: ${r:-no messages}" ) &
done
# CPU sampled over the same window, while hz runs
read -r _ u1 n1 s1 i1 w1 q1 sq1 _ < /proc/stat; sleep "$SECS"; read -r _ u2 n2 s2 i2 w2 q2 sq2 _ < /proc/stat
wait
busy=$(( (u2+n2+s2+q2+sq2) - (u1+n1+s1+q1+sq1) )); idle=$(( (i2+w2) - (i1+w1) ))
echo "== Whole Pi CPU busy: $(( 100*busy/(busy+idle) ))% of 4 cores (includes the hz measurement itself)"
echo "== Per process (% of one core)"
ps -eo pcpu,rss,args --sort=-pcpu | grep -E '[u]sb_cam|[d]rive_dashboard|[a]ll_sensors|[s]llidar' | awk '{printf "%5s%%  %4d MB  %s\n",$1,$2/1024,substr($0,index($0,$3),70)}'
echo "== Temperature / throttling"
awk '{printf "SoC temp %.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp
for f in /sys/devices/platform/soc/soc:firmware/get_throttled /sys/devices/platform/*/get_throttled; do [ -r "$f" ] && echo "throttled flags: $(cat "$f")" && break; done
echo "== USB errors since boot (uvcvideo)"
dmesg 2>/dev/null | grep -iE 'uvcvideo.*(fail|error)|no space left' | tail -3 || true
