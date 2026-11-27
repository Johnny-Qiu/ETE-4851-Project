#!/bin/bash
# Both NexiGo N60 cameras for computer vision (johnny/dev). Original: "Updated Files/start_cameras_640.sh".
# Defaults: raw YUYV 640x480 @30, manual exposure, brightness 128. Override with env vars, e.g.
#   FPS=20 bash start_cameras_fast.sh
#   EXPOSURE=auto GAIN=5 bash start_cameras_fast.sh
# FORMAT=yuyv publishes yuv422_yuy2 without converting (cheapest; cv_bridge converts on demand).
# Do NOT use mjpeg2rgb with usb_cam 0.8.1: half of every frame came out green/corrupted on the Pi.
# yuyv2rgb works but cost ~75% of a core per camera vs ~4% for yuyv (measured 2026-11-27).
# EXPOSURE is in 100 us units (100 = 10 ms; must stay below 1/FPS for full frame rate) or "auto".
# Topics are the same as the original script: /camera/image_raw and /camera/rear/image_raw.
WIDTH=${WIDTH:-640}; HEIGHT=${HEIGHT:-480}; FPS=${FPS:-30}
FORMAT=${FORMAT:-yuyv}               # yuyv (raw, cheapest) | yuyv2rgb | mjpeg2rgb (broken here, see above)
EXPOSURE=${EXPOSURE:-100}; GAIN=${GAIN:-30}; BRIGHTNESS=${BRIGHTNESS:-128}
case "$FPS" in *.*) FPS_F=$FPS ;; *) FPS_F=$FPS.0 ;; esac   # usb_cam wants a double
FRONT=/dev/v4l/by-id/usb-webcamvendor_NexiGo_N60_FHD_Webcam_Audio_Sep_8_2023-18:48:27-N60-video-index0
REAR=/dev/v4l/by-id/usb-webcamvendor_NexiGo_N60_FHD_Webcam_Audio_Apr_21_2023-20:00:20-N60-video-index0

source /opt/ros/lyrical/setup.bash
source ~/camera_ws/install/setup.bash

apply_controls() {  # device
  if [ "$EXPOSURE" = auto ]; then
    v4l2-ctl -d "$1" -c auto_exposure=0 -c gain="$GAIN" -c brightness="$BRIGHTNESS"
  else
    v4l2-ctl -d "$1" -c auto_exposure=1 && v4l2-ctl -d "$1" -c exposure_time_absolute="$EXPOSURE" -c gain="$GAIN" -c brightness="$BRIGHTNESS"
  fi
  echo "$(basename "$1" | cut -d_ -f6-7): $(v4l2-ctl -d "$1" -C auto_exposure,exposure_time_absolute,gain,brightness | tr '\n' ' ')"
}

PIDS=()
start_cam() {  # namespace device
  if [ ! -e "$2" ]; then echo "WARNING: $1 camera not connected ($2) - skipped"; return; fi
  ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:="$1" -p video_device:="$2" \
    -p image_width:="$WIDTH" -p image_height:="$HEIGHT" -p framerate:="$FPS_F" \
    -p pixel_format:="$FORMAT" -p brightness:="$BRIGHTNESS" &
  PIDS+=($!)
}
echo "Cameras: $FORMAT ${WIDTH}x${HEIGHT} @${FPS}, exposure=$EXPOSURE gain=$GAIN brightness=$BRIGHTNESS"
start_cam /camera "$FRONT"
start_cam /camera/rear "$REAR"
[ ${#PIDS[@]} -eq 0 ] && { echo "No cameras found"; exit 1; }
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait' INT TERM
sleep 4   # let usb_cam open the devices, then set controls it may have reset
for d in "$FRONT" "$REAR"; do [ -e "$d" ] && apply_controls "$d"; done
wait
