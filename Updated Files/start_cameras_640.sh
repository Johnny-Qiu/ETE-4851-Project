#!/bin/bash
source /opt/ros/lyrical/setup.bash
source ~/camera_ws/install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera -p video_device:=/dev/v4l/by-id/usb-webcamvendor_NexiGo_N60_FHD_Webcam_Audio_Sep_8_2023-18:48:27-N60-video-index0 -p image_width:=640 -p image_height:=480 -p framerate:=15.0 -p pixel_format:=yuyv -p brightness:=255 &
FRONT_PID=$!
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera/rear -p video_device:=/dev/v4l/by-id/usb-webcamvendor_NexiGo_N60_FHD_Webcam_Audio_Apr_21_2023-20:00:20-N60-video-index0 -p image_width:=640 -p image_height:=480 -p framerate:=15.0 -p pixel_format:=yuyv -p brightness:=255 &
REAR_PID=$!
trap 'kill -INT "$FRONT_PID" "$REAR_PID" 2>/dev/null; wait' INT TERM
wait
