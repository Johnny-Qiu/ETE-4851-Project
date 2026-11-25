#!/bin/bash
set -e
source /opt/ros/lyrical/setup.bash
source "$HOME/lidar_ws/install/setup.bash"
exec ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_3466a3ceba31f111b2db8d5f30d20014-if00-port0 serial_baudrate:=460800
