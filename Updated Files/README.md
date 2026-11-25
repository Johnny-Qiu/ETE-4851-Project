# Updated Files — November 23, 2026 sensor and WASD setup

Packaged November 24, 2026. This folder collects the latest local files for the
combined two-camera, LiDAR, BNO055 IMU, shared INA219 current sensor and manual
WASD dashboard used on the Raspberry Pi 5 / NUCLEO-F767ZI.

## Files

| Path | Purpose |
| --- | --- |
| sensor_viewer/all_sensors.py | Combined browser page, ROS subscriptions, sensor telemetry and optional WASD server |
| wasd_demo/dashboard.py | Serial bridge, browser ownership, drive watchdog and smooth steering |
| wasd_demo/vehicle_control.py | Independent motor/servo settings and protocol v5 commands |
| wasd_demo/motor_base.py | Base control state machine, CRC and serial protocol support |
| wasd_demo/encoder_monitor.py | Encoder monitoring and stall checks |
| wasd_demo/vehicle.html | Companion standalone WASD page used by dashboard.py |
| wasd_demo/current_readings.py, diagnose_vehicle.py | Supporting current and firmware diagnostic tools |
| start_cameras_640.sh | Both NexiGo cameras, YUYV 640 x 480 at 15 FPS, brightness 255 |
| start_lidar.sh | C1 driver using persistent USB path at 460800 baud |
| start_dashboard.sh | Launch wrapper for sensor monitoring or integrated WASD |
| calibration/ | Last-known sensor settings and provenance limitations |
| firmware/ | Available Nucleo binaries, models, custom sources and upload scripts; read its README first |
| VALIDATION.md | Checks performed for this package |
| SOURCE_FILES.md | Mapping to the original local source files |
| MANIFEST_SHA256.txt | SHA-256 of every packaged file except the manifest itself, as stored in Git |

## Dependencies on the Pi

The existing machine used Ubuntu 26.04, ROS 2 Lyrical, Python 3, pyserial,
OpenCV, rclpy, cv_bridge and sensor_msgs, plus usb_cam from ~/camera_ws and
sllidar_ros2 from ~/lidar_ws. These system libraries and installed ROS driver
workspaces are external prerequisites; their binaries and full source trees
are not included here. Their exact installed revisions could not be captured
because the Pi was offline during packaging. This folder is a project software
backup, not a Pi operating-system image.

Check that the packages are available in a Pi terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/camera_ws/install/setup.bash
source ~/lidar_ws/install/setup.bash
python3 -c 'import rclpy, serial, cv2, cv_bridge, sensor_msgs'
ros2 pkg prefix usb_cam
ros2 pkg prefix sllidar_ros2
ls -l /dev/v4l/by-id/ /dev/serial/by-id/
```

The device IDs in the scripts identify the original hardware. Edit them if
using replacement devices. Confirm which physical camera is front/rear.
The Pi's existing ~/robot_nav_calibration.json is retained; see calibration/.
The higher USB current setting was enabled in the working Pi configuration;
that boot configuration has not been copied or changed by this package.

## Start the software

Place this entire folder on the Pi, for example at ~/Updated Files. Stop the
previous dashboard/serial controller and duplicate camera/LiDAR drivers first.
Only one process may own the Nucleo serial port. Explorer is not needed.

Open three Pi terminals, each with this folder as its current directory:

```bash
cd "$HOME/Updated Files"
bash start_cameras_640.sh
```

```bash
cd "$HOME/Updated Files"
bash start_lidar.sh
```

```bash
cd "$HOME/Updated Files"
bash start_dashboard.sh --drive
```

For receive-only sensor monitoring, omit --drive. The wrapper also accepts
dashboard options such as --http-port 8006, --port DEVICE and --shunt-ohms VALUE.
It uses the packaged wasd_demo folder, equivalent to yesterday's ~/sensor_drive.

From Windows PowerShell, replace PI_ADDRESS with the Pi's current IP:

```powershell
ssh -L 8006:127.0.0.1:8006 swerve@PI_ADDRESS
```

Keep the SSH connection open. Open the full token URL printed by the dashboard
in the Pi terminal. The server listens on 127.0.0.1:8006. A fresh private token
is generated at startup; no saved access token or SSH key is included here.
Use one driving browser tab. Ctrl+C stops each foreground launch script.

## Driving behavior

- Starts disarmed, with the power selector at 25%.
- Arm at zero, then W/S for drive and A/D for steering.
- Releasing W/S requests zero motor output while remaining armed.
- Releasing A/D holds steering. The integrated mode uses the faster steering
  response with a 0.06 s smoothing time constant and 0.02 s serial interval.
- STOP, Space, Escape, page/focus loss, connection loss and faults disarm.
- LiDAR is a display only; WASD has no automatic obstacle stop.

The single INA219 measures the combined motor/servo supply. Current accuracy,
IMU mounting/calibration, camera placement and LiDAR alignment still require
physical checks. Software tests do not establish physical driving performance.

## Recheck the package locally

From this folder (no ROS or vehicle required):

```powershell
python -m unittest discover -s wasd_demo -p 'test_*.py' -v
python -m unittest discover -s sensor_viewer -p 'test_*.py' -v
node --check sensor_viewer/sensor_page_check.js
node sensor_viewer/test_dashboard_drive.cjs
node wasd_demo/test_keyboard.cjs
```
