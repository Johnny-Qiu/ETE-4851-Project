"""Combined sensor dashboard + manual WASD driving (johnny/dev).

Based on "Updated Files/sensor_viewer/all_sensors.py". Motor control, arming,
watchdog and encoder stall stop are the unchanged Bridge from
"Updated Files/wasd_demo/dashboard.py"; this file only replaces the browser
page and adds the 'steer' action (Center steering) to the allowed commands.

Default is receive-only. --enable-drive starts the WASD controller DISARMED.
--demo uses the Bridge's simulated board and never opens the Nucleo port.
"""
import argparse
import json
import math
from pathlib import Path
import secrets
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

PACKAGE = Path(__file__).resolve().parents[2] / 'Updated Files'
DRIVE_ACTIONS = ('arm', 'stop', 'keys', 'heartbeat', 'configure', 'steer')

# Same devices as johnny/cameras/start_cameras_fast.sh. Controls are changed live with v4l2-ctl
# while usb_cam streams; they reset when the camera is unplugged or the camera script restarts.
CAMERAS = {
    'front': '/dev/v4l/by-id/usb-webcamvendor_NexiGo_N60_FHD_Webcam_Audio_Sep_8_2023-18:48:27-N60-video-index0',
    'rear': '/dev/v4l/by-id/usb-webcamvendor_NexiGo_N60_FHD_Webcam_Audio_Apr_21_2023-20:00:20-N60-video-index0',
}
# Exposure is in 100 us units. 330 = 33 ms, just under one frame at 30 fps, so manual exposure
# can never lower the frame rate. Auto exposure has no such cap (it dropped to ~25 fps in room light).
CAMERA_LIMITS = dict(exposure=(1, 330), gain=(0, 100), brightness=(0, 255))

# PREVIEW BOOST (2026-11-27, temporary). Display only: ROS topics are unchanged (cameras 30 fps,
# /scan 10 Hz, full range). To revert: CAMERA_PREVIEW_INTERVAL_S = .2 (5 fps) and
# LIDAR_DRAW_MAX_M = 4.5, plus the PREVIEW BOOST constants in page.html.
CAMERA_PREVIEW_INTERVAL_S = .1   # was .2: 10 fps JPEG preview per camera
LIDAR_DRAW_MAX_M = None          # was 4.5; None = everything up to the sensor's range_max


def read_camera(device):
    if not Path(device).exists():
        return None
    out = subprocess.run(['v4l2-ctl', '-d', device, '-C', 'auto_exposure,exposure_time_absolute,gain,brightness'],
                         capture_output=True, text=True, timeout=2)
    if out.returncode:
        return None
    values = {}
    for line in out.stdout.splitlines():
        key, _, rest = line.partition(':')
        if rest.split():
            values[key.strip()] = int(rest.split()[0])
    return dict(auto=values.get('auto_exposure') != 1, exposure=values.get('exposure_time_absolute'),
                gain=values.get('gain'), brightness=values.get('brightness'))


def set_camera(data):
    name = data.get('camera')
    if name not in CAMERAS:
        raise ValueError('Unknown camera')
    if not isinstance(data.get('auto'), bool):
        raise ValueError('auto must be true or false')
    for key, (low, high) in CAMERA_LIMITS.items():
        v = data.get(key)
        if type(v) is not int or not low <= v <= high:
            raise ValueError(f'{key} must be a whole number {low}..{high}')
    device = CAMERAS[name]
    if not Path(device).exists():
        raise ValueError(f'{name} camera is not connected')
    base = ['v4l2-ctl', '-d', device]
    steps = ([base + ['-c', 'auto_exposure=0']] if data['auto'] else
             [base + ['-c', 'auto_exposure=1'], base + ['-c', f"exposure_time_absolute={data['exposure']}"]])
    steps.append(base + ['-c', f"gain={data['gain']}", '-c', f"brightness={data['brightness']}"])
    for cmd in steps:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        if out.returncode:
            raise ValueError('v4l2-ctl: ' + (out.stderr.strip() or 'failed'))
    return read_camera(device)

PAGE = (Path(__file__).with_name('page.html')).read_text(encoding='utf-8')


def board_values(fields, fresh, shunt):
    result = dict(amps=None, volts=None, watts=None, status='No fresh board data')
    if not fresh:
        return result
    result['status'] = 'Sensor unavailable or range exceeded'
    try:
        if fields.get('INA') == '1' and fields.get('ST1') == '1':
            amps = int(fields['SH1']) * 0.00001 / shunt
            volts = int(fields['BV1']) * 0.001
            result.update(amps=amps, volts=volts, watts=amps*volts, status='Live · shared supply sensor')
    except (ValueError, KeyError):
        pass
    return result


def apply_drive_command(drive, data, sequences):
    sequence, client = data.get('sequence'), data.get('client')
    if type(sequence) is not int or not 0 < sequence < 2**53 or not isinstance(client, str) or not 8 <= len(client) <= 100:
        raise ValueError('Invalid command sequence or browser identifier')
    with drive.lock:
        previous = sequences.get(client, 0)
        if data['action'] != 'stop' and sequence <= previous:
            raise ValueError('Out-of-order control command ignored')
        if client not in sequences and len(sequences) >= 128:
            raise ValueError('Too many browser sessions; reload the server')
        sequences[client] = max(sequence, previous)
        drive.command(data)


def main():
    import rclpy
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, LaserScan
    from rclpy.qos import qos_profile_sensor_data

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF575251717867104619-if02')
    parser.add_argument('--http-port', type=int, default=8006)
    parser.add_argument('--shunt-ohms', type=float, default=None)
    parser.add_argument('--enable-drive', action='store_true', help='Enable the WASD controller; starts disarmed')
    parser.add_argument('--demo', action='store_true', help='Simulated board (implies --enable-drive); never opens the Nucleo port')
    parser.add_argument('--drive-dir', default=str(PACKAGE / 'wasd_demo'))
    args = parser.parse_args()
    if args.demo:
        args.enable_drive = True
    cfg = {}
    try:
        cfg = json.loads((Path.home()/'robot_nav_calibration.json').read_text())
    except (OSError, ValueError):
        pass
    shunt = args.shunt_ohms if args.shunt_ohms is not None else float(cfg.get('ina_shunt_ohms', .1))
    if not math.isfinite(shunt) or shunt <= 0:
        parser.error('Shunt resistance must be positive and finite')
    lock = threading.Lock()
    stop = threading.Event()
    shared = dict(fields={}, board_at=0., serial_error='Waiting for board', scan=None, cameras={}, camctl={})
    token = secrets.token_urlsafe(32)
    drive = None
    drive_worker = None
    drive_sequences = {}
    rclpy.init()
    node = rclpy.create_node('car_dashboard_drive')
    bridge = CvBridge()

    def image_cb(msg, name):
        now = time.monotonic()
        with lock:
            previous = shared['cameras'].get(name)
        if previous and now-previous[0] < CAMERA_PREVIEW_INTERVAL_S:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not -.1 <= node.get_clock().now().nanoseconds/1e9-stamp <= 1.:
            return
        try:
            ok, jpeg = cv2.imencode('.jpg', bridge.imgmsg_to_cv2(msg, 'bgr8'), [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with lock:
                    shared['cameras'][name] = (now, jpeg.tobytes())
        except Exception as exc:
            node.get_logger().warning('Camera conversion: '+str(exc))

    def scan_cb(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not -.1 <= node.get_clock().now().nanoseconds/1e9-stamp <= .6:
            return
        mounted = cfg.get('laser_frame') == msg.header.frame_id and 'laser_yaw_rad' in cfg
        yaw = float(cfg['laser_yaw_rad']) if mounted else 0.
        points = []
        for i in range(0, len(msg.ranges), max(1, math.ceil(len(msg.ranges)/720))):
            radius = msg.ranges[i]
            if math.isfinite(radius) and msg.range_min < radius < min(msg.range_max, LIDAR_DRAW_MAX_M or msg.range_max):
                angle = msg.angle_min+i*msg.angle_increment+yaw
                points.append([radius*math.cos(angle), radius*math.sin(angle)])
        with lock:
            shared['scan'] = (time.monotonic(), points, 'Saved mounting yaw · forward up' if mounted else 'Raw laser frame · +X up')

    subs = [node.create_subscription(Image, '/camera/image_raw', lambda m: image_cb(m, 'front'), qos_profile_sensor_data),
            node.create_subscription(Image, '/camera/rear/image_raw', lambda m: image_cb(m, 'rear'), qos_profile_sensor_data),
            node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)]

    def receive_board():
        import serial
        while not stop.is_set():
            try:
                with serial.Serial(args.port, 115200, timeout=.3, exclusive=True) as link:
                    while not stop.is_set():
                        line = link.readline(2048).decode('ascii', errors='replace').strip()
                        if not line.startswith('H='):
                            continue
                        fields = dict(item.split('=', 1) for item in line.split() if '=' in item)
                        with lock:
                            shared.update(fields=fields, board_at=time.monotonic(), serial_error='')
            except (serial.SerialException, OSError) as exc:
                with lock:
                    shared['serial_error'] = 'Board connection: '+str(exc)+'. Stop WASD/navigation before opening this viewer.'
                stop.wait(2.)

    def poll_camera_controls():
        while not stop.is_set():
            values = {}
            for name, device in CAMERAS.items():
                try:
                    values[name] = read_camera(device)
                except (OSError, subprocess.SubprocessError, ValueError):
                    values[name] = None
            with lock:
                shared['camctl'] = values
            stop.wait(2.)
    threading.Thread(target=poll_camera_controls, daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def drive_access(self):
            host = self.headers.get('Host', '')
            return (bool(re.fullmatch(r'(?:127\.0\.0\.1|localhost)(?::[0-9]{1,5})?', host))
                    and self.headers.get('Origin', 'http://'+host) == 'http://'+host
                    and secrets.compare_digest(self.headers.get('X-Sensor-Token', ''), token))

        def send(self, code, mime, payload):
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            data = None
            if self.path == '/camera':
                return self.camera_post()
            if self.path != '/drive' or drive is None or not self.drive_access():
                code, value = 403, {'error': 'Drive access denied'}
            else:
                try:
                    self.connection.settimeout(1.)
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 2048:
                        raise ValueError('Invalid request size')
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict) or data.get('action') not in DRIVE_ACTIONS:
                        raise ValueError('Invalid drive command')
                    apply_drive_command(drive, data, drive_sequences)
                    code, value = 200, drive.snapshot()
                except (ValueError, TypeError, OSError) as exc:
                    code, value = 400, {'error': str(exc)}
            # Log everything except the high-rate key/heartbeat traffic (no token is logged).
            action = data.get('action') if isinstance(data, dict) else None
            if action not in ('keys', 'heartbeat') or code != 200:
                detail = {k: v for k, v in data.items() if k not in ('client', 'sequence')} if isinstance(data, dict) else {}
                print(time.strftime('%H:%M:%S'), 'POST /drive', code, json.dumps(detail),
                      value.get('error', '') if code != 200 else 'settings=' + json.dumps(value.get('settings')), flush=True)
            self.send(code, 'application/json', json.dumps(value).encode())

        def camera_post(self):
            data = None
            if not self.drive_access():   # same host/origin/token check as driving
                code, value = 403, {'error': 'Camera access denied'}
            else:
                try:
                    self.connection.settimeout(1.)
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 512:
                        raise ValueError('Invalid request size')
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict):
                        raise ValueError('Invalid camera command')
                    readback = set_camera(data)
                    with lock:
                        shared['camctl'][data['camera']] = readback
                    code, value = 200, {'camera': data['camera'], 'controls': readback}
                except (ValueError, TypeError, OSError, subprocess.SubprocessError) as exc:
                    code, value = 400, {'error': str(exc)}
            print(time.strftime('%H:%M:%S'), 'POST /camera', code,
                  json.dumps(data) if isinstance(data, dict) else '', value.get('error', ''), flush=True)
            self.send(code, 'application/json', json.dumps(value).encode())

        def log_message(self, *unused):
            pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/':
                return self.send(200, 'text/html; charset=utf-8', PAGE.encode())
            if not secrets.compare_digest(self.headers.get('X-Sensor-Token', ''), token):
                return self.send(401, 'text/plain', b'Access denied')
            now = time.monotonic()
            with lock:
                if path == '/state':
                    fields = shared['fields'].copy()
                    fresh = bool(shared['board_at'] and now-shared['board_at'] < 1.5)
                    drive_status = drive.snapshot() if drive is not None else None
                    if drive_status is not None:
                        fields = drive_status['fields']
                        fresh = drive_status['fresh']
                    payload = json.dumps(dict(
                        drive=drive_status, fields=fields, board_fresh=fresh,
                        imu_valid=fresh and fields.get('VALID') == '1',
                        current=board_values(fields, fresh, shunt), shunt_ohms=shunt,
                        camctl=shared['camctl'], camera_limits=CAMERA_LIMITS,
                        serial_error=drive_status['error'] if drive_status else shared['serial_error'])).encode()
                    code, mime = 200, 'application/json'
                elif path == '/scan':  # polled on its own by the page (~10 Hz)
                    scan = shared['scan']
                    scan_fresh = bool(scan and now-scan[0] < .8)
                    payload = json.dumps(dict(fresh=scan_fresh, points=scan[1] if scan_fresh else [],
                                              frame_label=scan[2] if scan else 'Waiting for scan')).encode()
                    code, mime = 200, 'application/json'
                else:
                    frame = shared['cameras'].get(path[1:-4]) if path in ('/front.jpg', '/rear.jpg') else None
                    code, mime, payload = ((200, 'image/jpeg', frame[1]) if frame and now-frame[0] < 1.2
                                           else (503, 'text/plain', b'No fresh image'))
            self.send(code, mime, payload)

    server = ThreadingHTTPServer(('127.0.0.1', args.http_port), Handler)
    if args.enable_drive:
        sys.path.insert(0, str(Path(args.drive_dir).resolve()))
        from dashboard import Bridge
        try:
            drive = Bridge(port=None if args.demo else args.port, demo=args.demo,
                           command_interval=.02, steering_response_s=.06)
        except Exception:
            server.server_close()
            node.destroy_node()
            rclpy.try_shutdown()
            raise
        drive_worker = threading.Thread(target=drive.run, daemon=True)
        drive_worker.start()
    else:
        threading.Thread(target=receive_board, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    mode = 'DEMO (simulated motors)' if args.demo else 'DRIVE (starts disarmed)' if drive else 'SENSORS ONLY'
    print(f'{mode} http://127.0.0.1:{args.http_port}/#token={token}', flush=True)
    print('WASD enabled; starts disarmed. Key release stops movement while armed; leaving page stops and disarms. '
          'No automatic obstacle stopping.' if drive else
          'Receive-only Nucleo connection. Start camera and LiDAR drivers separately.', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if drive is not None:
            drive.closed.set()
            drive_worker.join(timeout=3.)
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
