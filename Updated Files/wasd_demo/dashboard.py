#!/usr/bin/env python3
"""Separate browser driving and four-wheel steering test; protocol v5 firmware required."""
from encoder_monitor import EncoderMonitor
import argparse
import json
import math
import re
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from vehicle_control import Control, MOTOR
BUILD = 'ENCODER-WASD-7'


class Bridge:
    def __init__(self, port=None, demo=False, sensors_only=False, disable_stall_stop=False,
                 command_interval=.1, steering_response_s=0.):
        if not .02 <= command_interval <= .1 or not 0 <= steering_response_s <= .2:
            raise ValueError('Invalid control timing')
        self.command_interval = command_interval
        self.steering_response_s = steering_response_s
        self.steer_velocity = 0.
        self.sensors_only = sensors_only
        self.encoder = EncoderMonitor(stop_on_stall=not disable_stall_stop)
        self.control = Control()
        self.lock = threading.RLock()
        self.owner = None
        self.lease = 0
        self.demo = demo
        self.error = ''
        self.message = 'Waiting for board telemetry'
        self.fields = {}
        self.held_keys = []
        self.last_tx = 'No packet sent yet'
        self.steer_rate = 0.0
        self.steer_updated = time.monotonic()
        self.serial = None
        self.closed = threading.Event()
        if not demo:
            import serial
            self.serial = serial.Serial(port, 115200, timeout=.02, write_timeout=.1, exclusive=True)
            self.serial.reset_input_buffer()

    def stop(self, message):
        self.steer_rate = 0.0
        self.steer_velocity = 0.0
        self.control.stop()
        self.owner = None
        self.message = message

    def advance_steering(self, now):
        elapsed = max(0, min(.1, now - self.steer_updated))
        self.steer_updated = now
        if self.control.state == 'armed':
            if self.steering_response_s:
                self.steer_velocity += (self.steer_rate-self.steer_velocity)*(1-math.exp(-elapsed/self.steering_response_s))
            else:
                self.steer_velocity = self.steer_rate
            if abs(self.steer_velocity) > .001:
                self.control.steer(max(-1, min(1, self.control.steering + elapsed*self.steer_velocity)))
        elif self.control.state != 'armed':
            self.steer_rate = 0.0
            self.steer_velocity = 0.0

    def expire(self, now):
        if self.owner and now - self.lease > 1.0:
            self.stop('Browser connection lost — stopped. Arm again to resume.')

    def command(self, data):
        with self.lock:
            now = time.monotonic()
            self.expire(now)
            action = data.get('action')
            client = data.get('client')
            if not isinstance(client, str) or not 8 <= len(client) <= 100:
                raise ValueError('Invalid browser identifier')
            if action == 'stop':
                self.stop('STOP requested — both motors disabled')
                return
            if self.error:
                raise ValueError(self.error)
            if action == 'heartbeat':
                if self.owner == client:
                    self.lease = now
                return
            if self.owner and self.owner != client:
                raise ValueError('Another browser controls the motors. Stop first to take control.')
            if action == 'keys':
                held, power, turn = data.get('keys'), data.get('power'), data.get('turn')
                if (not isinstance(held, list) or len(held)>4 or
                    any(type(k) is not str or k not in ('w','a','s','d') for k in held) or
                    type(power) not in (int,float) or not math.isfinite(power) or not 0<=power<=100 or
                    type(turn) not in (int,float) or not math.isfinite(turn) or not 0<=turn<=1):
                    raise ValueError('Invalid keyboard input')
                self.held_keys = held[:]
                if self.owner != client or self.control.state != 'armed':
                    raise ValueError('Arm from this browser first')
                self.advance_steering(now)
                self.control.drive((int('w' in held)-int('s' in held))*power)
                self.steer_rate = (int('d' in held)-int('a' in held))*turn*2.0
                self.lease = now
                self.message = 'Armed — independent throttle and steering active'
                return
            if action == 'arm' and self.sensors_only:
                raise ValueError('Encoder monitor mode: motor arming is disabled')
            if action == 'arm':
                if any(p is None for p in self.encoder.previous):
                    raise ValueError('Wait for valid encoder telemetry before arming')
                self.control.request_arm(now)
                self.encoder.acknowledge()
                self.owner = client
                self.lease = now
                self.message = 'Arming at zero duty…'
            elif action == 'input':
                if self.owner != client or self.control.state != 'armed':
                    raise ValueError('Arm from this browser first')
                throttle, steering = data.get('throttle'), data.get('steering')
                if (type(throttle) not in (int, float) or not math.isfinite(throttle) or abs(throttle)>100 or
                    (steering is not None and (type(steering) not in (int, float) or
                                              not math.isfinite(steering) or abs(steering)>1))):
                    raise ValueError('Invalid throttle or steering input')
                self.control.drive(throttle)
                if steering is not None:
                    self.steer_rate = 0.0
                    self.control.steer(steering)
                self.lease = now
                self.message = 'Driving' if throttle else 'Armed — ready for WASD'
            elif action == 'configure':
                self.control.configure(data)
                self.message = 'Direction settings updated. Verify with wheels raised.'
            elif action == 'drive':
                if self.owner != client:
                    raise ValueError('Arm from this browser first')
                self.control.drive(data.get('value'))
                self.message = 'Driving' if data.get('value') else 'Throttle zero'
                self.lease = now
            elif action == 'steer':
                if self.owner != client:
                    raise ValueError('Arm from this browser first')
                self.steer_rate = 0.0
                self.control.steer(data.get('value'))
                self.lease = now
            else:
                raise ValueError('Unknown command')

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            c = self.control
            age = None if c.last_telemetry is None else now - c.last_telemetry
            blocked = self.error
            if not blocked and (age is None or age > .5):
                blocked = 'Waiting for complete live board telemetry. Check Nucleo USB and the serial port.'
            if not blocked and any(p is None for p in self.encoder.previous):
                blocked = 'Waiting for valid encoder telemetry.'
            if not blocked and c.firmware_revision < 2:
                blocked = f'Board firmware is FW={c.firmware_revision}; this page needs FW=2. Upload the new Vehicle_Drive.bin using Upload_Vehicle.ps1.'
            if not blocked and c.state == 'disarmed' and not c.confirmed_disarmed:
                blocked = 'Waiting for the Nucleo to acknowledge DISARMED.'
            return dict(sensors_only=self.sensors_only, encoders=self.encoder.snapshot(age is not None and age <= .5 and not self.error), build=BUILD, blocked_reason=blocked or ('Encoder monitor mode — motor arming disabled' if self.sensors_only else ''), tx=self.last_tx, held_keys=self.held_keys[:], state=c.state, ready=c.state == 'disarmed' and not blocked and not self.sensors_only,
                        fresh=age is not None and age <= .5 and not self.error, age=age,
                        duty=[c.duty, c.duty2], directions=c.directions[:], settings=c.settings.copy(), steering=c.steering, pulses=c.pulses(), fields=self.fields.copy(),
                        raw=c.last_line, message=self.encoder.fault or self.encoder.warning or self.message, error=self.error, demo=self.demo, owner=self.owner)

    def receive(self, line, now):
        # Opening/resetting USB input can leave the first read mid-line.
        # A valid suffix alone is not evidence of firmware identity or state.
        if not line.startswith('H='):
            return
        if line.startswith('H=') and MOTOR.search(line) is None:
            raise RuntimeError('Incompatible firmware. Install Vehicle_Drive protocol v5 firmware.')
        if MOTOR.search(line):
            self.fields = dict(part.split('=', 1) for part in line.split() if '=' in part)
        fault = self.encoder.update(self.fields, now)
        if fault and self.control.state != 'disarmed':
            self.stop(fault)
        event = self.control.receive(line, now)
        if event:
            self.message = ('Armed — hold WASD to drive and steer. Release keys to stop; stays armed.' if self.control.state == 'armed'
                            else 'Disarmed. Ready to arm at zero duty.' if self.control.confirmed_disarmed else event)
            if self.control.state == 'disarmed':
                self.owner = None

    def run(self):
        buffer = bytearray()
        next_send = 0
        try:
            while not self.closed.is_set():
                with self.lock:
                    now = time.monotonic()
                    self.expire(now)
                    self.advance_steering(now)
                    event = self.control.check_link(now)
                    if event:
                        self.owner = None
                        self.message = event
                    if now >= next_send:
                        frame = self.control.next_frame()
                        self.last_tx = frame.hex()
                        if self.demo:
                            c = self.control
                            line = (f'H={180 + 25*math.sin(now/5):+.1f} R={5*math.sin(now):+.1f} P={3*math.cos(now):+.1f} '
                                    f'DRIVE=1 FW=2 S1={c.pulses()[0] if c.state != "disarmed" else 0:04d} '
                                    f'S2={c.pulses()[1] if c.state != "disarmed" else 0:04d} T2=20000 '
                                    f'CAL=3/3/3/3 VALID=1 ERR=000 E1={int(now*abs(c.duty))%65536:05d} '
                                    f'E2={int(now*abs(c.duty2))%65536:05d} ARM={int(c.state != "disarmed")} '
                                    f'D1={c.duty:+06.1f} D2={c.duty2:+06.1f} CMDERR=000')
                            self.receive(line, now)
                        elif self.serial.write(frame) != len(frame):
                            raise OSError('Incomplete serial command write')
                        next_send = now + self.command_interval
                if self.demo:
                    self.closed.wait(.01)
                else:
                    buffer.extend(self.serial.read(min(max(1, self.serial.in_waiting), 1024)))
                    if len(buffer) > 4096:
                        buffer.clear()
                    while b'\n' in buffer:
                        raw, _, buffer = buffer.partition(b'\n')
                        with self.lock:
                            self.receive(raw.decode('ascii', errors='replace').strip(), time.monotonic())
        except Exception as error:
            with self.lock:
                self.error = str(error)
                self.stop('Connection failed — stopped. Restart dashboard to reconnect.')
        finally:
            with self.lock:
                self.stop('Controller stopped')
                if self.serial:
                    for _ in range(3):
                        try:
                            self.serial.write(self.control.next_frame())
                            self.serial.flush()
                        except Exception:
                            break
                    self.serial.close()


def make_server(bridge, port, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Keep credentials and polling out of terminal logs.

        def reply(self, status, value, kind='application/json'):
            payload = value.encode() if isinstance(value, str) else json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind + '; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(payload)

        def access_error(self):
            host = self.headers.get('Host', '')
            # SSH forwards may use a different browser port from the Pi port.
            match = re.fullmatch(r'(?:127\.0\.0\.1|localhost)(?::([0-9]{1,5}))?', host)
            valid_host = bool(match) and (match.group(1) is None or 1 <= int(match.group(1)) <= 65535)
            origin = self.headers.get('Origin')
            if not valid_host:
                return 'Open this page at http://127.0.0.1 through the SSH tunnel. The current hostname is not allowed.'
            if origin is not None and origin != 'http://' + host:
                return 'Browser origin does not match this page. Open the HTTP link directly through the SSH tunnel.'
            provided = self.headers.get('X-Control-Token', '')
            if not provided:
                return 'No access token received. Paste the complete current Pi link into the Access link box and click Connect.'
            if not secrets.compare_digest(provided.encode('utf-8'), token.encode('utf-8')):
                return 'Token does not match this running server. Paste its latest link into the Access link box. Check that the SSH tunnel points to the vehicle server port.'
            return None

        def authorized(self):
            return self.access_error() is None

        def do_GET(self):
            if self.path == '/':
                self.reply(200, Path(__file__).with_name('vehicle.html').read_text(encoding='utf-8'), 'text/html')
            elif self.path == '/api/status' and self.authorized():
                self.reply(200, self.server.bridge.snapshot())
            else:
                self.reply(403, {'error': self.access_error() or 'Unknown page'})

        def do_POST(self):
            if self.path != '/api/command' or not self.authorized():
                self.reply(403, {'error': self.access_error() or 'Unknown command page'})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048:
                    raise ValueError('Invalid request size')
                self.connection.settimeout(2)
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError('Invalid command')
                self.server.bridge.command(data)
                self.reply(200, self.server.bridge.snapshot())
            except (ValueError, TypeError) as error:
                self.reply(400, {'error': str(error)})
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.bridge = bridge
    if bridge and bridge.sensors_only:
        bridge.message = 'Encoder monitor mode — motor arming disabled'
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0', help='Nucleo USB serial port')
    parser.add_argument('--http-port', type=int, default=8002)
    parser.add_argument('--monitor-only', '--sensors-only', dest='sensors_only', action='store_true', help='Read encoders with motor arming disabled')
    parser.add_argument('--demo', action='store_true', help='Simulated telemetry; never opens hardware')
    parser.add_argument('--disable-stall-stop', action='store_true', help='Report possible encoder stalls without stopping motors; connection-loss and missing-telemetry stops remain active')
    args = parser.parse_args()
    token = secrets.token_urlsafe(32)
    # Reserve HTTP port before opening hardware.
    server = make_server(None, args.http_port, token)
    try:
        bridge = Bridge(args.port, args.demo, args.sensors_only, args.disable_stall_stop)
    except Exception as error:
        server.server_close()
        raise SystemExit(f'Cannot open controller: {error}. Close the old controller/miniterm first.')
    server.bridge = bridge
    worker = threading.Thread(target=bridge.run, daemon=True)
    worker.start()
    print(f'BUILD {BUILD} | running {Path(__file__).resolve()}', flush=True)
    if args.disable_stall_stop:
        print('STALL MODE: warning only; automatic encoder stall stop disabled.', flush=True)
    print(f'\n{"DEMO — simulated motors" if args.demo else "LIVE — starts disarmed"}\n'
          f'Open through SSH tunnel: http://127.0.0.1:{args.http_port}/#token={token}\n'
          'WASD demo: release keys to stop; Disarm disables motors. Browser loss disarms after 1 second. Ctrl+C stops and exits.', flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.closed.set()
        worker.join(timeout=3)
        server.server_close()


if __name__ == '__main__':
    main()
