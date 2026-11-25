"""Sensor viewer with optional manual WASD control. Default mode is receive-only."""
import argparse
import json
import math
from pathlib import Path
import secrets
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

PAGE = r'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Car sensors</title>
<style>
body{background:#101923;color:#e7f0fa;font:16px system-ui;margin:24px}h1{margin-bottom:6px}p{color:#b2c3d5}main{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}.card{background:#192838;border:1px solid #33495d;border-radius:12px;padding:18px;min-width:0}h2{font-size:19px;margin-top:0}.screen{aspect-ratio:4/3;background:#080e15;display:grid;place-items:center;border-radius:8px;overflow:hidden}img,canvas{width:100%;height:100%;object-fit:contain}img[hidden]{display:none}.status{color:#ffce7c;font-size:14px}table{width:100%;border-collapse:collapse}td{padding:9px 0;border-bottom:1px solid #33495d}td:last-child{text-align:right;font-variant-numeric:tabular-nums}.wide{grid-column:span 2}@media(max-width:950px){main{grid-template-columns:1fr 1fr}.wide{grid-column:auto}}@media(max-width:600px){main{grid-template-columns:1fr}}
</style><h1>Car sensors</h1><p>Cameras, LiDAR, IMU and one shared supply sensor</p><p id="connection" class="status">Connecting…</p><main>
<section class="card" id="driving" style="grid-column:1/-1" hidden><h2>Manual WASD driving</h2><p id="driveStatus" class="status">Disarmed</p><button id="arm" type="button">Arm at zero</button> <button id="driveStop" type="button" style="background:#bd2638;color:white;padding:14px 30px">STOP / Disarm</button> <label>Power <select id="drivePower"><option>10</option><option selected>25</option><option>35</option><option>50</option><option>75</option><option>100</option></select> %</label><p>W forward · S reverse · A left · D right · Space/Escape stop. Release W/S to stop the motors while staying armed. Release A/D to hold steering. STOP / Disarm disables the motors.</p><p id="driveReadback"></p><details><summary>Direction and steering setup — change while disarmed</summary><div id="driveSetup"></div><label>Steering travel (µs) <input id="driveTravel" type="number" min="25" max="500" value="25"></label> <button type="button" id="applySetup">Apply setup</button></details></section>
<section class="card"><h2>Front camera</h2><p class="status" id="frontStatus">Waiting</p><div class="screen"><img id="front" alt="Front camera" hidden></div></section>
<section class="card"><h2>Rear camera</h2><p class="status" id="rearStatus">Waiting</p><div class="screen"><img id="rear" alt="Rear camera" hidden></div></section>
<section class="card"><h2>LiDAR</h2><p class="status" id="lidarStatus">Waiting</p><div class="screen"><canvas id="scan" width="600" height="450"></canvas></div><p id="frame">Missing returns are unknown space.</p></section>
<section class="card"><h2>IMU</h2><p class="status" id="imuStatus">Waiting</p><table id="imu"></table><p>Calibration order: System / Gyro / Accel / Mag. Levels 0–3.</p></section>
<section class="card wide"><h2>Shared battery supply</h2><p>One sensor measures the combined supply to both motor drivers and the servos.</p><p class="status" id="currentStatus">Waiting</p><table id="current"></table><p id="shunt"></p><p class="status" id="serialStatus"></p></section></main>
<script>
const token=new URLSearchParams(location.hash.slice(1)).get('token')||'';
const el=id=>document.getElementById(id),headers={'X-Sensor-Token':token};
const driveClient=crypto.randomUUID(),held=new Set();let driveEnabled=false,driveState='disarmed',driveBusy=false,setupReady=false,driveSequence=0;
for(const [key,label]of [['motor1','Motor 1 polarity'],['motor2','Motor 2 polarity'],['front','Front steering sign'],['rear','Rear steering sign']]){const line=document.createElement('label');line.textContent=label+' ';const select=document.createElement('select');select.id='setting-'+key;for(const value of [1,-1]){const option=document.createElement('option');option.value=value;option.textContent=value;select.append(option);}line.append(select);el('driveSetup').append(line,document.createTextNode(' '));}
async function driveCommand(action,extra={}){const response=await fetch('/drive',{method:'POST',headers:{...headers,'Content-Type':'application/json'},body:JSON.stringify({action,client:driveClient,sequence:++driveSequence,...extra}),signal:AbortSignal.timeout(700),keepalive:action==='stop'});const value=await response.json();if(!response.ok)throw Error(value.error||'Drive command failed');return value;}
function stopDrive(){held.clear();if(driveEnabled)driveCommand('stop').catch(e=>{el('driveStatus').textContent=e.message;});}
el('driveStop').onclick=stopDrive;
el('arm').onclick=async()=>{try{if(held.size)throw Error('Release all keys before arming');await driveCommand('arm');}catch(e){el('driveStatus').textContent=e.message;}};
el('applySetup').onclick=async()=>{try{const settings={travel:Number(el('driveTravel').value)};for(const key of ['motor1','motor2','front','rear'])settings[key]=Number(el('setting-'+key).value);await driveCommand('configure',settings);}catch(e){el('driveStatus').textContent=e.message;}};
function editing(target){return target&&['INPUT','SELECT','TEXTAREA'].includes(target.tagName);}
window.addEventListener('keydown',e=>{if(!driveEnabled||e.repeat)return;const key=e.key.toLowerCase();if(key===' '||key==='escape'){e.preventDefault();stopDrive();return;}if(editing(e.target)||!['w','a','s','d'].includes(key))return;e.preventDefault();if(driveState==='armed')held.add(key);});
window.addEventListener('keyup',e=>{const key=e.key.toLowerCase();if(held.delete(key)){e.preventDefault();if(driveEnabled&&driveState==='armed')driveCommand('keys',{keys:[...held],power:Number(el('drivePower').value),turn:1}).catch(()=>stopDrive());}});
window.addEventListener('blur',stopDrive);window.addEventListener('pagehide',stopDrive);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopDrive();});
setInterval(async()=>{if(!driveEnabled||driveBusy||document.hidden||!['armed','arming'].includes(driveState))return;driveBusy=true;try{if(driveState==='armed')await driveCommand('keys',{keys:[...held],power:Number(el('drivePower').value),turn:1});else await driveCommand('heartbeat');}catch(e){held.clear();el('driveStatus').textContent=e.message;driveCommand('stop').catch(()=>{});}finally{driveBusy=false;}},50);
function renderDrive(d){driveEnabled=!!d;el('driving').hidden=!d;if(!d)return;driveState=d.state;el('driveStatus').textContent=d.state.toUpperCase()+' · '+(d.blocked_reason||d.message);el('arm').disabled=!d.ready;el('applySetup').disabled=d.state!=='disarmed';el('driveReadback').textContent='Motor PWM: '+d.duty.map(x=>x.toFixed(1)+'%').join(' / ')+' · Steering pulses: '+d.pulses.join(' / ')+' µs';if(!setupReady){for(const key of ['motor1','motor2','front','rear'])el('setting-'+key).value=d.settings[key];el('driveTravel').value=d.settings.travel;setupReady=true;}if(d.state==='disarmed')held.clear();}
for(const name of ['front','rear']){
  let angle=name==='front'?180:0;
  try{const saved=localStorage.getItem('sensor-view-angle-'+name);if(saved!==null&&[0,90,180,270].includes(Number(saved)))angle=Number(saved);}catch(e){}
  const apply=()=>{el(name).style.transform='rotate('+angle+'deg)';};
  const button=document.createElement('button');button.textContent='Rotate view';
  button.style.cssText='margin-top:12px;padding:8px 14px;background:#263e54;color:#e7f0fa;border:1px solid #526c83;border-radius:7px;cursor:pointer';
  button.setAttribute('aria-label','Rotate '+name+' camera view 90 degrees');
  button.onclick=()=>{angle=(angle+90)%360;apply();try{localStorage.setItem('sensor-view-angle-'+name,String(angle));}catch(e){}};
  el(name).closest('section').append(button);apply();
}
const fmt=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';
function rows(id,items){el(id).replaceChildren(...items.map(([label,value])=>{const tr=document.createElement('tr');for(const v of [label,value]){const td=document.createElement('td');td.textContent=v;tr.append(td);}return tr;}));}
async function get(path){const r=await fetch(path,{headers,signal:AbortSignal.timeout(2500),cache:'no-store'});if(!r.ok)throw Error(r.status===401?'Access denied: open the complete token link.':'Feed unavailable');return r;}
function draw(scan){const c=el('scan'),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);ctx.strokeStyle='#33495d';for(let i=1;i<=4;i++){ctx.beginPath();ctx.arc(300,225,i*50,0,Math.PI*2);ctx.stroke();}ctx.fillStyle='#70bcff';ctx.fillRect(296,221,8,8);if(scan.fresh){ctx.fillStyle='#72e4bd';for(const [x,y]of scan.points)ctx.fillRect(300-y*50,225-x*50,2,2);}el('lidarStatus').textContent=scan.fresh?'Live · '+scan.points.length+' returns':'No fresh scan';el('frame').textContent=scan.frame_label+' · rings every metre · missing returns are unknown';}
function clearState(message){stopDrive();driveState='disarmed';el('arm').disabled=true;el('connection').textContent=message;for(const id of ['imuStatus','currentStatus'])el(id).textContent='Stale / disconnected';rows('imu',[]);rows('current',[]);draw({fresh:false,frame_label:'No scan'});}
async function state(){try{const s=await(await get('/state')).json();el('connection').textContent='Connected';draw(s.scan);renderDrive(s.drive);const f=s.fields;el('imuStatus').textContent=s.board_fresh?(f.VALID==='1'?'Live · valid':'Live · IMU invalid'):'No fresh board data';rows('imu',[['Heading (°)',s.imu_valid?f.H:'—'],['Roll (°)',s.imu_valid?f.R:'—'],['Pitch (°)',s.imu_valid?f.P:'—'],['Calibration',s.board_fresh?f.CAL:'—'],['Communication error',s.board_fresh?f.ERR:'—']]);el('currentStatus').textContent=s.current.status;rows('current',[['Total current (A)',fmt(s.current.amps,4)],['Bus voltage (V)',fmt(s.current.volts,3)],['Supply power (W)',fmt(s.current.watts,3)]]);el('shunt').textContent='Conversion assumes '+s.shunt_ohms+' Ω shunt. Verify the sensor’s actual shunt value.';el('serialStatus').textContent=s.serial_error||'';}catch(e){clearState(e.message);}}
const urls={};async function camera(name){try{const blob=await(await get('/'+name+'.jpg')).blob();const next=URL.createObjectURL(blob);el(name).src=next;el(name).hidden=false;if(urls[name])URL.revokeObjectURL(urls[name]);urls[name]=next;el(name+'Status').textContent='Live';}catch(e){el(name).hidden=true;el(name).removeAttribute('src');el(name+'Status').textContent='No fresh image';}}
async function loop(){if(!document.hidden)await Promise.all([state(),camera('front'),camera('rear')]);else{clearState('Paused');for(const n of ['front','rear'])el(n).hidden=true;}setTimeout(loop,350);}loop();
</script></html>'''

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
    if type(sequence) is not int or not 0 < sequence < 2**53 or not isinstance(client,str) or not 8 <= len(client) <= 100:
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
    import serial
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, LaserScan
    from rclpy.qos import qos_profile_sensor_data

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF575251717867104619-if02')
    parser.add_argument('--http-port', type=int, default=8006)
    parser.add_argument('--shunt-ohms', type=float, default=None)
    parser.add_argument('--enable-drive', action='store_true', help='Enable the existing WASD controller; starts disarmed')
    parser.add_argument('--drive-dir', default=str(Path.home()/'wasd_demo'))
    args = parser.parse_args()
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
    shared = dict(fields={}, board_at=0., serial_error='Waiting for board', scan=None, cameras={})
    token = secrets.token_urlsafe(32)
    drive = None
    drive_worker = None
    drive_sequences = {}
    rclpy.init()
    node = rclpy.create_node('car_sensors_only')
    bridge = CvBridge()
    def image_cb(msg, name):
        now = time.monotonic()
        with lock:
            previous = shared['cameras'].get(name)
        if previous and now-previous[0] < .2:
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
            if math.isfinite(radius) and msg.range_min < radius < min(msg.range_max, 4.5):
                angle = msg.angle_min+i*msg.angle_increment+yaw
                points.append([radius*math.cos(angle), radius*math.sin(angle)])
        with lock:
            shared['scan'] = (time.monotonic(), points, 'Saved mounting yaw · forward up' if mounted else 'Raw laser frame · +X up')
    subs = [node.create_subscription(Image, '/camera/image_raw', lambda m:image_cb(m,'front'), qos_profile_sensor_data),
            node.create_subscription(Image, '/camera/rear/image_raw', lambda m:image_cb(m,'rear'), qos_profile_sensor_data),
            node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)]
    def receive_board():
        while not stop.is_set():
            try:
                with serial.Serial(args.port, 115200, timeout=.3, exclusive=True) as link:
                    while not stop.is_set():
                        line = link.readline(2048).decode('ascii', errors='replace').strip()
                        if not line.startswith('H='):
                            continue
                        fields = dict(item.split('=',1) for item in line.split() if '=' in item)
                        with lock:
                            shared.update(fields=fields, board_at=time.monotonic(), serial_error='')
            except (serial.SerialException, OSError) as exc:
                with lock:
                    shared['serial_error'] = 'Board connection: '+str(exc)+'. Stop WASD/navigation before opening this viewer.'
                stop.wait(2.)
    class Handler(BaseHTTPRequestHandler):
        def drive_access(self):
            host = self.headers.get('Host', '')
            return (bool(re.fullmatch(r'(?:127\.0\.0\.1|localhost)(?::[0-9]{1,5})?', host))
                and self.headers.get('Origin', 'http://'+host) == 'http://'+host
                and secrets.compare_digest(self.headers.get('X-Sensor-Token',''), token))
        def do_POST(self):
            if self.path != '/drive' or drive is None or not self.drive_access():
                code, value = 403, {'error':'Drive access denied'}
            else:
                try:
                    self.connection.settimeout(1.)
                    length = int(self.headers.get('Content-Length','0'))
                    if not 0 < length <= 2048:
                        raise ValueError('Invalid request size')
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict) or data.get('action') not in ('arm','stop','keys','heartbeat','configure'):
                        raise ValueError('Invalid drive command')
                    apply_drive_command(drive, data, drive_sequences)
                    code, value = 200, drive.snapshot()
                except (ValueError, TypeError, OSError) as exc:
                    code, value = 400, {'error':str(exc)}
            payload = json.dumps(value).encode()
            self.send_response(code)
            self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass
        def log_message(self, *unused):
            pass
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/':
                code, mime, payload = 200, 'text/html; charset=utf-8', PAGE.encode()
            elif not secrets.compare_digest(self.headers.get('X-Sensor-Token',''), token):
                code, mime, payload = 401, 'text/plain', b'Access denied'
            else:
                now = time.monotonic()
                with lock:
                    if path == '/state':
                        fields = shared['fields'].copy()
                        fresh = bool(shared['board_at'] and now-shared['board_at'] < 1.5)
                        drive_status = drive.snapshot() if drive is not None else None
                        if drive_status is not None:
                            fields = drive_status['fields']
                            fresh = drive_status['fresh']
                        scan = shared['scan']
                        scan_fresh = bool(scan and now-scan[0] < .8)
                        payload = json.dumps(dict(drive=drive_status, fields=fields, board_fresh=fresh, imu_valid=fresh and fields.get('VALID')=='1',
                            current=board_values(fields, fresh, shunt), shunt_ohms=shunt, serial_error=drive_status['error'] if drive_status else shared['serial_error'],
                            scan=dict(fresh=scan_fresh, points=scan[1] if scan_fresh else [], frame_label=scan[2] if scan else 'Waiting for scan'))).encode()
                        code, mime = 200, 'application/json'
                    else:
                        frame = shared['cameras'].get(path[1:-4]) if path in ('/front.jpg','/rear.jpg') else None
                        code, mime, payload = (200,'image/jpeg',frame[1]) if frame and now-frame[0] < 1.2 else (503,'text/plain',b'No fresh image')
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass
    server = ThreadingHTTPServer(('127.0.0.1', args.http_port), Handler)
    if args.enable_drive:
        sys.path.insert(0, str(Path(args.drive_dir).resolve()))
        from dashboard import Bridge
        try:
            drive = Bridge(port=args.port, command_interval=.02, steering_response_s=.06)
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
    print(f'SENSORS ONLY http://127.0.0.1:{args.http_port}/#token={token}', flush=True)
    print('WASD enabled; starts disarmed. Key release stops movement while armed; leaving page stops and disarms. No automatic obstacle stopping.' if drive else 'Receive-only Nucleo connection. Start camera and LiDAR drivers separately.', flush=True)
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
