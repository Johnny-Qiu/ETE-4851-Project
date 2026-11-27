"""Log the shared-supply INA219 (bus voltage + current) from the running dashboard. Read-only.

    python3 battery_log.py --token TOKEN --seconds 600 --out ~/imu_logs/battery_plug.csv

Prints a line whenever the voltage jumps by more than 1 V, plus a summary at the end.
"""
import argparse
import csv
import json
import time
import urllib.request

p = argparse.ArgumentParser()
p.add_argument('--dashboard', default='http://127.0.0.1:8006')
p.add_argument('--token', required=True)
p.add_argument('--seconds', type=float, default=600)
p.add_argument('--hz', type=float, default=20)
p.add_argument('--out', required=True)
a = p.parse_args()

req = urllib.request.Request(a.dashboard + '/state', headers={'X-Sensor-Token': a.token})
t0 = time.monotonic()
last_v = None
peak_a = 0.0
rows = 0
with open(a.out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'volts', 'amps', 'fresh', 'ARM', 'D1', 'D2', 'S1', 'S2', 'SH1', 'BV1', 'ST1'])
    while (t := time.monotonic() - t0) < a.seconds:
        try:
            s = json.load(urllib.request.urlopen(req, timeout=1))
            c, fl = s['current'], s['fields']
            v, i = c['volts'], c['amps']
            w.writerow([f'{t:.2f}', v, i, s['board_fresh'], fl.get('ARM'), fl.get('D1'), fl.get('D2'),
                        fl.get('S1'), fl.get('S2'), fl.get('SH1'), fl.get('BV1'), fl.get('ST1')])
            rows += 1
            if v is not None:
                if last_v is not None and abs(v - last_v) > 1:
                    print(f'{time.strftime("%H:%M:%S")} t={t:7.2f}s  {last_v:6.2f} V -> {v:6.2f} V  current {i:+.3f} A', flush=True)
                last_v = v
                peak_a = max(peak_a, abs(i or 0))
        except Exception as exc:  # keep logging through dashboard hiccups
            print(f't={t:.2f}s read failed: {exc}', flush=True)
        time.sleep(max(0, 1 / a.hz - .005))
print(f'done: {rows} samples, peak |current| {peak_a:.3f} A, last voltage {last_v}', flush=True)
