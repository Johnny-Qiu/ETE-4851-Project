"""Log commanded steering pulses vs. the Nucleo's servo readback (S1/S2) from the running dashboard.

    python3 steer_log.py --token TOKEN --seconds 600 --out ~/imu_logs/steer.csv

If a servo jumps while commanded and readback pulses stay sane, the cause is electrical/mechanical
(power, ground, signal noise, binding), not the software. Prints every readback outside the
commanded range or any jump larger than --jump microseconds between samples. Read-only.
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
p.add_argument('--hz', type=float, default=50)
p.add_argument('--jump', type=int, default=600, help='flag pulse jumps bigger than this (us)')
p.add_argument('--out', required=True)
a = p.parse_args()

req = urllib.request.Request(a.dashboard + '/state', headers={'X-Sensor-Token': a.token})
t0, last, n, flagged = time.monotonic(), None, 0, 0
with open(a.out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'state', 'steering', 'cmd_front', 'cmd_rear', 'S1', 'S2', 'ARM', 'D1', 'D2', 'CMDERR', 'age', 'travel'])
    while (t := time.monotonic() - t0) < a.seconds:
        try:
            s = json.load(urllib.request.urlopen(req, timeout=1))
            d, fl = s['drive'], s['fields']
            cf, cr = d['pulses']
            s1, s2 = int(fl.get('S1', 0)), int(fl.get('S2', 0))
            travel = d['settings']['travel']
            w.writerow([f'{t:.3f}', d['state'], round(d['steering'], 3), cf, cr, s1, s2, fl.get('ARM'),
                        fl.get('D1'), fl.get('D2'), fl.get('CMDERR'), round(d['age'] or 0, 3), travel])
            n += 1
            lo, hi = 1500 - travel - 5, 1500 + travel + 5
            notes = []
            for name, v in (('S1', s1), ('S2', s2)):
                if v and not lo <= v <= hi:
                    notes.append(f'{name}={v} outside {lo}..{hi}')
            if last:
                for name, v, pv in (('S1', s1, last[0]), ('S2', s2, last[1])):
                    if v and pv and abs(v - pv) > a.jump:
                        notes.append(f'{name} jumped {pv}->{v}')
            if fl.get('CMDERR', '000') != '000':
                notes.append('CMDERR=' + fl['CMDERR'])
            if notes:
                flagged += 1
                print(f'{time.strftime("%H:%M:%S")} t={t:7.2f} cmd {cf}/{cr} readback {s1}/{s2} ' + '; '.join(notes), flush=True)
            last = (s1, s2)
        except Exception as exc:
            print(f't={t:.2f} read failed: {exc}', flush=True)
        time.sleep(max(0, 1 / a.hz - .004))
print(f'done: {n} samples, {flagged} flagged', flush=True)
