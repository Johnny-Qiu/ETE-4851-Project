"""Servo snap vs. power dip test. Read-only: polls the running drive dashboard, never commands it.

    python3 servo_dip_log.py --token TOKEN --out ~/imu_logs/servo_dip.csv

Drive normally. When a servo snaps/spazzes, DISARM (Space/STOP). Every armed -> disarmed
transition prints what happened in the previous --window seconds: commanded vs. readback steering,
motor duty, telemetry gaps, board errors and the Pi's own undervoltage flags. Ctrl+C ends the run.

Reading it: readback S1/S2 sane while a servo snapped -> not software. Snap right after a duty step
(throttle on, reverse) or with a telemetry gap / Pi undervoltage -> power. If the INA219 is wired as
a voltmeter on the servo rail (VIN+ and VIN- both to rail +, GND shared), its volts are logged too.
"""
import argparse
import collections
import csv
import json
import subprocess
import time
import urllib.request

p = argparse.ArgumentParser()
p.add_argument('--dashboard', default='http://127.0.0.1:8006')
p.add_argument('--token', required=True)
p.add_argument('--hz', type=float, default=50)
p.add_argument('--window', type=float, default=3.0, help='seconds summarised before each disarm')
p.add_argument('--jump', type=int, default=150, help='flag readback jumps bigger than this (us/sample)')
p.add_argument('--out', required=True)
a = p.parse_args()

THROTTLE_BITS = {0: 'UNDERVOLTAGE NOW', 1: 'freq capped', 2: 'throttled', 3: 'temp limit',
                 16: 'undervoltage since boot', 18: 'throttled since boot'}


def throttled():
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=1).stdout
        return int(out.strip().split('=')[1], 16)
    except Exception:
        return None


def describe(bits):
    return 'n/a' if bits is None else (', '.join(v for k, v in THROTTLE_BITS.items() if bits >> k & 1) or 'ok')


def num(x):
    try:
        v = float(x)
        return int(v) if v.is_integer() else v
    except (TypeError, ValueError):
        return None


req = urllib.request.Request(a.dashboard + '/state', headers={'X-Sensor-Token': a.token})
recent = collections.deque()
t0 = time.monotonic()
prev, prev_state, prev_err, thr, thr_at, n, events = None, None, None, throttled(), 0., 0, 0
print(f'Start: Pi power flags = {describe(thr)} (0x{thr or 0:x}). Logging; Ctrl+C to stop.', flush=True)
f = open(a.out, 'w', newline='')
w = csv.writer(f)
w.writerow(['t', 'state', 'steering', 'cmd_front', 'cmd_rear', 'S1', 'S2', 'D1', 'D2', 'ARM', 'CMDERR',
            'age', 'volts', 'pi_throttled', 'note'])
try:
    while True:
        t = time.monotonic() - t0
        notes = []
        if t - thr_at > .5:
            new = throttled()
            thr_at = t
            if new is not None and thr is not None and new & 0xF != thr & 0xF:
                notes.append('Pi power flags -> ' + describe(new))
            thr = new if new is not None else thr
        try:
            s = json.load(urllib.request.urlopen(req, timeout=1))
        except Exception as exc:
            print(f't={t:7.2f} dashboard read failed: {exc}', flush=True)
            time.sleep(.2)
            continue
        d, fl = s.get('drive') or {}, s.get('fields') or {}
        cf, cr = (d.get('pulses') or (None, None))[:2]
        s1, s2, d1, d2 = num(fl.get('S1')), num(fl.get('S2')), num(fl.get('D1')), num(fl.get('D2'))
        age = d.get('age')
        volts = (s.get('current') or {}).get('volts')
        state, err = d.get('state'), d.get('error') or d.get('message')
        if prev:
            for name, v, pv in (('S1', s1, prev[0]), ('S2', s2, prev[1])):
                if v and pv and abs(v - pv) > a.jump:
                    notes.append(f'{name} readback jumped {pv}->{v}')
        if age is not None and age > .2:
            notes.append(f'telemetry gap {age:.2f}s')
        if fl.get('CMDERR', '000') not in ('000', None):
            notes.append('CMDERR=' + str(fl.get('CMDERR')))
        if err and err != prev_err:
            notes.append('dashboard: ' + str(err))
        prev_err = err
        row = [f'{t:.3f}', state, round(d.get('steering') or 0, 3), cf, cr, s1, s2, d1, d2, fl.get('ARM'),
               fl.get('CMDERR'), round(age or 0, 3), volts, f'0x{thr or 0:x}', '; '.join(notes)]
        w.writerow(row)
        n += 1
        recent.append((t, row))
        while recent and t - recent[0][0] > a.window:
            recent.popleft()
        for note in notes:
            if not note.startswith('telemetry gap') or state == 'armed':
                print(f'{time.strftime("%H:%M:%S")} t={t:7.2f} {note}', flush=True)
        if prev_state == 'armed' and state != 'armed':
            events += 1
            rows = [r for _, r in recent]
            duty = [abs(x) for r in rows for x in (r[7], r[8]) if x is not None]
            rb = [x for r in rows for x in (r[5], r[6]) if x]
            cmd = [x for r in rows for x in (r[3], r[4]) if x]
            flagged = [f'  t={r[0]} {r[14]}' for r in rows if r[14]]
            print(f'\n=== DISARM #{events} at t={t:.2f} ({time.strftime("%H:%M:%S")}) — last {a.window:.0f} s ===\n'
                  f'  commanded pulses {min(cmd, default="-")}..{max(cmd, default="-")}  '
                  f'readback {min(rb, default="-")}..{max(rb, default="-")}\n'
                  f'  max |duty| {max(duty, default="-")}  max telemetry age {max((r[11] for r in rows), default=0)} s  '
                  f'min volts {min((r[12] for r in rows if r[12] is not None), default="-")}\n'
                  f'  Pi power flags: {describe(thr)}\n' + ('\n'.join(flagged) or '  no flagged samples') +
                  f'\n  (full detail in {a.out})\n', flush=True)
            f.flush()
        prev, prev_state = (s1, s2), state
        time.sleep(max(0, 1 / a.hz - .004))
except KeyboardInterrupt:
    pass
finally:
    f.close()
    print(f'done: {n} samples, {events} disarms, Pi power flags at end: {describe(throttled())}', flush=True)
