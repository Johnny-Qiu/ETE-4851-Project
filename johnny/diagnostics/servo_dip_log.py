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
p.add_argument('--jump', type=int, default=20, help='flag readback differing from command by more (us)')
p.add_argument('--lag', type=float, default=.3, help='flag readback/command disagreement longer than this (s)')
p.add_argument('--no-volts', action='store_true', help='hide INA219 volts (use while it is bypassed)')
p.add_argument('--out', required=True)
a = p.parse_args()

THROTTLE_BITS = {0: 'UNDERVOLTAGE NOW', 1: 'freq capped', 2: 'throttled', 3: 'temp limit',
                 16: 'undervoltage seen earlier', 18: 'throttled since boot'}


THROTTLE_WHY = None


def hwmon_undervolt():
    """Pi kernel 'rpi_volt' hwmon: in0_lcrit_alarm = 1 while undervoltage. Readable without sudo."""
    import glob
    for d in glob.glob('/sys/class/hwmon/hwmon*'):
        try:
            if open(d + '/name').read().strip() == 'rpi_volt':
                return int(open(d + '/in0_lcrit_alarm').read().strip())
        except (OSError, ValueError):
            pass
    return None


SEEN_UV = False


def throttled():
    """vcgencmd bits, or the hwmon fallback mapped to the same bits (0 now, 16 since logger start)."""
    global THROTTLE_WHY, SEEN_UV
    uv = hwmon_undervolt()
    if uv is not None:
        SEEN_UV = SEEN_UV or bool(uv)
        return (uv << 0) | (SEEN_UV << 16)
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=1)
        return int(out.stdout.strip().split('=')[1], 16)
    except FileNotFoundError:
        THROTTLE_WHY = 'vcgencmd not installed (Ubuntu: sudo apt install libraspberrypi-bin)'
    except Exception:
        THROTTLE_WHY = ('vcgencmd failed: ' + ((out.stderr or out.stdout).strip() if 'out' in locals() else '?')
                        + ' (try: sudo usermod -aG video $USER, then log in again)')
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
mismatch = [None, None]
print(f'Start: Pi power flags = {describe(thr)}' + (f' — {THROTTLE_WHY}' if thr is None else f' (0x{thr:x})')
      + '. Logging; Ctrl+C to stop.', flush=True)
# One logger at a time: a second copy opening the same CSV with 'w' wiped the first one's data
# (2026-11-27 grass test). The lock is released automatically when the process exits.
import fcntl
lock_file = open('/tmp/servo_dip_log.lock', 'w')
try:
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    raise SystemExit('Another servo_dip_log.py is already running (stop it: pkill -INT -f servo_dip_log.py). '
                     'Nothing was overwritten.')
import os
if os.path.exists(a.out) and os.path.getsize(a.out) > 0:
    raise SystemExit(f'{a.out} already exists; pick a new --out name so old data is kept.')
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
        volts = None if a.no_volts else (s.get('current') or {}).get('volts')
        state, err = d.get('state'), d.get('error') or d.get('message')
        # Board telemetry is ~10 Hz, so readback trails each steering change by up to ~165 ms (measured).
        # Only flag disagreement lasting longer than --lag seconds, or readback outside 1000..2000 us.
        for i, (name, v, c) in enumerate((('S1', s1, cf), ('S2', s2, cr))):
            bad = bool(v and c and abs(v - c) > a.jump and state == 'armed')
            if not bad:
                mismatch[i] = None
            elif mismatch[i] is None:
                mismatch[i] = t
            elif mismatch[i] is not False and t - mismatch[i] > a.lag:
                notes.append(f'{name} readback {v} != commanded {c} for >{a.lag:.1f}s')
                mismatch[i] = False   # reported; wait until it agrees again
            if v and not 1000 <= v <= 2000:
                notes.append(f'{name} readback {v} outside 1000..2000')
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
            # Timeline every ~0.25 s: what steering and the motors were doing when it went wrong.
            tl, last_t = [], -1.
            for r in rows:
                if float(r[0]) - last_t >= .25:
                    tl.append(f'    t={float(r[0]):7.2f} cmd {r[3]}/{r[4]} readback {r[5]}/{r[6]} duty {r[7]}/{r[8]}')
                    last_t = float(r[0])
            flagged = ['  timeline (front/rear pulses, M1/M2 duty %):'] + tl + flagged
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
