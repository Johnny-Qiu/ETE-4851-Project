#!/usr/bin/env python3
"""Read-only BNO055 heading/calibration logger and phase comparison tool.

Records the H/R/P/CAL/VALID/ERR fields that the Nucleo firmware already sends
(firmware/motor_imu/bno055_text.m, reused by Vehicle_Drive via vehicle_text.m).
It NEVER sends motor, arm or servo commands.

Sources (pick one):
  --dashboard URL --token TOKEN   poll the running dashboard's /state endpoint
                                   (Updated Files/sensor_viewer/all_sensors.py).
                                   Works while the dashboard owns the serial port,
                                   including --drive mode for motor-on phases.
  --serial PORT                    read the Nucleo USB serial port directly. Only
                                   when no dashboard/controller has the port open.

Examples:
  python3 imu_mag_log.py record --dashboard http://127.0.0.1:8006 --token TOKEN \
      --phase A_lidar_off --seconds 60 --out logs/A_lidar_off.csv
  python3 imu_mag_log.py watch  --dashboard http://127.0.0.1:8006 --token TOKEN
  python3 imu_mag_log.py compare logs/A_lidar_off.csv logs/B_lidar_on.csv
  python3 imu_mag_log.py summary logs/*.csv

The PASS/FAIL thresholds are proposed starting points for this vehicle, not
values established by measurement. Override them with the --max-* options.
"""
import argparse
import csv
import json
import math
import sys
import time
import urllib.request

FIELDS = ('t', 'phase', 'H', 'R', 'P', 'CAL_SYS', 'CAL_GYR', 'CAL_ACC', 'CAL_MAG', 'VALID', 'ERR', 'ARM', 'D1', 'D2')


def parse_fields(fields):
    """Turn a telemetry field dict (strings) into a sample, or None if unusable."""
    try:
        cal = [int(c) for c in fields['CAL'].split('/')]
        if len(cal) != 4 or not all(0 <= c <= 3 for c in cal):
            return None
        sample = dict(H=float(fields['H']), R=float(fields['R']), P=float(fields['P']),
                      CAL_SYS=cal[0], CAL_GYR=cal[1], CAL_ACC=cal[2], CAL_MAG=cal[3],
                      VALID=int(fields['VALID']), ERR=int(fields['ERR']))
    except (KeyError, ValueError):
        return None
    for key in ('ARM', 'D1', 'D2'):
        sample[key] = fields.get(key, '')
    return sample


def parse_line(line):
    """Parse one firmware telemetry line beginning with 'H='."""
    line = line.strip()
    if not line.startswith('H='):
        return None
    return parse_fields(dict(p.split('=', 1) for p in line.split() if '=' in p))


def wrap180(angle):
    return (angle + 180.0) % 360.0 - 180.0


def circular_mean(angles):
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(s, c)) % 360.0


def circular_std(angles):
    n = len(angles)
    s = sum(math.sin(math.radians(a)) for a in angles) / n
    c = sum(math.cos(math.radians(a)) for a in angles) / n
    r = min(1.0, math.hypot(s, c))
    return 0.0 if r >= 1.0 else math.degrees(math.sqrt(-2.0 * math.log(r)))


def summarize(rows, window_s=10.0):
    """Statistics for one stationary phase. rows: list of sample dicts with 't'."""
    total = len(rows)
    good = [r for r in rows if int(r['VALID']) == 1 and int(r['ERR']) == 0]
    out = dict(samples=total, valid_fraction=(len(good) / total) if total else 0.0)
    if not good:
        return out
    h = [float(r['H']) for r in good]
    mean = circular_mean(h)
    dev = [wrap180(a - mean) for a in h]
    t0, t1 = float(good[0]['t']), float(good[-1]['t'])
    first = [float(r['H']) for r in good if float(r['t']) <= t0 + window_s]
    last = [float(r['H']) for r in good if float(r['t']) >= t1 - window_s]
    out.update(duration_s=t1 - t0, heading_mean=mean, heading_std=circular_std(h),
               heading_p2p=max(dev) - min(dev),
               drift=wrap180(circular_mean(last) - circular_mean(first)),
               roll_mean=sum(float(r['R']) for r in good) / len(good),
               pitch_mean=sum(float(r['P']) for r in good) / len(good))
    for k in ('CAL_SYS', 'CAL_GYR', 'CAL_ACC', 'CAL_MAG'):
        vals = [int(r[k]) for r in good]
        out[k + '_min'] = min(vals)
        out[k + '_last'] = vals[-1]
    return out


def check_phase(s, args):
    """Proposed per-phase checks. Returns list of (name, ok, detail)."""
    if 'heading_mean' not in s:
        return [('valid samples', False, 'no VALID=1 ERR=000 samples')]
    return [
        ('valid fraction', s['valid_fraction'] >= args.min_valid, f"{s['valid_fraction']:.3f} >= {args.min_valid}"),
        ('mag CAL stays 3', s['CAL_MAG_min'] >= 3, f"min {s['CAL_MAG_min']}"),
        ('heading noise', s['heading_std'] <= args.max_std, f"std {s['heading_std']:.2f} <= {args.max_std} deg"),
        ('heading drift', abs(s['drift']) <= args.max_drift, f"{s['drift']:+.2f} within +/-{args.max_drift} deg"),
    ]


def compare(base, test, args):
    """Compare two stationary phases recorded without moving the car."""
    checks = check_phase(base, args) + check_phase(test, args)
    if 'heading_mean' in base and 'heading_mean' in test:
        shift = wrap180(test['heading_mean'] - base['heading_mean'])
        checks.append(('heading shift', abs(shift) <= args.max_shift, f"{shift:+.2f} within +/-{args.max_shift} deg"))
    return checks


# ---------------------------------------------------------------- sources
def dashboard_source(url, token, period=0.1):
    url = url.rstrip('/') + '/state'
    while True:
        req = urllib.request.Request(url, headers={'X-Sensor-Token': token})
        with urllib.request.urlopen(req, timeout=2) as resp:
            state = json.loads(resp.read().decode())
        if state.get('board_fresh'):
            sample = parse_fields(state.get('fields') or {})
            if sample:
                yield sample
        time.sleep(period)


def serial_source(port):
    import serial  # pyserial; read-only use, nothing is written
    with serial.Serial(port, 115200, timeout=0.3, exclusive=True) as link:
        link.reset_input_buffer()
        while True:
            sample = parse_line(link.readline(2048).decode('ascii', errors='replace'))
            if sample:
                yield sample


def open_source(args):
    if bool(args.dashboard) == bool(args.serial):
        sys.exit('Give exactly one of --dashboard URL (with --token) or --serial PORT')
    if args.dashboard:
        if not args.token:
            sys.exit('--dashboard needs --token (the token in the URL the dashboard printed)')
        return dashboard_source(args.dashboard, args.token)
    return serial_source(args.serial)


def cal_text(s):
    return f"{s['CAL_SYS']}/{s['CAL_GYR']}/{s['CAL_ACC']}/{s['CAL_MAG']}"


def print_summary(name, s, args):
    print(f"\n== {name}")
    if 'heading_mean' not in s:
        print(f"  samples {s['samples']}, no usable samples")
    else:
        print(f"  samples {s['samples']} over {s['duration_s']:.1f} s, valid {s['valid_fraction']:.3f}")
        print(f"  heading mean {s['heading_mean']:.2f}  std {s['heading_std']:.2f}  p2p {s['heading_p2p']:.2f}  drift {s['drift']:+.2f} deg")
        print(f"  roll {s['roll_mean']:+.1f}  pitch {s['pitch_mean']:+.1f} deg")
        print(f"  CAL min sys/gyr/acc/mag {s['CAL_SYS_min']}/{s['CAL_GYR_min']}/{s['CAL_ACC_min']}/{s['CAL_MAG_min']}"
              f"  last {s['CAL_SYS_last']}/{s['CAL_GYR_last']}/{s['CAL_ACC_last']}/{s['CAL_MAG_last']}")
    for name_, ok, detail in check_phase(s, args):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name_}: {detail}")


def load_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def cmd_record(args):
    rows, start = [], time.monotonic()
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for sample in open_source(args):
            t = time.monotonic() - start
            if t > args.seconds:
                break
            row = dict(sample, t=f'{t:.3f}', phase=args.phase)
            w.writerow(row)
            rows.append(row)
            print(f"\r{t:5.1f}s H={sample['H']:6.1f} CAL={cal_text(sample)} VALID={sample['VALID']} ERR={sample['ERR']:03d}  ", end='', flush=True)
    print()
    print_summary(args.phase, summarize(rows), args)


def cmd_watch(args):
    print('Showing CAL as system/gyro/accel/mag (0..3). Ctrl+C to stop.')
    try:
        for s in open_source(args):
            print(f"\rH={s['H']:6.1f} R={s['R']:+6.1f} P={s['P']:+6.1f} CAL={cal_text(s)} VALID={s['VALID']} ERR={s['ERR']:03d}   ", end='', flush=True)
    except KeyboardInterrupt:
        print()


def cmd_summary(args):
    for path in args.files:
        print_summary(path, summarize(load_csv(path)), args)


def cmd_compare(args):
    base, test = summarize(load_csv(args.base)), summarize(load_csv(args.test))
    print_summary(args.base, base, args)
    print_summary(args.test, test, args)
    print('\n== comparison (car must not have moved between the two recordings)')
    failed = False
    for name, ok, detail in compare(base, test, args):
        failed |= not ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    return 1 if failed else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--min-valid', type=float, default=0.99)
    p.add_argument('--max-std', type=float, default=0.5, help='proposed stationary heading std, deg')
    p.add_argument('--max-drift', type=float, default=1.0, help='proposed drift first vs last 10 s, deg')
    p.add_argument('--max-shift', type=float, default=2.0, help='proposed heading shift between phases, deg')
    sub = p.add_subparsers(dest='cmd', required=True)
    for name in ('record', 'watch'):
        s = sub.add_parser(name)
        s.add_argument('--dashboard')
        s.add_argument('--token')
        s.add_argument('--serial')
        if name == 'record':
            s.add_argument('--phase', required=True)
            s.add_argument('--seconds', type=float, default=60.0)
            s.add_argument('--out', required=True)
    s = sub.add_parser('summary'); s.add_argument('files', nargs='+')
    s = sub.add_parser('compare'); s.add_argument('base'); s.add_argument('test')
    args = p.parse_args(argv)
    return dict(record=cmd_record, watch=cmd_watch, summary=cmd_summary, compare=cmd_compare)[args.cmd](args) or 0


if __name__ == '__main__':
    raise SystemExit(main())
