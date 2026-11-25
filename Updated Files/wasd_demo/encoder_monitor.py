"""Possible-stall detection from reported PWM and 16-bit encoder counts.

Runs on the Pi, not an electrical overcurrent protection device.
"""
import math


class EncoderMonitor:
    MIN_DUTY = 15
    GRACE = 1.0
    WINDOW = .5
    MIN_COUNTS = 4

    def __init__(self, stop_on_stall=True):
        self.stop_on_stall = stop_on_stall
        self.warning = ''
        self.previous = [None, None]
        self.previous_time = None
        self.start = [None, None]
        self.window_start = [None, None]
        self.counts = [0, 0]
        self.sign = [0, 0]
        self.rates = [None, None]
        self.missing_since = None
        self.fault = ''

    def update(self, fields, now):
        try:
            positions = [int(fields[f'E{i}']) for i in (1, 2)]
            duties = [float(fields[f'D{i}']) for i in (1, 2)]
            if any(not 0 <= p <= 65535 for p in positions) or any(not math.isfinite(d) or abs(d)>100 for d in duties):
                raise ValueError('Invalid telemetry')
        except (KeyError, ValueError):
            self.rates = [None, None]
            if self.missing_since is None:
                self.missing_since = now
            if fields.get('ARM') == '1' and now-self.missing_since >= .5:
                self.fault = 'Encoder telemetry missing — stopped. Check firmware and wiring.'
            return self.fault
        self.missing_since = None
        dt = None if self.previous_time is None else now-self.previous_time
        for i in range(2):
            delta = 0 if self.previous[i] is None else ((positions[i]-self.previous[i]+32768)%65536)-32768
            self.rates[i] = delta/dt if dt and 0 < dt <= .5 else None
            active = fields.get('ARM') == '1' and abs(duties[i]) >= self.MIN_DUTY
            sign = 1 if duties[i] > 0 else -1
            if not active or dt is None or dt <= 0 or dt > .5:
                self.start[i] = self.window_start[i] = None
                self.counts[i] = 0
                self.sign[i] = 0
            elif self.start[i] is None or sign != self.sign[i]:
                self.start[i] = now
                self.window_start[i] = None
                self.counts[i] = 0
                self.sign[i] = sign
            elif now-self.start[i] >= self.GRACE:
                if self.window_start[i] is None:
                    self.window_start[i] = now
                    self.counts[i] = 0
                else:
                    self.counts[i] += abs(delta)
                    if now-self.window_start[i] >= self.WINDOW:
                        if self.counts[i] < self.MIN_COUNTS:
                            if self.stop_on_stall:
                                self.fault = f'Possible stall: motor {i+1} — stopped. Check wheel and encoder wiring; re-arm explicitly.'
                            else:
                                self.warning = f'Possible stall: motor {i+1} — warning only; automatic stall stop disabled.'
                        self.window_start[i] = now
                        self.counts[i] = 0
            self.previous[i] = positions[i]
        self.previous_time = now
        return self.fault

    def acknowledge(self):
        self.fault = ''
        self.warning = ''
        self.start = [None, None]
        self.window_start = [None, None]
        self.sign = [0, 0]
        self.counts = [0, 0]

    def snapshot(self, fresh):
        return dict(fault=self.fault, warning=self.warning, stop_on_stall=self.stop_on_stall,
                    rates=self.rates[:] if fresh else [None, None],
                    counts=self.previous[:] if fresh else [None, None],
                    min_duty=self.MIN_DUTY, grace=self.GRACE, window=self.WINDOW,
                    min_counts=self.MIN_COUNTS)
