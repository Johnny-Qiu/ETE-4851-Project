import argparse
import csv
import os
import tempfile
import unittest

import imu_mag_log as m

LINE = ('H=+359.5 R=-001.2 P=+000.8 CAL=3/3/1/3 VALID=1 ERR=000 DRIVE=1 FW=2 S1=1500 S2=1500 T2=20000 '
        'INA=1 SH1=+00001 BV1=07400 ST1=1 SH2=+00000 BV2=00000 ST2=0 IDG=1 IC=7 IL=0 IE1=0 IE2=1 '
        'E1=00000 E2=00000 ARM=0 D1=+000.0 D2=+000.0 CMDERR=000')
ARGS = argparse.Namespace(min_valid=0.99, max_std=0.5, max_drift=1.0, max_shift=2.0)


def rows(headings, mag=3, valid=1):
    return [dict(t=i * 0.1, H=h, R=0, P=0, CAL_SYS=3, CAL_GYR=3, CAL_ACC=1, CAL_MAG=mag, VALID=valid, ERR=0)
            for i, h in enumerate(headings)]


class Tests(unittest.TestCase):
    def test_parse_vehicle_line(self):
        s = m.parse_line(LINE)
        self.assertEqual((s['H'], s['CAL_ACC'], s['CAL_MAG'], s['VALID'], s['ARM']), (359.5, 1, 3, 1, '0'))

    def test_parse_rejects_partial_and_bad_cal(self):
        self.assertIsNone(m.parse_line('=+000.0 R=+000.0'))
        self.assertIsNone(m.parse_line('H=+000.0 R=+000.0 P=+000.0 CAL=4/0/0/0 VALID=1 ERR=000'))

    def test_wraparound_mean_and_std(self):
        s = m.summarize(rows([359.8, 0.2] * 300))
        self.assertAlmostEqual(m.wrap180(s['heading_mean']), 0.0, places=6)
        self.assertLess(s['heading_std'], 0.3)
        self.assertTrue(all(ok for _, ok, _ in m.check_phase(s, ARGS)))

    def test_drift_and_mag_drop_fail(self):
        s = m.summarize(rows([10 + i * 0.01 for i in range(600)], mag=2))
        result = {n: ok for n, ok, _ in m.check_phase(s, ARGS)}
        self.assertFalse(result['heading drift'])
        self.assertFalse(result['mag CAL stays 3'])

    def test_shift_across_zero(self):
        checks = {n: (ok, d) for n, ok, d in m.compare(m.summarize(rows([359.0] * 600)), m.summarize(rows([3.0] * 600)), ARGS)}
        self.assertFalse(checks['heading shift'][0])
        self.assertIn('+4.00', checks['heading shift'][1])

    def test_invalid_samples_excluded(self):
        s = m.summarize(rows([5.0] * 10, valid=0))
        self.assertEqual(s['valid_fraction'], 0.0)
        self.assertFalse(m.check_phase(s, ARGS)[0][1])

    def test_csv_round_trip_compare_exit_code(self):
        with tempfile.TemporaryDirectory() as d:
            paths = []
            for name, h in (('a', 90.0), ('b', 90.5)):
                path = os.path.join(d, name + '.csv')
                with open(path, 'w', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=m.FIELDS)
                    w.writeheader()
                    for r in rows([h] * 600):
                        w.writerow(dict(r, phase=name, ARM='0', D1='', D2=''))
                paths.append(path)
            self.assertEqual(m.main(['compare'] + paths), 0)


if __name__ == '__main__':
    unittest.main()
