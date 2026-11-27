"""Bridge-level checks for the combined dashboard. Demo Bridge only; never opens hardware.

Run on the Pi:  python3 -m unittest johnny/drive_dashboard/test_drive_dashboard.py
"""
import sys
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'Updated Files' / 'wasd_demo'))
sys.path.insert(0, str(HERE))
from dashboard import Bridge  # noqa: E402
from drive_dashboard import DRIVE_ACTIONS, apply_drive_command  # noqa: E402


class CombinedDriveTests(unittest.TestCase):
    def setUp(self):
        self.b = Bridge(demo=True, command_interval=.02, steering_response_s=.06)
        self.t = threading.Thread(target=self.b.run, daemon=True)
        self.t.start()
        self.seq, self.n = {}, 0
        time.sleep(.2)

    def tearDown(self):
        self.send('stop')
        self.b.closed.set()
        self.t.join(1)

    def send(self, action, **extra):
        self.n += 1
        apply_drive_command(self.b, dict(action=action, client='unit-test-client', sequence=self.n, **extra), self.seq)

    def arm(self):
        self.send('arm')
        deadline = time.monotonic() + 2
        while self.b.control.state != 'armed' and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertEqual(self.b.control.state, 'armed')

    def test_steer_is_an_allowed_action(self):
        self.assertIn('steer', DRIVE_ACTIONS)

    def test_page_setup_payload_applies_all_fields(self):
        payload = dict(motor1=-1, motor2=1, front=-1, rear=1, travel=150, front_center=1510, rear_center=1490)
        self.send('configure', **payload)
        self.assertEqual(self.b.control.settings, payload)

    def test_hold_d_steers_and_center_returns_straight(self):
        self.send('configure', travel=200)
        self.arm()
        for _ in range(12):
            self.send('keys', keys=['d'], power=0, turn=1)
            time.sleep(.05)
        self.assertGreater(self.b.control.steering, .9)
        self.assertEqual(self.b.control.pulses(), [1700, 1300])
        self.send('keys', keys=[], power=0, turn=1)
        self.send('steer', value=0)
        self.assertEqual(self.b.control.pulses(), [1500, 1500])

    def test_snap_mode_holds_fixed_angle_and_release_centers(self):
        # What the page sends in snap mode: W/S only in 'keys' (turn=0), plus 'steer' on A/D change.
        self.send('configure', travel=150)
        self.arm()
        self.send('keys', keys=['w'], power=10, turn=0)
        self.send('steer', value=0.6)          # D held at 60 % steering angle
        for _ in range(6):                     # keys keep flowing while D is held
            self.send('keys', keys=['w'], power=10, turn=0)
            time.sleep(.05)
        self.assertEqual(self.b.control.pulses(), [1590, 1410])
        self.send('steer', value=0)            # D released
        self.send('keys', keys=['w'], power=10, turn=0)
        time.sleep(.1)
        self.assertEqual(self.b.control.pulses(), [1500, 1500])
        self.assertEqual(self.b.control.duty, 10)

    def test_configure_refused_while_armed(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.send('configure', travel=300)
        self.assertEqual(self.b.control.settings['travel'], 25)


if __name__ == '__main__':
    unittest.main()
