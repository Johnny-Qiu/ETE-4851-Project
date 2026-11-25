import time
import unittest
from encoder_monitor import EncoderMonitor
from dashboard import Bridge


def sample(a=0, b=0, duty=40, arm=1):
    return dict(E1=str(a), E2=str(b), D1=str(duty), D2=str(-duty), ARM=str(arm))


class EncoderTests(unittest.TestCase):
    def test_stuck_motor_stops_after_grace_and_window(self):
        m = EncoderMonitor()
        for k in range(15):
            self.assertEqual(m.update(sample(0, k*10), k*.1), '')
        for k in range(15, 20):
            m.update(sample(0, k*10), k*.1)
        self.assertIn('motor 1', m.fault)

    def test_motion_both_directions_and_wrap(self):
        m = EncoderMonitor()
        for k in range(40):
            self.assertFalse(m.update(sample((65530+k*10)%65536, (5-k*10)%65536), k*.1))
        self.assertGreater(m.rates[0], 0)
        self.assertLess(m.rates[1], 0)

    def test_zero_low_duty_and_reversals_reset_grace(self):
        m = EncoderMonitor()
        for k in range(60):
            duty = 0 if k < 10 else 10 if k < 20 else 40 if (k//5)%2 else -40
            self.assertFalse(m.update(sample(duty=duty), k*.1))

    def test_missing_telemetry_and_stale_output(self):
        m = EncoderMonitor()
        m.update(dict(ARM='1'), 0)
        self.assertIn('missing', m.update(dict(ARM='1'), .6))
        self.assertEqual(m.snapshot(False)['rates'], [None, None])
        m.acknowledge()
        self.assertEqual(m.fault, '')

    def test_bridge_stops_both_and_does_not_auto_rearm(self):
        b = Bridge(demo=True)
        now = time.monotonic()
        def receive(k, arm):
            b.receive(f'H=0 DRIVE=1 FW=2 E1=00000 E2=00000 ARM={arm} D1=+040.0 D2=-040.0 CMDERR=000', now+k*.1)
        b.control.next_frame()
        receive(0, 0)
        b.command(dict(action='arm', client='test-client'))
        b.control.next_frame()
        receive(1, 1)
        for k in range(2, 23):
            receive(k, 1)
        self.assertEqual(b.control.state, 'disarmed')
        self.assertEqual((b.control.duty, b.control.duty2), (0, 0))
        self.assertIsNone(b.owner)
        self.assertIn('Possible stall', b.snapshot()['message'])
        receive(24, 0)
        self.assertEqual(b.control.state, 'disarmed')

    def test_monitor_only_refuses_arm(self):
        b = Bridge(demo=True, sensors_only=True)
        with self.assertRaisesRegex(ValueError, 'arming is disabled'):
            b.command(dict(action='arm', client='test-client'))

    def test_warning_only_stall_keeps_motor_command_and_manual_stop(self):
        b = Bridge(demo=True, disable_stall_stop=True)
        now = time.monotonic()
        def receive(k, arm):
            b.receive(f'H=0 DRIVE=1 FW=2 E1=00000 E2=00000 ARM={arm} D1=+040.0 D2=-040.0 CMDERR=000', now+k*.1)
        b.control.next_frame()
        receive(0, 0)
        b.command(dict(action='arm', client='test-client'))
        b.control.next_frame()
        receive(1, 1)
        b.control.drive(40)
        commanded = (b.control.duty, b.control.duty2)
        for k in range(2, 23):
            receive(k, 1)
        self.assertEqual(b.control.state, 'armed')
        self.assertEqual((b.control.duty, b.control.duty2), commanded)
        self.assertEqual(b.owner, 'test-client')
        self.assertEqual(b.encoder.fault, '')
        self.assertIn('warning only', b.snapshot()['message'])
        b.command(dict(action='stop', client='test-client'))
        self.assertEqual(b.control.state, 'disarmed')
        self.assertEqual((b.control.duty, b.control.duty2), (0, 0))

    def test_warning_only_still_faults_for_missing_encoders(self):
        m = EncoderMonitor(stop_on_stall=False)
        m.update(dict(ARM='1'), 0)
        self.assertIn('missing', m.update(dict(ARM='1'), .6))

    def test_warning_only_still_stops_on_browser_loss(self):
        b = Bridge(demo=True, disable_stall_stop=True)
        b.owner = 'test-client'
        b.lease = 1
        b.expire(2.1)
        self.assertIsNone(b.owner)
        self.assertEqual(b.control.state, 'disarmed')


if __name__ == '__main__':
    unittest.main()
