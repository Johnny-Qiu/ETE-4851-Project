import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'wasd_demo'))
from dashboard import Bridge
from all_sensors import apply_drive_command

def telemetry(arm=0):
    return f'H=0 DRIVE=1 FW=7 E1=00000 E2=00000 ARM={arm} D1=+000.0 D2=+000.0 CMDERR=000'

class IntegratedDriveTests(unittest.TestCase):
    def armed(self):
        b=Bridge(demo=True,command_interval=.02,steering_response_s=.06)
        b.control.next_frame();b.receive(telemetry(),time.monotonic())
        b.command(dict(action='arm',client='test-client'))
        b.control.next_frame();b.receive(telemetry(1),time.monotonic())
        return b

    def test_release_keeps_armed_and_old_input_cannot_resume_motion(self):
        b=self.armed();sequences={}
        def send(seq,action='keys',keys=None):
            apply_drive_command(b,dict(sequence=seq,client='test-client',action=action,keys=keys or [],power=25,turn=1),sequences)
        send(1,keys=['w','a']);self.assertEqual(b.control.duty,25)
        send(3);self.assertEqual(b.control.state,'armed');self.assertEqual(b.control.duty,0)
        with self.assertRaises(ValueError):send(2,keys=['w'])
        self.assertEqual(b.control.duty,0)
        send(1,action='stop');self.assertEqual(b.control.state,'disarmed')
        with self.assertRaises(ValueError):send(4,keys=['w'])

    def test_steering_ramps_velocity_and_stops_on_fault(self):
        b=self.armed();now=time.monotonic();b.steer_updated=now;b.steer_rate=2
        b.advance_steering(now+.02)
        self.assertGreater(b.steer_velocity,0);self.assertLess(b.steer_velocity,2)
        for i in range(2,20):b.advance_steering(now+i*.02)
        self.assertGreater(b.control.steering,.5)
        velocity=b.steer_velocity;b.steer_rate=-2;b.advance_steering(now+.4)
        self.assertLess(b.steer_velocity,velocity);self.assertGreater(b.steer_velocity,-2)
        b.stop('fault');self.assertEqual(b.steer_velocity,0);self.assertEqual(b.control.duty,0)

    def test_connection_loss_still_disarms(self):
        b=self.armed();b.command(dict(action='keys',client='test-client',keys=['w'],power=25,turn=1))
        b.expire(b.lease+1.01)
        self.assertEqual(b.control.state,'disarmed');self.assertEqual(b.control.duty,0)

if __name__=='__main__':unittest.main()
