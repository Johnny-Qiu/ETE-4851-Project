import unittest
from unittest.mock import patch
import struct
import time
import threading
import json
import urllib.request
from dashboard import Bridge, make_server
from vehicle_control import Control

def telemetry(arm=0):return f'H=0 DRIVE=1 FW=2 E1=00000 E2=00000 ARM={arm} D1=+000.0 D2=+000.0 CMDERR=000'

class DemoTests(unittest.TestCase):
    def test_arm_blocker_is_visible_instead_of_generic_stop(self):
        b=Bridge(demo=True);b.stop('STOP requested — both motors disabled')
        self.assertIn('telemetry',b.snapshot()['blocked_reason'])
        c=b.control;c.next_frame()
        b.receive(telemetry().replace('FW=2','FW=1'),time.monotonic())
        self.assertIn('FW=1',b.snapshot()['blocked_reason']);self.assertFalse(b.snapshot()['ready'])
        b.receive(telemetry(),time.monotonic())
        self.assertEqual(b.snapshot()['blocked_reason'],'');self.assertTrue(b.snapshot()['ready'])
    def test_midline_start_is_not_a_firmware_fault_or_arm_ack(self):
        b=Bridge(demo=True);c=b.control;c.next_frame()
        partial='686 E2=00000 ARM=0 D1=+000.0 D2=+000.0 CMDERR=000'
        b.receive(partial,time.monotonic())
        self.assertEqual(b.error,'');self.assertFalse(c.capable)
        self.assertIsNone(c.last_telemetry);self.assertFalse(c.confirmed_disarmed)
        b.receive(telemetry(),time.monotonic())
        self.assertTrue(c.capable);self.assertTrue(c.confirmed_disarmed)
        b.command(dict(action='arm',client='demo-client'));c.next_frame()
        b.receive('686 E2=00000 ARM=1 D1=+000.0 D2=+000.0 CMDERR=000',time.monotonic())
        self.assertEqual(c.state,'arming')
        b.receive(telemetry(1),time.monotonic());self.assertEqual(c.state,'armed')
        with self.assertRaises(RuntimeError):
            b.receive(telemetry().replace(' DRIVE=1',' SERVO=1'),time.monotonic())
    def test_independent_continuous_steering_and_throttle(self):
        b=self.armed();c=b.control;b.steer_updated=10
        def keys(held,now):
            with patch('dashboard.time.monotonic',return_value=now):
                b.command(dict(action='keys',client='demo-client',keys=held,power=100,turn=1))
        keys(['a'],10);b.advance_steering(10.1)
        self.assertEqual(c.pulses(),[1495,1505]);self.assertEqual(c.duty,0)
        keys(['w'],10.1);b.advance_steering(10.2)
        self.assertEqual(c.pulses(),[1495,1505]);self.assertEqual((c.duty,c.duty2),(100,-100))
        keys(['w','d'],10.2);b.advance_steering(10.3);b.advance_steering(10.4)
        self.assertEqual(c.pulses(),[1505,1495]);self.assertEqual(c.duty,100)
        keys(['d'],10.4);b.advance_steering(10.5)
        self.assertEqual(c.duty,0);self.assertEqual(c.pulses(),[1510,1490])
        keys([],10.5);b.advance_steering(10.6)
        self.assertEqual(c.pulses(),[1510,1490]);self.assertEqual(c.state,'armed')
        keys(['s'],10.6);b.advance_steering(10.7)
        self.assertEqual((c.duty,c.duty2),(-100,100));self.assertEqual(c.pulses(),[1510,1490])
        keys(['s','a','d'],10.7);b.advance_steering(10.8)
        self.assertEqual(c.pulses(),[1510,1490])
        keys(['d'],10.8)
        for i in range(1,30):b.advance_steering(10.8+i*.1)
        self.assertEqual(c.pulses(),[1525,1475])
        b.stop('test');self.assertEqual(b.steer_rate,0)
        c.next_frame();c.receive(telemetry(),time.monotonic())
        c.configure(dict(front_center=1480,rear_center=1530))
        self.assertEqual(c.pulses(),[1480,1530])
    def armed(self):
        b=Bridge(demo=True);b.control=Control(42);c=b.control
        c.next_frame();b.receive(telemetry(),time.monotonic());b.command(dict(action='arm',client='demo-client'))
        c.next_frame();c.receive(telemetry(1),time.monotonic());return b

    def test_simultaneous_keys_release_and_reversal_stay_armed(self):
        b=self.armed();c=b.control
        for throttle,steer in ((100,-1),(100,1),(0,0),(-100,1),(-50,-1),(0,0)):
            b.command(dict(action='input',client='demo-client',throttle=throttle,steering=steer))
            self.assertEqual(c.state,'armed');self.assertEqual((c.duty,c.duty2),(throttle,-throttle))
            self.assertEqual(struct.unpack('<hhHH',c.next_frame()[12:20]),(int(throttle*10),int(-throttle*10),1500+int(25*steer),1500-int(25*steer)))

    def test_validation_is_atomic(self):
        b=self.armed();c=b.control
        for throttle,steer in ((101,0),(0,2),(float('nan'),0),(10,float('inf')),(True,0)):
            with self.assertRaises(ValueError):b.command(dict(action='input',client='demo-client',throttle=throttle,steering=steer))
            self.assertEqual((c.duty,c.duty2,c.steering),(0,0,0))
        with self.assertRaises(ValueError):b.command(dict(action='input',client='other-client',throttle=100,steering=0))

    def test_stop_and_disconnect_require_rearm(self):
        b=self.armed();b.command(dict(action='input',client='demo-client',throttle=100,steering=1))
        b.expire(time.monotonic()+1.1);self.assertEqual(b.control.state,'disarmed')
        with self.assertRaises(ValueError):b.command(dict(action='input',client='demo-client',throttle=100,steering=0))
        self.assertEqual(struct.unpack('<hhHH',b.control.next_frame()[12:20]),(0,0,0,0))
        b=self.armed();b.control.check_link(time.monotonic()+1);self.assertEqual(b.control.state,'disarmed')

    def test_real_http_demo(self):
        b=Bridge(demo=True);server=make_server(b,0,'test-token');serial=threading.Thread(target=b.run);http=threading.Thread(target=server.serve_forever)
        serial.start();http.start()
        def req(path,data=None):
            r=urllib.request.Request(f'http://127.0.0.1:{server.server_port}'+path,data=json.dumps(dict(data,client='demo-client')).encode() if data else None,headers={'X-Control-Token':'test-token','Content-Type':'application/json'})
            return urllib.request.urlopen(r,timeout=2)
        def wait_state(name):
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                s=json.load(req('/api/status'))
                if s['state']==name and (name!='disarmed' or s['ready']):return s
                time.sleep(.01)
            self.fail('Missing state '+name)
        try:
            self.assertIn('Drive with WASD',req('/').read().decode());wait_state('disarmed')
            json.load(req('/api/command',dict(action='arm')));wait_state('armed')
            json.load(req('/api/command',dict(action='keys',keys=['a'],power=100,turn=1)))
            time.sleep(.15)
            s=json.load(req('/api/status'))
            self.assertLess(s['steering'],0);self.assertEqual(s['duty'],[0,0])
            json.load(req('/api/command',dict(action='keys',keys=['w','d'],power=100,turn=1)))
            time.sleep(.15)
            s=json.load(req('/api/status'));self.assertEqual(s['duty'],[100,-100])
            s=json.load(req('/api/command',dict(action='keys',keys=['w'],power=100,turn=1)))
            angle=s['steering'];time.sleep(.15)
            s=json.load(req('/api/status'))
            self.assertEqual(s['steering'],angle);self.assertEqual(s['state'],'armed')
            for throttle,steering in ((100,-1),(0,0),(-100,1),(0,0)):
                s=json.load(req('/api/command',dict(action='input',throttle=throttle,steering=steering)))
                self.assertEqual(s['state'],'armed');self.assertEqual(s['duty'],[throttle,-throttle])
            json.load(req('/api/command',dict(action='stop')));wait_state('disarmed')
        finally:
            b.closed.set();serial.join(2);server.shutdown();http.join(2);server.server_close()

if __name__=='__main__':unittest.main()
