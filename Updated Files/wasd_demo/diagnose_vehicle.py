"""Read board telemetry without sending any motor or arm commands."""
import argparse
from pathlib import Path
import time
import serial

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='/dev/ttyACM0')
    args=parser.parse_args()
    print('Controller folder:',Path(__file__).resolve().parent)
    print('Read-only check: no commands are sent. Close all other serial programs first.')
    with serial.Serial(args.port,115200,timeout=.2,exclusive=True) as link:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            line=link.readline().decode('ascii',errors='replace').strip()
            if not line.startswith('H='):continue
            print(line)
            if ' DRIVE=1 ' in line:
                print('MATCH: Vehicle_Drive firmware is installed. Use this folder\'s dashboard or vehicle_terminal.')
                return 0
            print('MISMATCH: older firmware is running. Use Upload_Vehicle.ps1 on Windows, then reconnect to Pi.')
            return 1
    print('No telemetry received. Check USB port/cable and whether the Nucleo program is running.')
    return 1

if __name__=='__main__':raise SystemExit(main())
