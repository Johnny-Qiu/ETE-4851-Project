# Quick check: does the IMU heading hold with the LiDAR in its new spot?

Uses only what is already in `Updated Files/` (sensor viewer in receive-only
mode + `start_lidar.sh`) plus the read-only `imu_mag_log.py` from this folder.
No motors are armed; the dashboard runs without `--drive`. Not yet run on the car.

## 1. Copy the logger to the Pi (Windows PowerShell)
From the folder holding `imu_mag_log.py` (this PR's branch, or the downloaded file):
```powershell
scp imu_mag_log.py swerve@PI_ADDRESS:~/
```

## 2. Start the sensor viewer (Pi terminal 1)
Stop any other dashboard/serial program first; only one may own the Nucleo port.
```bash
cd "$HOME/Updated Files"
bash start_dashboard.sh
```
It prints `SENSORS ONLY http://127.0.0.1:8006/#token=XXXX`. Copy the XXXX.

## 3. Open it on Windows (PowerShell, keep open)
```powershell
ssh -L 8006:127.0.0.1:8006 swerve@PI_ADDRESS
```
Open the full printed URL in a browser. On the IMU card check: Communication
error 000, "Live · valid", and Calibration `S/G/A/M`. The **4th number is the
magnetometer** (0 to 3).

## 4. Calibrate (LiDAR mounted in its new spot, motors off)
Keep the car still a few seconds (gyro, 2nd number, goes to 3). Then lift and
slowly figure-8 the car, or turn it by hand twice while tipping it, until the 4th
number reads 3. Put it down on tape marks and don't touch it after this.

## 5. LiDAR off / on / off (Pi terminal 2 and 3)
Terminal 3, with TOKEN set to the XXXX value:
```bash
TOKEN=XXXX; D="--dashboard http://127.0.0.1:8006 --token $TOKEN"; mkdir -p ~/imu_logs
python3 ~/imu_mag_log.py record $D --phase lidar_off  --seconds 60 --out ~/imu_logs/A_off.csv
```
Terminal 2: start the LiDAR, check it spins, wait 5 s:
```bash
cd "$HOME/Updated Files"
bash start_lidar.sh
```
Terminal 3:
```bash
python3 ~/imu_mag_log.py record $D --phase lidar_on   --seconds 60 --out ~/imu_logs/B_on.csv
```
Terminal 2: Ctrl+C, confirm the LiDAR has stopped spinning. Terminal 3:
```bash
python3 ~/imu_mag_log.py record $D --phase lidar_off2 --seconds 60 --out ~/imu_logs/C_off2.csv
python3 ~/imu_mag_log.py compare ~/imu_logs/A_off.csv ~/imu_logs/B_on.csv
python3 ~/imu_mag_log.py compare ~/imu_logs/A_off.csv ~/imu_logs/C_off2.csv
```

## 6. Reading it (proposed limits, adjust after real runs)
- **Good:** heading shift A→B within ±2°, mag (4th) stays 3 in all three, std ≤ 0.5°.
- **LiDAR still interferes:** B shifts or mag drops below 3, and C comes back near A.
- **Drift, not LiDAR:** C does not return to A. Repeat once.

Without the logger you can do the same by eye on the IMU card: note heading and
the 4th CAL number for ~30 s with the LiDAR off, start it, watch for a jump or a
mag drop, stop it and see if heading returns. The logger just gives numbers.
