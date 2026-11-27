# BNO055 magnetometer test and calibration (IMU mounted on the car)

Status: **procedure and tooling only. Nothing here has been run on the car yet.**
No result in this folder is a measurement. Thresholds marked *proposed* are
starting points to adjust once you have real numbers.

## 1. What the current code actually does

Everything below is from the files in `Updated Files/` (Nov 23 package) and
`autonomous_testing/`.

| Item | Where | What it means for this test |
| --- | --- | --- |
| IMU driver | `Updated Files/firmware/motor_imu/bno055_controller.m` (reused by `ina219_diagnostic/build_vehicle.m` for Vehicle_Drive) | BNO055 over UART (USART6, PG9/PG14). Init: chip ID, CONFIG, page 0, normal power, degrees, then **NDOF** (`OPR_MODE=0x0C`). Loops reading Euler 0x1A (6 bytes) and `CALIB_STAT` 0x35. |
| The four calibration values | `bno055_text.m` → `CAL=S/G/A/M` in every telemetry line | System / Gyro / Accel / Mag, each 0–3. This is the BNO055 `CALIB_STAT` register split into four 2-bit fields. **The 4th number is the magnetometer.** |
| VALID / ERR | same line | VALID=1 only means a fresh Euler frame (<300 ms). It is *not* a calibration or accuracy flag. ERR=000 means no UART error. |
| Pi display | `Updated Files/sensor_viewer/all_sensors.py` IMU card | Shows Heading, Roll, Pitch, Calibration (`CAL` string) and ERR. No logging or history. `/state` JSON (token-protected, 127.0.0.1:8006) carries the same fields. |
| Saving calibration | not implemented | `pi_motor_imu/INSTRUCTIONS.txt`: "Flashing/resetting can require IMU recalibration; calibration is not saved." Every Nucleo power-up starts uncalibrated. |
| Raw magnetometer | not read | Firmware never reads `MAG_DATA` (0x0E–0x13), so field strength in µT is not available. Interference has to be judged from heading shift and the mag CAL value. |
| How heading is used | `autonomous_testing/.../robot_explorer/bridge.py` | Publishes `imu/data` yaw = `imu_yaw_sign * H + imu_yaw_offset_rad` whenever VALID=1 and ERR=0, **without checking CAL**. EKF uses yaw only, `imu0_relative=True`. So for navigation the key requirement is that heading does not jump when the LiDAR or motors switch on, more than absolute north accuracy. |

## 2. Why the LiDAR matters (what to expect)

- **Static magnets and steel** (motor magnets, LiDAR motor magnet when stopped,
  screws, battery) make a fixed offset ("hard iron"). The BNO055's calibration
  can absorb this **only if the parts are in their final position during
  calibration.** Calibrate with the LiDAR mounted and all wiring routed as it will
  run.
- **Changing fields** (the LiDAR motor spinning, drive-motor and servo current,
  high-current wires) cannot be calibrated out. They show up as a heading shift
  or extra noise when the device turns on, and in NDOF the BNO055 may drop the mag
  CAL value and start recalibrating, which also moves the heading.
- The test below separates these by recording the car **without moving it**
  while switching one thing on at a time.

## 3. Tool: `imu_mag_log.py` (read-only)

Records H/R/P/CAL/VALID/ERR to CSV and prints per-phase statistics
(circular mean and std of heading, peak-to-peak, drift first vs last 10 s,
minimum of each CAL value). `compare` gives the heading shift between two phases.
It never sends motor, arm or servo commands.

Run it **on the Pi** (the dashboard only listens on 127.0.0.1) with the token
from the URL the dashboard prints:

```bash
cd ~/johnny/imu_mag_test          # wherever you copy this folder
TOKEN=...                          # part after the token in the printed URL
D="--dashboard http://127.0.0.1:8006 --token $TOKEN"
python3 imu_mag_log.py watch $D                                  # live CAL readout
python3 imu_mag_log.py record $D --phase A_all_off --seconds 60 --out logs/A_all_off.csv
python3 imu_mag_log.py compare logs/A_all_off.csv logs/B_lidar_on.csv
```

`--serial /dev/ttyACM0` is also supported but only when no dashboard or other
controller has the port open (only one process may own it). Using the
dashboard source for every phase keeps the method identical when you need
`--drive` for the motor phases. Tests: `python3 -m unittest test_imu_mag_log -v`.

Do not record while the dashboard runs with `--demo`; it fakes `CAL=3/3/3/3`.

*Proposed* defaults (override with `--max-std`, `--max-drift`, `--max-shift`,
`--min-valid`): heading std ≤ 0.5°, drift ≤ 1.0° per phase, shift between phases
≤ 2.0°, VALID with ERR=000 in ≥ 99 % of samples, mag CAL stays 3.

## 4. Procedure

### 4.0 Setup
1. Car on its normal floor spot, away from steel desks, radiators, rebar-heavy
   floors if you can, and other running electronics. Mark the car outline with tape
   so you can return it to the same pose.
2. **Wheels raised** on a stand for any motor phase. Physical motor-power
   disconnect within reach.
3. Pi: start the dashboard **receive-only**: `bash start_dashboard.sh` (no
   `--drive`). Open the IMU card. Confirm `VALID=1` and `ERR=000`.
4. On a level floor, roll and pitch should read near 0°. If one reads near ±90°,
   the board is mounted on its side and heading is not the car's yaw. The firmware
   does not set `AXIS_MAP_CONFIG`, so stop and fix the mount or add an axis remap
   (see gaps).

### 4.1 Calibrate (mounted)
Calibration is lost on every Nucleo reset or power cycle, so do this each session
after the Nucleo boots, with the LiDAR in place (its power on or off does not
matter for this step; record which).
1. **Gyro (2nd value):** leave the car completely still for 3–5 s. Expect 3.
2. **Magnetometer (4th value):** this is the "figure-8". With the car disarmed and
   motor power off, lift it and slowly trace a figure-8 in the air, rolling and
   pitching it through the loops, for 20–40 s. If it is too heavy, rotate it
   slowly by hand through two full 360° turns on the floor while tipping the nose
   up/down and side to side ~20–30° (a wedge or book under one side). Watch until
   the 4th value reaches 3.
3. **Accelerometer (3rd value):** needs several stationary orientations (level,
   on each side, nose up/down). On a mounted car this is often impractical; if you
   can safely rest it at 4–6 different tilts for a few seconds each, do. Accel
   cal mostly affects roll/pitch, not heading. A low accel value is expected and
   is not a failure for this test, but note it.
4. **System (1st value):** fusion confidence. It usually rises to 3 once mag is 3.
5. Put the car back on its tape marks, still. Use `watch` to confirm the mag value
   is 3 before starting 4.2. If it falls back below 3 while stationary with
   everything off, the environment itself is disturbed; move location.

### 4.2 Stationary on/off interference comparison
Do not touch or move the car between phases. 60 s each unless noted. After each
phase, look at the mag CAL minimum and the heading mean.

| Phase | State | Command |
| --- | --- | --- |
| A | Everything off except Pi + Nucleo + IMU. LiDAR driver stopped and **visibly not spinning** (unplug the LiDAR USB if it keeps spinning). Motor power off. | `record ... --phase A_all_off` |
| B | LiDAR on (`bash start_lidar.sh`), confirm spinning, wait 5 s | `record ... --phase B_lidar_on` |
| C | LiDAR off again (Ctrl+C, confirm stopped) | `record ... --phase C_lidar_off_again` |
| D | Motor/servo supply on, still disarmed. Restart dashboard as `bash start_dashboard.sh --drive` (new token) | `record ... --phase D_power_on_disarmed` |
| E | Armed, wheels raised, hold **W** at 25 % in the browser. Releasing W stops, so use 30 s | `record ... --phase E_drive_25 --seconds 30` |
| F | Armed, hold **A** then **D** (steering servos at end stops), 30 s | `record ... --phase F_steer --seconds 30` |
| G | Everything on: LiDAR + W at 25 % | `record ... --phase G_all_on --seconds 30` |

Then compare:
```bash
python3 imu_mag_log.py compare logs/A_all_off.csv logs/B_lidar_on.csv       # LiDAR effect
python3 imu_mag_log.py compare logs/A_all_off.csv logs/C_lidar_off_again.csv # returns? (drift vs interference)
python3 imu_mag_log.py compare logs/C_lidar_off_again.csv logs/D_power_on_disarmed.csv
python3 imu_mag_log.py compare logs/D_power_on_disarmed.csv logs/E_drive_25.csv
python3 imu_mag_log.py compare logs/D_power_on_disarmed.csv logs/F_steer.csv
python3 imu_mag_log.py compare logs/A_all_off.csv logs/G_all_on.csv
```
Reading the results:
- B shifts but C returns to A → LiDAR interference (the earlier problem).
- B shifts and C does **not** return → drift or the BNO055 recalibrated during B
  (check if mag CAL dropped in B). Repeat A–C once more.
- Mag CAL drops below 3 in a phase → the BNO055 saw a disturbance; that phase fails
  even if the mean shift is small.
- Write the INA219 current from the dashboard next to phases E–G; it helps relate
  the heading shift to current.

### 4.3 Rotation check (calibrated heading tracks the car)
With everything in its normal driving state (LiDAR on), mark 4 poses 90° apart
(floor tiles or a square drawn with tape), and return to the start:
record 15 s at 0°, 90°, 180°, 270°, back to 0° (`--seconds 15`). Then run
`summary` on the five files. *Proposed* pass: each step 90° ± 5°, and the final 0°
within ± 2° of the first. Repeat with the LiDAR off. A rotation error that differs
between LiDAR on and off points to interference that depends on heading.

### 4.4 Record and decide
Keep the CSVs and the printed summaries. If A–C and A–G pass and the rotation
check passes, set `imu_yaw_sign` / `imu_yaw_offset_rad` for the autonomy config
from a known car-forward direction. If B or G fails:
1. Move the IMU further from the LiDAR and motors. Field from a small motor falls
   roughly with the cube of distance, so doubling the gap helps a lot. A short
   non-ferrous standoff/mast is common.
2. Twist motor and servo supply pairs, route them away from the IMU.
3. Re-run 4.1–4.2 after every change.
4. If it still fails, use the BNO055 **IMU fusion mode** (`OPR_MODE=0x08`, gyro +
   accel, no magnetometer). Heading becomes relative and drifts slowly, but the
   EKF already treats IMU yaw as relative (`imu0_relative=True`) and LiDAR SLAM
   corrects drift. This is a one-byte change in stage 6 of `bno055_controller.m`
   (12 → 8), which needs a rebuild and flash.

## 5. Code gaps found

1. **Calibration offsets are not saved or restored.** The BNO055 keeps its 22
   offset/radius bytes (`0x55–0x6A`) only while powered. Adding: read them in
   CONFIG mode once `CAL=3/3/3/3`, send them to the Pi, store them in a file on the
   Pi, and write them back between the CONFIG and NDOF stages at boot. Needs
   firmware (MATLAB/Simulink rebuild) and a Pi command, so it is not in this PR.
2. **No raw magnetometer telemetry.** Reading `MAG_DATA` (0x0E–0x13, 16 LSB/µT)
   and sending the field magnitude would give a direct µT measure of interference
   and would show it even when heading barely moves. Indoors the Earth field is
   typically ~25–65 µT; a jump of a few µT when the LiDAR starts would be clear.
3. **No axis remap.** `AXIS_MAP_CONFIG/SIGN` (0x41/0x42) are left at default;
   heading is only the car's yaw if the board is mounted flat (check 4.0 step 4).
4. **Autonomy ignores CAL.** `bridge.py` feeds heading to the EKF whenever VALID=1,
   even with mag CAL 0–2. Consider requiring CAL mag ≥ 2 (or inflating covariance)
   before publishing yaw.
5. **CAL resets to 0 on reinit display only.** After 3 failed transactions the
   firmware reinitializes and reports `CAL=0` until the next read; it does not
   reset the chip, so live calibration in the sensor is probably kept. Treat a
   brief CAL=0 with ERR≠000 as a communication event, not a lost calibration,
   until confirmed.
6. **No IMU logging in the dashboard.** This folder's script fills that gap
   without changing `Updated Files/`.
