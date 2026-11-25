# Original local sources

Paths are relative to the original ETE 4851 Project folder.

- `sensor_viewer/all_sensors.py` ← `outputs/sensor_viewer/all_sensors.py`
- `sensor_viewer/sensor_page_check.js` ← `outputs/sensor_viewer/sensor_page_check.js`
- `sensor_viewer/test_dashboard_drive.cjs` ← `outputs/sensor_viewer/test_dashboard_drive.cjs`
- `sensor_viewer/test_drive_integration.py` ← `outputs/sensor_viewer/test_drive_integration.py`
- `wasd_demo/dashboard.py` ← `outputs/wasd_demo/dashboard.py`
- `wasd_demo/vehicle_control.py` ← `outputs/wasd_demo/vehicle_control.py`
- `wasd_demo/motor_base.py` ← `outputs/wasd_demo/motor_base.py`
- `wasd_demo/encoder_monitor.py` ← `outputs/wasd_demo/encoder_monitor.py`
- `wasd_demo/vehicle.html` ← `outputs/wasd_demo/vehicle.html`
- `wasd_demo/test_demo.py` ← `outputs/wasd_demo/test_demo.py`
- `wasd_demo/test_encoder_monitor.py` ← `outputs/wasd_demo/test_encoder_monitor.py`
- `wasd_demo/test_keyboard.cjs` ← `outputs/wasd_demo/test_keyboard.cjs`
- `wasd_demo/current_readings.py` ← `outputs/wasd_demo/current_readings.py`
- `wasd_demo/diagnose_vehicle.py` ← `outputs/wasd_demo/diagnose_vehicle.py`
- `firmware/ina219_diagnostic/Vehicle_Drive.bin` ← `outputs/ina219_diagnostic/Vehicle_Drive.bin`
- `firmware/ina219_diagnostic/Vehicle_Drive.slx` ← `outputs/ina219_diagnostic/Vehicle_Drive.slx`
- `firmware/ina219_diagnostic/build_vehicle.m` ← `outputs/ina219_diagnostic/build_vehicle.m`
- `firmware/ina219_diagnostic/open_vehicle.m` ← `outputs/ina219_diagnostic/open_vehicle.m`
- `firmware/ina219_diagnostic/current_ina219.c` ← `outputs/ina219_diagnostic/current_ina219.c`
- `firmware/ina219_diagnostic/current_ina219.h` ← `outputs/ina219_diagnostic/current_ina219.h`
- `firmware/ina219_diagnostic/vehicle_io.c` ← `outputs/ina219_diagnostic/vehicle_io.c`
- `firmware/ina219_diagnostic/vehicle_io.h` ← `outputs/ina219_diagnostic/vehicle_io.h`
- `firmware/ina219_diagnostic/vehicle_command.m` ← `outputs/ina219_diagnostic/vehicle_command.m`
- `firmware/ina219_diagnostic/vehicle_crc16.m` ← `outputs/ina219_diagnostic/vehicle_crc16.m`
- `firmware/ina219_diagnostic/vehicle_text.m` ← `outputs/ina219_diagnostic/vehicle_text.m`
- `firmware/ina219_diagnostic/test_vehicle_protocol.m` ← `outputs/ina219_diagnostic/test_vehicle_protocol.m`
- `firmware/ina219_diagnostic/Upload_Vehicle.ps1` ← `outputs/ina219_diagnostic/Upload_Vehicle.ps1`
- `firmware/ina219_diagnostic/VALIDATION.txt` ← `outputs/ina219_diagnostic/VALIDATION.txt`
- `firmware/ina219_diagnostic/Vehicle_Drive_generated.c` ← `outputs/ina219_diagnostic/Vehicle_Drive_ert_rtw/Vehicle_Drive.c`
- `firmware/fw7_reference/build_vehicle.m` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/build_vehicle.m`
- `firmware/fw7_reference/current_ina219.c` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/current_ina219.c`
- `firmware/fw7_reference/current_ina219.h` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/current_ina219.h`
- `firmware/fw7_reference/START_HERE.txt` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/START_HERE.txt`
- `firmware/fw7_reference/test_vehicle_protocol.m` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/test_vehicle_protocol.m`
- `firmware/fw7_reference/Upload_Vehicle.ps1` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/Upload_Vehicle.ps1`
- `firmware/fw7_reference/VALIDATION.txt` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/VALIDATION.txt`
- `firmware/fw7_reference/vehicle_command.m` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/vehicle_command.m`
- `firmware/fw7_reference/vehicle_crc16.m` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/vehicle_crc16.m`
- `firmware/fw7_reference/Vehicle_Drive.bin` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/Vehicle_Drive.bin`
- `firmware/fw7_reference/Vehicle_Drive_generated.c` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/Vehicle_Drive_generated.c`
- `firmware/fw7_reference/vehicle_io.c` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/vehicle_io.c`
- `firmware/fw7_reference/vehicle_io.h` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/vehicle_io.h`
- `firmware/fw7_reference/vehicle_text.m` ← `tmp/github_ete4851_handoff_20261108/autonomous_testing/firmware/fw7/vehicle_text.m`

The camera script was recovered from the saved November 23 setup commands,
with the final brightness change from 80 to 255 applied. The LiDAR and dashboard
wrappers and this documentation were added for this package from the working
commands. Calibration is a minimal reconstruction, not a fresh Pi download.
Runtime Python and browser files were copied without changes. Firmware references
are identified separately and are not a current Nucleo flash backup.

## Firmware build support

- `firmware/motor_imu/bno055_controller.m` ← `autonomous_testing/firmware/motor_imu/bno055_controller.m` (existing repository firmware support)
- `firmware/motor_imu/bno055_text.m` ← `autonomous_testing/firmware/motor_imu/bno055_text.m` (existing repository firmware support)
- `firmware/motor_imu/build_motor_imu.m` ← `autonomous_testing/firmware/motor_imu/build_motor_imu.m` (existing repository firmware support)
- `firmware/motor_imu/inspect_motor_model.m` ← `autonomous_testing/firmware/motor_imu/inspect_motor_model.m` (existing repository firmware support)
- `firmware/motor_imu/INSTRUCTIONS.txt` ← `autonomous_testing/firmware/motor_imu/INSTRUCTIONS.txt` (existing repository firmware support)
- `firmware/motor_imu/Motor_IMU_Test.slx` ← `autonomous_testing/firmware/motor_imu/Motor_IMU_Test.slx` (existing repository firmware support)
- `firmware/motor_imu/open_motor_imu.m` ← `autonomous_testing/firmware/motor_imu/open_motor_imu.m` (existing repository firmware support)
- `firmware/motor_imu/test_motor_imu.m` ← `autonomous_testing/firmware/motor_imu/test_motor_imu.m` (existing repository firmware support)
- `firmware/pi_motor_imu/build_pi_motor_imu.m` ← `autonomous_testing/firmware/pi_motor_imu/build_pi_motor_imu.m` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Inc/main.h` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Inc/main.h` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Inc/stm32_assert.h` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Inc/stm32_assert.h` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Inc/stm32f7xx_hal_conf.h` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Inc/stm32f7xx_hal_conf.h` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Inc/stm32f7xx_it.h` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Inc/stm32f7xx_it.h` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Src/main.c` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Src/main.c` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_hal_msp.c` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_hal_msp.c` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_hal_timebase_tim.c` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_hal_timebase_tim.c` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_it.c` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_it.c` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/Core/Src/system_stm32f7xx.c` ← `autonomous_testing/firmware/pi_motor_imu/hardware/Core/Src/system_stm32f7xx.c` (existing repository firmware support)
- `firmware/pi_motor_imu/hardware/motortest.ioc` ← `autonomous_testing/firmware/pi_motor_imu/hardware/motortest.ioc` (existing repository firmware support)
- `firmware/pi_motor_imu/INSTRUCTIONS.txt` ← `autonomous_testing/firmware/pi_motor_imu/INSTRUCTIONS.txt` (existing repository firmware support)
- `firmware/pi_motor_imu/motor_crc16.m` ← `autonomous_testing/firmware/pi_motor_imu/motor_crc16.m` (existing repository firmware support)
- `firmware/pi_motor_imu/Motor_Pi_IMU.slx` ← `autonomous_testing/firmware/pi_motor_imu/Motor_Pi_IMU.slx` (existing repository firmware support)
- `firmware/pi_motor_imu/open_pi_motor_imu.m` ← `autonomous_testing/firmware/pi_motor_imu/open_pi_motor_imu.m` (existing repository firmware support)
- `firmware/pi_motor_imu/pi_motor_command.m` ← `autonomous_testing/firmware/pi_motor_imu/pi_motor_command.m` (existing repository firmware support)
- `firmware/pi_motor_imu/pi_motor_text.m` ← `autonomous_testing/firmware/pi_motor_imu/pi_motor_text.m` (existing repository firmware support)
- `firmware/pi_motor_imu/test_pi_motor.m` ← `autonomous_testing/firmware/pi_motor_imu/test_pi_motor.m` (existing repository firmware support)
