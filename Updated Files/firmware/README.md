# Firmware references

The dashboard uses protocol v5 and requires DRIVE=1 and FW >= 2 telemetry.
It reads BNO055 fields H/R/P/VALID/CAL/ERR and INA219 INA/ST1/SH1/BV1.

- ina219_diagnostic: the diagnostic build programmed and verified on November 4,
  plus its Simulink model, generated C, custom sources, tests and uploader.
  See VALIDATION.txt for the observed sensor communication checks.
- fw7_reference: the independent encoder diagnostic reference already archived
  in autonomous_testing/firmware/fw7. Its own START_HERE.txt explains limitations.
- motor_imu and pi_motor_imu: the existing repository's BNO055 MATLAB source,
  base Simulink models and CubeMX project needed by the relative paths in
  build_vehicle.m. These support files are preserved from the prior handoff.

Neither image is claimed to be the exact image currently flashed on the Pi's
Nucleo. There was no firmware change in the November 23 dashboard work. The
Pi was unreachable during this upload, so its current firmware was not read
back. Keep the working firmware unless a deliberate firmware update is needed.
The original uploader text mentions two sensor addresses; the current physical
setup has one shared-supply INA219 at 0x40, and the dashboard displays only it.

These are project firmware artifacts, not a complete standalone STM32 toolchain
or all Simulink-generated support code. Rebuilding requires the original MATLAB
and STM32 support environment. No flashing was performed for this GitHub update.
