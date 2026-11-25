# Package validation — November 24, 2026

- 16 existing WASD and encoder Python tests passed.
- 3 integrated sensor/WASD Python tests passed.
- Integrated browser JavaScript syntax check passed.
- Integrated page tests passed: explicit arm, combined keys, zero output on
  release while armed, page/focus loss, Space/Escape and input editing guards.
- Standalone WASD page tests passed.
- All packaged Python files passed syntax parsing.
- All three Bash startup scripts passed `bash -n` with Git for Windows Bash.
- The JavaScript tested in sensor_page_check.js matches the actual script
  embedded in all_sensors.py.
- The combined dashboard, bridge, vehicle controller, base controller, encoder
  monitor and standalone page match their original local files byte for byte.
- The package was scanned for saved dashboard URLs containing real access
  tokens, GitHub token patterns and private-key blocks; none were found.

These checks ran locally without commanding the vehicle. The Pi did not respond
to SSH at its last observed address, so there was no fresh deployment, physical
drive, sensor accuracy check, Nucleo firmware readback, or ROS workspace backup.
The recovered camera script and new startup wrappers have syntax validation;
their hardware behavior was not retested during this packaging task.

Historical context: the November 23 session observed both camera feeds, LiDAR,
IMU and the shared current display live. Faster steering was software tested;
physical driving and sensor calibration were not established by those checks.

MANIFEST_SHA256.txt hashes the bytes stored in Git, excluding the manifest itself.
The folder's .gitattributes uses LF for text so hashes also survive checkout.
