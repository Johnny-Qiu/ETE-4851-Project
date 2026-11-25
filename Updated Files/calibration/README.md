# Sensor settings

This is a reconstruction of the settings consumed by the sensor dashboard,
not a download of the Pi's complete robot_nav_calibration.json. The Pi could
not be reached during packaging on November 24, 2026.

The dashboard reads ~/robot_nav_calibration.json. Keep the existing file on
the original Pi. On a fresh Pi, the supplied JSON can be copied to that path.
Its 0.1 ohm shunt is an unverified conversion assumption, not a calibration
measurement. Laser yaw was saved previously; physical alignment is unverified.
Laser height is retained for context; the dashboard uses only frame and yaw.
No autonomous-driving calibration or approval is implied by these settings.
