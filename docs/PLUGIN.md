# The plugin

RomanLut's [INAV X-Plane HITL](https://github.com/RomanLut/INAV-X-Plane-HITL) 1.5.0 (commit `5757661`), with the
changes below. His documentation of the base plugin (OSD, map, graphs, failures, battery, SITL link) is in
`plugin/doc/`. Platform: Windows (built here with MinGW-w64 g++ 16); Linux with the original Docker build;
macOS not built.

## Changes

| Where | What |
|---|---|
| Rendering | OSD and graphs drawn in a borderless overlay window: X-Plane 11.50+ on Vulkan never calls the legacy draw callback |
| Link | Frames paced with a microsecond clock instead of `GetTickCount` (15.6 ms steps). *Link rate* menu: *Safe for stock INAV* (62/s) or *Every X-Plane frame* (needs INAV #12177) |
| GPS | 10 Hz, phase-locked, as a u-blox M10/F10 |
| Accelerometer | The lateral axis had the wrong sign |
| Motor | *Motor* menu: *X-Plane's electric motor* or *Brushless and ESC of the T-28*. The latter drives X-Plane's prop through `override_torque_motors` with a DC motor + ESC model (850 Kv, R 0.07 ohm, I0 3 A, duty = throttle, 4S 16.6 V with 0.023 ohm; braking down to the commanded speed, throttle 0 included), the braked prop's extra drag (0.0085 m2) and the battery voltage sagging under load sent to INAV. Only with the T-28 loaded (tail number T28RC) |
| Inertia | With the T-28 loaded the yaw inertia is set to 0.09 m2/kg: X-Plane recomputes inertias from the drawn shape at every load and gets this one at a quarter of what a near-planar airframe has |
| Servos | The controls INAV sends are applied with a dead time, interpolated between the two commands around that instant so the delay is exact at any frame rate; per axis, roll 18 ms and pitch/yaw/throttle 2 ms by default. Datarefs `inav_hitl/servo_dead_time_us` and `inav_hitl/servo_dead_time_roll_us`, writable live. With the T-28 loaded only |
| Outputs | `gear_servo` and `flap_servo` in `inavhitl.ini`: INAV servo outputs polled with MSP_SERVO five times a second and copied to X-Plane's gear handle (> 1250 us = down) and flaps (`flap_servo_up` .. `flap_servo_full` us = 0 .. 100 %, default 1000 .. 2000) |
| Sensors | *Sensors* menu: *Ideal (X-Plane)* or *Noise from the T-28 logs* (gyro, acc and mag noise measured on the real flights, scaled with throttle; applied once armed), plus *Baro reads low with airspeed* (-0.030 m per (m/s)^2 of dynamic pressure inside the fuselage, measured) |
| Compass | The field has the real geometry (inclination from a dipole, declination from X-Plane's magnetic variation). *Pulled by motor current*: the pull measured on the real aircraft, a function of throttle and airspeed |
| Time | *Time of day > Always noon* (default): the sun held at local noon, a long session never turns to night |
| Failures | GPS frozen / offset / drifting / circling / altitude, compass to zero, pitot faults (the originals) kept |
| Commands | `inav_hitl/connect_sitl_uart1` .. `uart8`, `inav_hitl/disconnect`: bind to keys or send over UDP (`CMND`) |
| Robustness | The last INAV command is applied once when the link drops, not re-applied every frame |

The sensor models come from six blackbox logs of the real T-28; the numbers are in `tools/t28_noise.json` and in
[MODEL.md](MODEL.md).

## Menu

```
Plugins > INAV HITL/SITL
  Link            Connect to Flight Controller / Connect to SITL > UART1..8 / Edit SITL IP
  GPS Fix         fix / satellites / timeout / glitches
  Compass         Normal / Failure (reads zero) / Pulled by motor current (T-28 logs)
  Sensors         Ideal (X-Plane) / Noise from the T-28 logs / Baro reads low with airspeed
  Pitot           none / simulated / failures
  Motor           X-Plane's electric motor / Brushless and ESC of the T-28 (logs)
  Link rate       Safe for stock INAV (up to 62/s) / Every X-Plane frame (INAV with #12177)
  Time of day     As X-Plane runs it / Always noon
  Attitude        from X-Plane / computed by the board from the simulated sensors (HITL_USE_IMU)
  Battery, Beeper, OSD, Graph, Map, Actions   as in the original
```

`inavhitl.ini` keys added: `fast_link`, `esc_model`, `sensor_noise`, `mag_current`, `baro_dynamic`, `fix_noon`,
`gear_servo`, `flap_servo`, `flap_servo_up`, `flap_servo_full`.

## Building

Windows: a MinGW-w64 g++ (WinLibs GCC 13 or newer) on the PATH, then `bash plugin/build_win.sh` writes
`plugin/build/win.xpl`; the GitHub workflow in `.github/workflows/plugin.yml` does the same on every push. Install by
copying it over `aircraft/T28_RC_1200/plugins/INAV-X-Plane-HITL/64/win.xpl` with X-Plane closed.

Linux: the original's `Dockerfile` and `src/build_lin64.sh` (`run-docker-compiler-for-linux.bat` on Windows) build
`lin.xpl`; not tested with these changes. The X-Plane SDK and OpenAL headers and import libraries are in `src/`.
