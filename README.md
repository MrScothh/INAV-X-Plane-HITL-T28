<h1 align="center">INAV-X-Plane-HITL-T28</h1>

<p align="center">A real model aircraft's twin in X-Plane 11 for INAV hardware-in-the-loop: a 1.2 m foam T-28 matched to
its own blackbox logs, and the INAV HITL plugin extended with what a real airframe does to a flight controller.</p>

INAV's HITL lets a real flight controller fly an aircraft inside X-Plane, through
[RomanLut's INAV X-Plane HITL plugin](https://github.com/RomanLut/INAV-X-Plane-HITL). Out of the box it flies a stock
X-Plane model with ideal sensors. This repository is for the step after that: making the simulated flight behave like the
real one, so that what INAV does in the simulator (take-off, autotune, navigation, autoland, failsafes) means
something for the real aircraft. Everything here was built against six blackbox logs of the real T-28 (21 minutes of
flight at 1 kHz) and against the manual's data for its motor and propeller.

**Status: first release.** Flown on the bench with a TBS Lucid H7 on INAV 10, X-Plane 11.55 on Windows. The plugin
builds for Linux too (untested here); macOS is not built.

## What it does

- **The aircraft** (`aircraft/T28_RC_1200`): an E-flite T-28 Trojan 1.2 m stand-in, built from a blank Plane Maker
  file by a script (`aircraft/generator/build_t28.py`), with the real motor (850 Kv), the real 12x7 three-blade
  propeller, 2.2 kg, and everything else fitted to the logs: throttle for level flight, glide, climb, take-off run,
  stall, roll and pitch response per unit of stick, trim, flaps and their pitching moment. The numbers, how they were
  measured and where the model still differs are in [docs/MODEL.md](docs/MODEL.md).
- **The plugin** (`plugin/`), RomanLut's INAV X-Plane HITL 1.5.0 plus:
  - a frame to INAV on every X-Plane frame (microsecond pacing; 120 frames/s instead of 62), with a *Link rate* menu
    to stay safe on stock firmware;
  - the T-28's brushless motor and ESC driving X-Plane's propeller (`override_torque_motors`): the ESC brakes the
    prop at low throttle as the real one does, the glide and the climb come out as in the logs;
  - a 4S battery that sags under load, the braked prop's drag, the real yaw inertia;
  - a per-axis servo dead time, interpolated, so the aircraft answers a command 50 ms later as the real servos do;
  - sensor noise, baro error and a compass pulled by the motor current, all measured on the logs (menu options, off by
    default);
  - the lateral accelerometer sign fixed, GPS at 10 Hz, solar noon held, more failures to inject;
  - X-Plane commands to connect without the mouse (`inav_hitl/connect_sitl_uart1..8`, `inav_hitl/disconnect`) and
    datarefs for the servo dead times.
  The full list, the menu and the datarefs: [docs/PLUGIN.md](docs/PLUGIN.md).
- **The tools** (`tools/`): the scripts that measured the real aircraft in its logs and the model in X-Plane, side by
  side, so the same method can be applied to another airframe: [docs/TOOLS.md](docs/TOOLS.md).

## Getting started

[docs/INSTALL.md](docs/INSTALL.md) takes you from an empty X-Plane 11 to INAV flying the T-28, including the INAV
settings the aircraft wants (modes, flaps and gear on one channel, the flap compensation of the real aircraft).

## What is where

```
aircraft/T28_RC_1200/   the X-Plane aircraft, plugin included: copy into X-Plane 11/Aircraft/Extra Aircraft/
aircraft/generator/     the script that builds the .acf from a blank Plane Maker file, and the Blender model
plugin/                 plugin source, Windows build script, the original Docker build for Linux, the original docs
tools/                  log-matching tools (Python 3, numpy, pandas) and the noise model measured on the logs
docs/                   installation, plugin reference, model report, tools
```

## Credits and license

The plugin is [INAV X-Plane HITL](https://github.com/RomanLut/INAV-X-Plane-HITL) by Roman Lut (MIT), taken at commit
`5757661` (2024-08-14) and modified; his copyright notice is kept in `plugin/LICENSE.RomanLut` and applies to that
code. The HITL protocol and the simulator side are INAV's ([iNavFlight/inav](https://github.com/iNavFlight/inav)).
The aircraft, the generator, the tools and the modifications are MIT, see [LICENSE](LICENSE).

The T-28 Trojan 1.2 m is a product of Horizon Hobby (E-flite); this is an independent flight model for simulation,
not affiliated with them.
