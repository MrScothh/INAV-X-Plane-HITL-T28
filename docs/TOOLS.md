# Tools

Python 3 with numpy and pandas. Blackbox logs decoded with INAV's `blackbox_decode --unit-rotation deg/s --merge-gps`
(one CSV per log). The X-Plane tools talk to X-Plane over its UDP port 49000 (RREF, DREF, PREL, CMND) with X-Plane
running and the T-28 loaded; they fly the aircraft open loop with a small autopilot, no flight controller.

## Real aircraft, from the logs

| Script | What it gives |
|---|---|
| `log_targets.py` | Per airspeed and throttle band: specific excess power, current, voltage; angle of attack against load factor; speeds per flap setting. `LOG_DIR=<folder>` or the CSVs as arguments |
| `real_trim.py` | Elevator trim against airspeed in steady level flight, MANUAL and ACRO |
| `real_manual.py` | Roll and pitch rate per unit of command in MANUAL, per airspeed band |
| `servo_response.py real LOG.csv ...` | Delay from a quick servo step to half of the peak angular acceleration (pooled over the logs). `servo_response.py hitl outputs.csv sensors.csv` does the same on a recorded HITL flight |
| `mass_from_logs.py` | Flying weight and drag that fit the logged climb and current with the manual's motor and prop |
| `pick_esc.py`, `pick_esc_real.py`, `pick_motor.py` | Motor and ESC fits (resistance, no-load current, blade chord, extra drag) against the logged cells |
| `noise_from_logs.py`, `compare_noise.py` | The sensor noise model (`t28_noise.json`) and its check against a HITL flight |
| `sensor_delays.py` | GPS, accelerometer and gyro timing against each other |

## Model, in X-Plane

| Script | What it gives |
|---|---|
| `match_logs.py [label] [speeds]` | Level flight (throttle, stick, angle of attack), Ps at full and zero throttle, stick steps (rates per unit), flap effects, all against the logs' figures written in the script. `SKIP=ps,steps,flaps` skips parts |
| `model_stall.py` | Stall speed, flaps up / half / full |
| `model_takeoff.py` | The take-off run from the runway |
| `step_shape.py [Jxx Jyy Jzz] [speed]` | Shape of the step response: peak angular acceleration and steady rate per unit, their ratio the mode's time constant; inertias set live |
| `flap_grid.py [speed]` | Grid of flap lift, moment and full-flap angle on the live datarefs, against the logs |
| `t28_performance.py` | Blade element propeller and component drag estimate of the ACF |
| `reload_aircraft.py`, `read_dataref.py` | Reload the aircraft after regenerating it; read datarefs |

## The aircraft itself

`aircraft/generator/build_t28.py` (`XPLANE_DIR` in the environment, or `--xp`) writes the ACF into
`Aircraft/Extra Aircraft/T28_RC_1200` from `base_planemaker_blank.acf`, the empty file Plane Maker 11.55 writes for
File > New, and the Blender model in `blender/` (`t28_model.py` builds the mesh and writes the X-Plane object,
`t28_livery.py` paints it; `blender -b --factory-startup -P t28_model.py -- out`). Every constant is at the top of
the script with the reason for its value; `--check` only verifies an ACF.

Matching another airframe: take its logs through the first table, change the constants of the generator (or your
own ACF) and iterate with the second table until the model gives the logs' figures. The order that worked: mass and
propulsion first (cruise throttle, glide, climb, take-off), then stall (airfoil and incidence), then the controls
(effective throws, stabiliser incidence, CG for the trim slope and the rates together), flaps last.
