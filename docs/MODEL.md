# The aircraft, matched to the logs

The real aircraft: an E-flite T-28 Trojan 1.2 m (span 1.224 m, length 0.985 m), EFLM4115 850 Kv motor, Avian 70 A
ESC, 12x7 three-blade propeller, 4S 2200, tricycle retracts, flaps, seven 9 g servos, on a TBS Lucid H7 with INAV 10
and a u-blox F10 GPS. Six blackbox logs at 1 kHz, 21 minutes in the air, in MANUAL, ACRO and ANGLE, with take-offs,
stalls, flap changes and landings. The model lives in X-Plane 11 as `aircraft/T28_RC_1200/T28_RC.acf`, written from
a blank Plane Maker file by `aircraft/generator/build_t28.py`, so every number is in one script and can be changed.

## Geometry and masses

T-28B planform at 1:10 (wing 0.27 / 0.12 m chords, 7 degrees dihedral, NACA 2412; tail NACA 0009), fuselage from the
Blender model (`aircraft/generator/blender`, which also writes the 3D object and its livery). Effective centre of
gravity 0.42 of the mean chord: measured 0.38 on the aircraft, 0.42 gives the logs' trim slope and pitch response
together (the big cowl moves the real neutral point forward of X-Plane's). Mass 2.2 kg: the manual gives 1.47 kg bare
and 1.7 kg with a 4S 2200, and the logs' climb, cruise and stall fit 2.0-2.4 kg with the real motor and prop
(`tools/mass_from_logs.py`). Ground attitude 1.8 degrees nose down as the real one at rest; main wheels 70 mm behind the
CG (closer, a bounce on the nose wheel sat the model on its tail).

X-Plane estimates the moments of inertia from the drawn shape and ignores the ACF: 0.06 / 0.08 / 0.02 m2 per kg.
Roll and pitch agree with a count of the real masses; yaw is a quarter of what a near-planar airframe has, so the
plugin sets 0.09 whenever the T-28 is loaded.

## Propulsion

The X-Plane electric motor gives torque = throttle x power / redline, and no propeller makes that fly the logs' cruise
and climb together: at 20-26 % throttle the real current is already the electronics' and the ESC brakes the prop. So
the plugin drives X-Plane's prop with the real motor: 850 Kv, 0.07 ohm, 3 A no-load (fitted, `tools/pick_esc.py`),
duty = throttle, 4S at 16.6 V open circuit with 0.023 ohm, current capped at 75 A (73 in the logs), braking down to the
commanded speed. The prop is the manual's 12x7 three-blade with the blade chord fitted to the logged current and
climb (30 to 16.5 mm), its moment of inertia half of what X-Plane computes for that chord: with the current cap the
spin-up is set by the inertia, and the logged current reaches its cap in 0.4-0.5 s.
Right thrust 1.5 degrees: without it the prop wash swings the model left at the start of the
take-off run, while the real one runs straight with no rudder.

## What matches, and how it was measured

Real figures from the logs (`tools/log_targets.py`: specific excess power Ps = climb rate + V dV/dt / g over 1 s
windows, throttle, current; `tools/real_trim.py`; the MANUAL rate per unit of command), the model open loop in
X-Plane with a simple autopilot (`tools/match_logs.py`, `model_stall.py`, `model_takeoff.py`,
`step_shape.py`). One unit = 1/500 of the stick = 1 us of servo, what INAV outputs.

| | model | real |
|---|---|---|
| throttle, level flight at 17.5 / 20.5 / 23.5 / 26.5 m/s | 0.42 / 0.49 / 0.56 / 0.63 | 0.42 / 0.46 / 0.57 / 0.64 |
| glide at zero throttle, m/s, 17.5 / 20.5 / 23.5 | -3.4 / -5.1 / -7.2 | -3.5 / -5.5 / -7.3 |
| climb at full throttle, m/s, 19 / 20 / 26 | 16.3 / 16.2 / 14.0 | 17.0 / 15.6 / 15.9 (1 g windows) |
| take-off, 1.2 to 14 m/s | 1.09 s (1.52 s from standstill with a 0.3 s throttle ramp) | 1.24 s, 72 A |
| stall, flaps up / half / full | 10.1 / 9.0 / 9.1 m/s | ~10.5-11 / 9-10 (virtual airspeed) |
| roll rate per unit, 14.5 / 20.5 / 23.5 / 26.5 m/s | 0.30 / 0.51 / 0.63 / 0.75 deg/s | 0.27 / 0.53 / 0.66 / 0.69 |
| pitch rate per unit, 17.5 / 20.5 / 23.5 / 26.5 | 0.27 / 0.32 / 0.38 / 0.45 | 0.17 / 0.26 / 0.38 / 0.49 |
| half flaps: elevator, body angle of attack | -44..-50 units, -6.0..-6.4 deg | -50, -5.9 |
| full flaps | -75..-83 units, -6.7..-7.1 deg | -75, -7.2 |
| command to half of the peak angular acceleration | 49 / 44 ms (roll / pitch) | 50 / 46 |
| angular acceleration in closed loop (sd at 100 Hz), roll / pitch / yaw | 4.0 / 1.4 / 0.9 | 3.3-4.2 / 1.7-2.5 / 1.2-1.9 |

Still different: the pitch rate per unit under 20 m/s (the real figure there has a correlation of 0.6). The trim slope
with speed is 4.6 units per m/s in the model; the logs hold only 5-10 s of steady level flight and give 4 to 9.

## Flaps and their compensation

The real aircraft flies with `smix 8 1 15 4 0 -1`: 4 % of the flap channel on the elevator, half of the correction
the logs showed (half flaps want about -50 us on the elevator, full about -75; the servo centre is moved by half the
correction so nothing changes with flaps up). The model's flaps (20 and 27 degrees, lift 1.5, moment -0.78) were
solved on a grid of live datarefs (`tools/flap_grid.py`) so that the attitude change and the elevator the aircraft
needs with half and full flaps are the logs', with the mixer rule off. With the rule on, INAV sees what the real board
saw.

## Sensors

`tools/t28_noise.json` is the noise model measured on the logs (`tools/noise_from_logs.py`, checked with
`compare_noise.py`): gyro and accelerometer noise against throttle, baro noise and its dynamic error (-0.030 m per
(m/s)^2), and the compass pulled by the motor current (a least-squares fit of the field against attitude and current,
on the field's real geometry). The plugin applies them from its *Sensors* and *Compass* menus.

## Limits

The aircraft is a foam model, glued and dented; two of the same type will not match to this precision, and the
numbers above are one airframe on two days. What the method gives is a model that makes the controller do, in the
simulator, what it does in the logs: the same throttle, the same trims, the same response. For another airframe the
tools in `tools/` are the way to get there.
