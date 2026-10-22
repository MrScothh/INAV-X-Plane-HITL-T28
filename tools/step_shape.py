"""How the X-Plane T-28 answers a stick step, open loop, against the real aircraft's logs: peak angular acceleration
per unit of command (1 unit = 1/500 of the stick, as INAV sends it) and steady rate per unit; their ratio is the time
constant of the mode, inertia over damping. Real T-28 (servo_response.py-style pooling of 14-16 steps):
roll 11.8 deg/s2 per us at 22 m/s against 0.53 deg/s per us steady, tau 45 ms; pitch 9.0 against 0.33, tau 27 ms.
The moments of inertia are set live (sim/aircraft/weight/acf_J*_unitmass, m2 per kg) before the steps.
    python step_shape.py [Jxx Jyy Jzz] [speed]"""
import os
import struct
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import time

import numpy as np

import match_logs as c

args = [a for a in sys.argv[1:]]
J = [float(x) for x in args[:3]] if len(args) >= 3 else None
V = float(args[3]) if len(args) >= 4 else 22.0
STEP = 0.12                       # 60 units
HOLD = 0.7
N_STEPS = 6
HERE = os.path.dirname(os.path.abspath(__file__))


def record(pilot, axis, delta):
    """One step held HOLD s: (time, rate) samples from the step, then back to the autopilot."""
    base = pilot.yp if axis == "pitch" else pilot.yr
    samples = []
    t0 = time.monotonic()
    kw = dict(pitch=False, throttle=False) if axis == "pitch" else dict(roll=False, throttle=False)
    # 60 ms of baseline before the step
    while time.monotonic() - t0 < 0.06:
        c.drain()
        pilot.step(0.0, **kw)
        samples.append((time.monotonic() - t0 - 0.06, c.val[c.Q if axis == "pitch" else c.P]))
        time.sleep(0.002)
    if axis == "pitch":
        pilot.yp = max(-1.0, min(1.0, base + delta))
    else:
        pilot.yr = max(-1.0, min(1.0, base + delta))
    t1 = time.monotonic()
    while time.monotonic() - t1 < HOLD:
        c.drain()
        pilot.step(0.0, **kw)
        samples.append((time.monotonic() - t1, c.val[c.Q if axis == "pitch" else c.P]))
        time.sleep(0.002)
    if axis == "pitch":
        pilot.yp = base
    else:
        pilot.yr = base
    c.fly(pilot, 3.0)
    t = np.array([x[0] for x in samples])
    r = np.array([x[1] for x in samples]) * np.sign(delta)
    tt = np.arange(-0.05, HOLD, 0.001)
    return np.interp(tt, t, r) - np.interp(np.arange(-0.05, 0.0, 0.001), t, r).mean()


def main():
    pos = c.read_position()
    ground = pos[2] - pos[3]
    if J:
        for ax, v in zip("xyz", J):
            c.dref("sim/aircraft/weight/acf_J%s%s_unitmass" % (ax, ax), v)
        time.sleep(0.5)
    for i, n in enumerate(c.NAMES):
        c.s.sendto(b"RREF\x00" + struct.pack("<ii400s", 100, 700 + i, n.encode()), c.XP)
    c.dref("sim/operation/override/override_joystick", 1)
    for a in ("pitch", "roll", "heading"):
        c.dref("sim/joystick/joystick_%s_augment" % a, 0.0)
    try:
        c.place(pos, ground, V)
        pilot = c.Pilot(V)
        c.fly(pilot, 12.0)
        print("in flight: AGL %.0f m, TAS %.1f m/s, P %.1f Q %.1f, throttle %.2f"
              % (c.val[c.AGL], c.val[c.TAS], c.val[c.P], c.val[c.Q], pilot.throttle), flush=True)
        if c.val[c.AGL] < 30 or c.val[c.TAS] < 10:
            raise SystemExit("not flying: stopping here")
        out = ["== %s, J %s, %.1f m/s, steps of %.0f units" % (time.strftime("%H:%M"), J, V, STEP * 500)]
        for axis in ("roll", "pitch"):
            curves = []
            for k in range(N_STEPS):
                curves.append(record(pilot, axis, STEP if k % 2 == 0 else -STEP))
            m = np.mean(curves, axis=0) / (STEP * 500)
            tt = np.arange(-0.05, HOLD, 0.001)
            acc = np.gradient(m, 0.001)
            acc = np.convolve(acc, np.ones(15) / 15, mode="same")
            s = acc[(tt >= 0) & (tt < 0.3)]
            pk = int(np.argmax(np.abs(s)))
            half = np.flatnonzero(np.abs(s[:pk + 1]) >= 0.5 * abs(s[pk]))[0]
            steady = m[(tt > HOLD - 0.25) & (tt < HOLD - 0.05)].mean()
            out.append("%-10s peak %6.2f deg/s2 per unit at %3d ms (half at %3d), steady %.3f deg/s per unit, tau %.0f ms"
                       % (axis, s[pk], pk, half, steady, 1000 * steady / s[pk]))
        for ln in out:
            print(ln, flush=True)
        with open(os.path.join(HERE, "step_shape.txt"), "a", encoding="utf-8") as f:
            f.write("\n".join(out) + "\n")
    finally:
        c.dref("sim/joystick/yoke_roll_ratio", 0.0)
        c.dref("sim/joystick/yoke_pitch_ratio", 0.0)
        c.dref("sim/operation/override/override_joystick", 0)
        for i, n in enumerate(c.NAMES):
            c.s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 700 + i, n.encode()), c.XP)


if __name__ == "__main__":
    main()
