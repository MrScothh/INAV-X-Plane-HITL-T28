"""Stall speed of the X-Plane T-28: level at 18 m/s, throttle to idle, the autopilot holds height (pitch on climb
rate, no limit) and pulls ever harder as the speed bleeds, until the wing breaks (nose down or angle of attack
falling back); the slowest airspeed before that, flaps up, half and full. Same machinery as match_logs.py.
    python model_stall.py"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import match_logs as c  # noqa: E402


def stall(pos, ground, flap):
    c.place(pos, ground, 18.0, flap)
    p = c.Pilot(18.0)
    p.theta_max = 30.0
    # with the flaps down and the throttle cut the nose drops fast: the level-flight gains let it reach -15 degrees
    p.kp, p.ki = 0.02, 0.03
    c.fly(p, 10.0)
    p.throttle = 0.0
    t0 = time.monotonic()
    slowest, a_max = 99.0, -99.0
    while time.monotonic() - t0 < 40:
        cc = c.fly(p, 0.1, throttle=False)
        if not cc:
            continue
        v = cc[-1][1]
        # the autopilot pulls harder as the model sinks: the slowest speed before the break is the stall
        slowest = min(slowest, v[c.TAS])
        a_max = max(a_max, v[c.ALPHA])
        if v[c.THETA] < -15 or v[c.ALPHA] < a_max - 4:
            break
    return slowest, a_max


def main():
    pos = c.read_position()
    ground = pos[2] - pos[3]
    for i, n in enumerate(c.NAMES):
        c.s.sendto(b"RREF\x00" + c.struct.pack("<ii400s", 30, 700 + i, n.encode()), c.XP)
    c.dref("sim/operation/override/override_joystick", 1)
    try:
        for flap in (0.0, 0.5, 1.0):
            v, a = stall(pos, ground, flap)
            print("flaps %.1f: stall at %.1f m/s, maximum alpha %.1f" % (flap, v, a), flush=True)
    finally:
        c.dref("sim/cockpit2/controls/flap_ratio", 0.0)
        c.dref("sim/joystick/yoke_pitch_ratio", 0.0)
        c.dref("sim/operation/override/override_joystick", 0)


if __name__ == "__main__":
    main()
