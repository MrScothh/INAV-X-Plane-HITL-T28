"""Fits a brushless motor driven by an ESC to the T-28 logs: per airspeed and throttle cell of log_targets.py, the
specific excess power and the battery current the model gives against the logged ones.
Motor: duty = throttle, motor current Im = (duty Vb - w / Kv) / R, torque = (Im - I0) / Kv (SI), battery current
= duty Im + electronics; below the commanded speed the ESC brakes (damped light), throttle 0 included.
Prop: t28_performance.py's blade element model with the X-Plane factors (KT, KQ), which X-Plane then simulates.
Airframe drag: the X-Plane model as built (polar from its level-flight rpm) plus an extra drag area to fit.
    python pick_esc.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import build_t28 as bt  # noqa: E402
import t28_performance as tp  # noqa: E402
from pick_motor import KT, KQ, RHO, W, polar  # noqa: E402

I_BASE = 1.9            # A, flight controller, receiver, video and servos: the logged current at zero throttle
# (airspeed, throttle, Ps, battery current, battery voltage, windows) of log_targets.py, flaps up, cells >= 4
CELLS = [
    (14.5, 0.42, 0.71, 11.0, 16.63, 73), (14.5, 0.60, 5.48, 25.1, 15.73, 7),
    (17.5, 0.26, -3.17, 2.8, 16.83, 29), (17.5, 0.42, -0.22, 9.6, 16.07, 192), (17.5, 0.53, 3.82, 16.2, 16.53, 52),
    (17.5, 1.00, 11.73, 66.4, 15.62, 4),
    (20.5, 0.20, -5.60, 1.3, 16.81, 11), (20.5, 0.44, -0.48, 10.0, 16.23, 194), (20.5, 0.53, 2.00, 14.9, 16.09, 106),
    (20.5, 0.99, 14.52, 60.7, 15.61, 4),
    (23.5, 0.47, -2.68, 8.9, 16.51, 71), (23.5, 0.54, 0.54, 14.5, 16.13, 134), (23.5, 1.00, 16.38, 59.6, 15.27, 4),
    (26.5, 0.58, -1.86, 13.8, 16.55, 47), (26.5, 1.00, 15.64, 56.8, 14.96, 4),
    (29.5, 0.66, -1.96, 18.3, 16.30, 6),
    (14.5, 0.0, -2.34, 2.5, 15.59, 4), (17.5, 0.0, -3.52, 1.7, 16.91, 9), (20.5, 0.0, -5.48, 1.9, 16.90, 4),
    (23.5, 0.0, -7.30, 2.1, 16.84, 4), (26.5, 0.0, -12.95, 2.4, 15.56, 8),
]
GRID = list(range(0, 20001, 250))


def tables(diam, pitch, vs):
    acf = bt.Acf(os.path.join(bt.XP_DEFAULT, bt.OUT_REL, bt.OUT_NAME + ".acf"))
    pr = tp.Prop(acf, bt.XP_DEFAULT)
    k = diam / 11.0
    pr.R *= k
    pr.dr *= k
    pr.el = [(r * k, c, math.degrees(math.atan(pitch * 0.0254 / (2 * math.pi * r * k)))) for r, c, _ in pr.el]
    out = {}
    for v in vs:
        rows = []
        for n in GRID:
            t, q = pr.forces(max(n, 1) / 60.0, v)
            rows.append((n * 2 * math.pi / 60.0, t * KT, q * KQ))
        out[v] = rows
    return out


def equilibrium(rows, duty, vb, kv, r, i0):
    """Prop speed where the motor torque meets the prop's; thrust and motor current there."""
    kvs = kv * 2 * math.pi / 60.0      # rad/s per volt

    def excess(row):
        w, _, q = row
        im = (duty * vb - w / kvs) / r
        return (im - i0 * (1 if w > 0 else 0)) / kvs - q
    prev = rows[0]
    e_prev = excess(prev)
    for row in rows[1:]:
        e = excess(row)
        if e_prev >= 0 > e:
            f = e_prev / (e_prev - e)
            w = prev[0] + f * (row[0] - prev[0])
            t = prev[1] + f * (row[1] - prev[1])
            return w, t, (duty * vb - w / kvs) / r
        prev, e_prev = row, e
    if e_prev >= 0:
        w = rows[-1][0]
        return w, rows[-1][1], (duty * vb - w / kvs) / r
    w = rows[0][0]
    return w, rows[0][1], (duty * vb - w / kvs) / r


def main():
    a, b = polar()
    vs = sorted({c[0] for c in CELLS})
    results = []
    for diam in (10.0, 11.0, 12.0):
        for pitch in (6.0, 7.0, 8.0, 9.0, 10.0, 11.0):
            tab = tables(diam, pitch, vs)
            for kv in range(600, 1401, 50):
                for r in (0.03, 0.05, 0.07, 0.09, 0.12, 0.15):
                    for dcda in [x / 10000.0 for x in range(0, 201, 25)]:
                        err = 0.0
                        for v, g, ps, amp, vb, n in CELLS:
                            q = 0.5 * RHO * v * v
                            d = a * q + b / q + dcda * q
                            w, t, im = equilibrium(tab[v], g, vb, kv, r, 1.5)
                            ps_m = (t - d) * v / W
                            ib = g * im + I_BASE
                            weight = math.sqrt(min(n, 50))
                            err += weight * (((ps_m - ps) / 1.5) ** 2 + ((ib - amp) / 3.0) ** 2)
                        results.append((err, diam, pitch, kv, r, dcda))
    results.sort()
    for err, diam, pitch, kv, r, dcda in results[:6]:
        tab = tables(diam, pitch, vs)
        print("\nerr %.0f: prop %.0fx%.0f, Kv %d, R %.2f ohm, extra drag area %.4f m2" % (err, diam, pitch, kv, r, dcda))
        for v, g, ps, amp, vb, n in CELLS:
            q = 0.5 * RHO * v * v
            d = a * q + b / q + dcda * q
            w, t, im = equilibrium(tab[v], g, vb, kv, r, 1.5)
            print("  %4.1f m/s throttle %.2f: Ps %+5.1f / log %+5.1f | current %5.1f / log %5.1f A | rpm %5.0f, torque %+.3f N m"
                  " (%3d windows)" % (v, g, (t - d) * v / W, ps, g * im + I_BASE, amp, w * 60 / (2 * math.pi),
                                      (im - 1.5) / (kv * 2 * math.pi / 60.0), n))


if __name__ == "__main__":
    main()
