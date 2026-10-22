"""Chooses the prop pitch, the motor torque and the extra airframe drag that make the X-Plane T-28 fly like the logs
under power: throttle for level flight and Ps at full throttle per airspeed band (log_targets.py).
X-Plane's electric motor gives torque = throttle x Pmax / redline speed (measured on the ground); the prop is
t28_performance.py's blade element model, which matches X-Plane's static thrust and torque within 7 % (factors below).
The drag of the model as built comes from its level-flight rpm in X-Plane (match_logs.py run v1_3kg_1hp).
    python pick_motor.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import build_t28 as bt  # noqa: E402
import t28_performance as tp  # noqa: E402

KT, KQ = 0.93, 1.04          # X-Plane / blade element, static, 3200-6600 rpm
RHO = 1.225
W = bt.MASS_KG * 9.81
S = 0.2387
# v1 model in X-Plane: level flight (airspeed, drag from rpm) -> drag polar D = a q + b / q
V1 = ((13.9, 3.22), (17.2, 2.93), (20.2, 3.16), (23.3, 3.60), (26.2, 3.90))
# logs: airspeed, throttle for level flight, Ps at full throttle
LOG = ((14.5, 0.28, 9.8), (17.5, 0.42, 11.7), (20.5, 0.46, 14.5), (23.5, 0.57, 16.4), (26.5, 0.64, 15.6),
       (29.5, None, 11.6))


def polar():
    # least squares for D = a q + b / q
    rows = [(0.5 * RHO * v * v, d) for v, d in V1]
    a11 = sum(q * q for q, _ in rows)
    a12 = sum(1.0 for _ in rows)
    a22 = sum(1.0 / (q * q) for q, _ in rows)
    b1 = sum(q * d for q, d in rows)
    b2 = sum(d / q for q, d in rows)
    det = a11 * a22 - a12 * a12
    return (b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det


def prop_table(pr, pitch_in, vs):
    """Thrust and torque on an rpm grid at each airspeed, for a prop of that geometric pitch."""
    els = []
    for r, c, _ in pr.el:
        els.append((r, c, math.degrees(math.atan(pitch_in * 0.0254 / (2 * math.pi * r)))))
    pr.el = els
    grid = list(range(1000, 20001, 250))
    tab = {}
    for v in vs:
        tab[v] = [(n, *pr.forces(n / 60.0, v)) for n in grid]
    return tab


def equilibrium(rows, qm):
    """rpm where the prop's torque equals the motor's (torque rises with rpm); thrust there."""
    prev = rows[0]
    for row in rows[1:]:
        if row[2] * KQ >= qm:
            f = (qm - prev[2] * KQ) / max(1e-9, (row[2] - prev[2]) * KQ)
            return prev[0] + f * (row[0] - prev[0]), (prev[1] + f * (row[1] - prev[1])) * KT
        prev = row
    return rows[-1][0], rows[-1][1] * KT


def main():
    a, b = polar()
    print("v1 drag polar: D = %.5f q + %.1f / q  (CD0 x S = %.5f m2)" % (a, b, a))
    acf = bt.Acf(os.path.join(bt.XP_DEFAULT, bt.OUT_REL, bt.OUT_NAME + ".acf"))
    vs = [x[0] for x in LOG]
    best = []
    for diam in (10.0, 11.0, 12.0):
        for pitch in (6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0):
            pr = tp.Prop(acf, bt.XP_DEFAULT)
            k = diam / 11.0
            pr.R *= k
            pr.dr *= k
            pr.el = [(r * k, c, beta) for r, c, beta in pr.el]
            tab = prop_table(pr, pitch, vs)
            for qmax in [x / 100.0 for x in range(40, 301, 10)]:
                for dcda in [x / 10000.0 for x in range(0, 101, 10)]:
                    err = 0.0
                    out = []
                    for v, thr, ps in LOG:
                        q = 0.5 * RHO * v * v
                        d = a * q + b / q + dcda * q
                        n1, t1 = equilibrium(tab[v], qmax)
                        ps_m = (t1 - d) * v / W
                        if thr is not None:
                            lo, hi = 0.0, 1.0
                            for _ in range(12):
                                mid = 0.5 * (lo + hi)
                                if equilibrium(tab[v], mid * qmax)[1] > d:
                                    hi = mid
                                else:
                                    lo = mid
                            thr_m = 0.5 * (lo + hi)
                            err += ((thr_m - thr) / 0.05) ** 2
                        else:
                            thr_m = None
                        err += ((ps_m - ps) / 1.5) ** 2
                        out.append((v, thr_m, thr, ps_m, ps, n1))
                    best.append((err, diam, pitch, qmax, dcda, out))
    best.sort(key=lambda x: x[0])
    for err, diam, pitch, qmax, dcda, out in best[:8]:
        print("\nerr %.1f: prop %.0fx%.0f, max torque %.2f N m (%.2f hp at 12000 rpm), extra drag area %.4f m2" % (
            err, diam, pitch, qmax, qmax * 12000 * 2 * math.pi / 60 / bt.HP, dcda))
        for v, thr_m, thr, ps_m, ps, n1 in out:
            print("  %4.1f m/s: level throttle %s / log %s, Ps full %.1f / log %.1f, full-throttle rpm %.0f" % (
                v, "-" if thr_m is None else "%.2f" % thr_m, "-" if thr is None else "%.2f" % thr, ps_m, ps, n1))


if __name__ == "__main__":
    main()
