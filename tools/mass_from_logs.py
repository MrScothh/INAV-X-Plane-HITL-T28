"""Flying weight of the real T-28 from its logs, with the motor and prop of the manual (EFLM4115 850 Kv, 12x7 3-blade):
the ESC model of pick_esc.py with the blade element prop of t28_performance.py, a parabolic drag polar, and a grid of
weight, drag, blade chord, motor resistance and no-load current against the logged climb and current per cell
(log_targets.py). Result on the T-28's logs: best 2.2-2.4 kg, 2.0 and 2.6 close, 3.0 clearly worse.
    python mass_from_logs.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import build_t28 as bt  # noqa: E402
import t28_performance as tp  # noqa: E402
import pick_esc as pe  # noqa: E402

DIAM_IN, PITCH_IN, BLADES = 12.0, 7.0, 3


def prop(k_chord):
    acf = bt.Acf(os.path.join(bt.XP_DEFAULT, bt.OUT_REL, bt.OUT_NAME + ".acf"))
    pr = tp.Prop(acf, bt.XP_DEFAULT)
    k = DIAM_IN / (2 * pr.R / 0.0254)
    pr.R *= k
    pr.dr *= k
    pr.B = BLADES
    pr.el = [(r * k, c * k * k_chord, math.degrees(math.atan(PITCH_IN * 0.0254 / (2 * math.pi * r * k))))
             for r, c, _ in pr.el]
    return pr


def main():
    RHO, G, S, B = 1.225, 9.81, 0.2387, 1.224
    A = B * B / S
    vs = sorted({c[0] for c in pe.CELLS})
    tabs = {}
    for k in (0.8, 1.0, 1.2, 1.4):
        pr = prop(k)
        rows = {}
        for v in vs:
            rows[v] = [(n * 2 * math.pi / 60.0, *[x * f for x, f in zip(pr.forces(max(n, 1) / 60.0, v), (pe.KT, pe.KQ))])
                       for n in pe.GRID]
        tabs[k] = rows
    results = []
    for k, tab in tabs.items():
        for r in (0.03, 0.05, 0.07, 0.10):
            for i0 in (1.0, 2.0):
                for m in (1.7, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0):
                    for cd in [x / 1000 for x in range(30, 101, 5)]:
                        err = 0.0
                        for v, g, ps, amp, vb, n in pe.CELLS:
                            q = 0.5 * RHO * v * v
                            d = cd * q * S + (m * G) ** 2 / (q * S * math.pi * A * 0.8)
                            w, t, im = pe.equilibrium(tab[v], g, vb, 850, r, i0)
                            p = math.sqrt(min(n, 50))
                            err += p * ((((t - d) * v / (m * G)) - ps) / 1.5) ** 2 + p * ((g * im + pe.I_BASE - amp) / 3.0) ** 2
                        results.append((err, k, r, i0, m, cd))
    results.sort()
    for x in results[:8]:
        print("err %.0f chord x%.1f R %.2f I0 %.0f mass %.1f CD0 %.3f" % x)
    print("best per mass:")
    for m in (1.7, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0):
        b = min(x for x in results if x[4] == m)
        print("  %.1f kg: err %.0f (chord x%.1f R %.2f I0 %.0f CD0 %.3f)" % (m, b[0], b[1], b[2], b[3], b[5]))
    err, k, r, i0, m, cd = results[0]
    for v, g, ps, amp, vb, n in pe.CELLS:
        q = 0.5 * RHO * v * v
        d = cd * q * S + (m * G) ** 2 / (q * S * math.pi * A * 0.8)
        w, t, im = pe.equilibrium(tabs[k][v], g, vb, 850, r, i0)
        print("  %4.1f throttle %.2f: Ps %+5.1f / %+5.1f | A %5.1f / %5.1f | rpm %5.0f (%d)"
              % (v, g, (t - d) * v / (m * G), ps, g * im + pe.I_BASE, amp, w * 60 / 6.2832, n))


if __name__ == "__main__":
    main()
