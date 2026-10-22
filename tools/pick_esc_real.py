"""Motor resistance, no-load current, blade chord and extra drag of the X-Plane T-28 with the manual's motor and prop
(EFLM4115 850 Kv, 12x7 3-blade), fitted to the logged climb and current per cell (log_targets.py). Drag in X-Plane's
frame: the bare model's polar (match_logs.py run v1, its induced part scaled to the weight) plus an extra area.
    python pick_esc_real.py [weight kg]"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mass_from_logs as ml  # noqa: E402
import pick_esc as pe  # noqa: E402
from pick_motor import polar  # noqa: E402

RHO, G = 1.225, 9.81
M = float(sys.argv[1]) if len(sys.argv) > 1 else 2.2
a, b = polar()
b *= (M / 3.0) ** 2
vs = sorted({c[0] for c in pe.CELLS})
results = []
for k in (1.2, 1.3, 1.4, 1.5, 1.6):
    pr = ml.prop(k)
    tab = {v: [(n * 2 * math.pi / 60.0, *[x * f for x, f in zip(pr.forces(max(n, 1) / 60.0, v), (pe.KT, pe.KQ))])
               for n in pe.GRID] for v in vs}
    for r in (0.05, 0.06, 0.07, 0.08, 0.10):
        for i0 in (1.0, 2.0, 3.0):
            for dc in [x / 10000.0 for x in range(-30, 41, 5)]:
                err, out = 0.0, []
                for v, g, ps, amp, vb, n in pe.CELLS:
                    q = 0.5 * RHO * v * v
                    d = a * q + b / q + dc * q
                    w, t, im = pe.equilibrium(tab[v], g, vb, 850, r, i0)
                    p = math.sqrt(min(n, 50))
                    psm = (t - d) * v / (M * G)
                    err += p * ((psm - ps) / 1.5) ** 2 + p * ((g * im + pe.I_BASE - amp) / 3.0) ** 2
                    out.append((v, g, psm, ps, g * im + pe.I_BASE, amp, w * 60 / 6.2832, n))
                results.append((err, k, r, i0, dc, out))
results.sort(key=lambda x: x[0])
for x in results[:6]:
    print("err %.0f chord x%.1f R %.2f I0 %.0f extra area %+.4f m2" % x[:5])
err, k, r, i0, dc, out = results[0]
for v, g, psm, ps, am, amp, rpm, n in out:
    print("  %4.1f throttle %.2f: Ps %+5.1f / %+5.1f | A %5.1f / %5.1f | rpm %5.0f (%d)" % (v, g, psm, ps, am, amp, rpm, n))
