#!/usr/bin/env python3
"""Performance estimate of the T-28 RC in X-Plane, calibrated on a HITL flight.

Model: blade element propeller (geometry and airfoil read from the ACF and its .afl), drag by components (wing and
tail profile drag from the same .afl polar X-Plane uses, at their Reynolds number; bodies = cd x frontal area;
induced drag with e = 0.85). X-Plane's electric motor is undocumented: two laws are tried,
A power = throttle x Pmax, B torque = throttle x Pmax / omega_max.
Calibration: a flight flown with a known ACF gives, for each law, the thrust factor (X-Plane / this model) and the
profile drag factor; other ACFs are then evaluated with the same factors.
The flight CSV has one row per frame with t_s, vel_aria (airspeed m/s), gas_inav (INAV throttle 1000-2000),
quota_agl (height m) and armato (1 while armed), as the HITL flight recorder of this project writes it.

    python t28_performance.py                        # calibrate on flight 1, evaluate the generated ACF
    python t28_performance.py --explore              # grid of fuselage cd x power (law B)
    python t28_performance.py --flight flights/x.csv --acf-flight flights/x.acf
"""
import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import build_t28 as bt  # noqa: E402

RHO = 1.225
NU = 1.46e-5
G = 9.80665
E_OSW = 0.85
MU_ROLL = 0.025       # acf/_roll_co of the earlier model
HERE = os.path.dirname(os.path.abspath(__file__))
FLIGHT_DEF = os.path.join(HERE, "flights", "t28_flight1.csv")
ACF_FLIGHT_DEF = os.path.join(HERE, "flights", "T28_RC_flight1.acf")

_afl_cache = {}


def read_afl(path):
    """Blocks (Re, [(alpha, cl, cd)]) of an X-Plane 11 .afl (Re in millions on the parameter line)."""
    if path in _afl_cache:
        return _afl_cache[path]
    blocks = []
    lines = open(path, encoding="latin-1").read().split("\n")
    i = 0
    while i < len(lines):
        if lines[i].startswith("alpha"):
            re_m = float(lines[i - 1].split()[0])
            rows = []
            i += 1
            while i < len(lines) and len(lines[i].split()) == 4:
                a, cl, cd, _ = (float(x) for x in lines[i].split())
                rows.append((a, cl, cd))
                i += 1
            blocks.append((re_m * 1e6, rows, [r[0] for r in rows]))
        else:
            i += 1
    _afl_cache[path] = blocks
    return blocks


def polar(blocks, alpha_deg, re):
    alpha_deg = (alpha_deg + 180.0) % 360.0 - 180.0

    def at(rows, alphas):
        # non-uniform step: 1 degree far from zero, 0.1 near it
        j = max(0, min(len(rows) - 2, bisect.bisect_right(alphas, alpha_deg) - 1))
        f = (alpha_deg - alphas[j]) / (alphas[j + 1] - alphas[j])
        return [rows[j][n] * (1 - f) + rows[j + 1][n] * f for n in (1, 2)]
    if re <= blocks[0][0]:
        return at(blocks[0][1], blocks[0][2])
    if re >= blocks[-1][0]:
        return at(blocks[-1][1], blocks[-1][2])
    for (r0, b0, a0), (r1, b1, a1) in zip(blocks, blocks[1:]):
        if r0 <= re <= r1:
            f = (math.log(re) - math.log(r0)) / (math.log(r1) - math.log(r0))
            p0, p1 = at(b0, a0), at(b1, a1)
            return [p0[n] * (1 - f) + p1[n] * f for n in (0, 1)]


def afl(xp, name):
    return read_afl(os.path.join(xp, "Airfoils", name))


class Prop:
    def __init__(self, acf, xp):
        self.B = acf.f("_prop/0/_num_blades")
        self.R = acf.f("_wing/0/_semilen_SEG") * bt.FT
        self.el = []
        for i in range(10):
            r = (i + 0.5) * self.R / 10.0
            c = 0.5 * (acf.f("_wing/0/_chord_for_RIB/%d" % i) + acf.f("_wing/0/_chord_for_RIB/%d" % (i + 1))) * bt.FT
            self.el.append((r, c, acf.f("_wing/0/_incidence/%d" % i)))
        self.dr = self.R / 10.0
        self.afl = afl(xp, acf.get("_wing/0/_afl_file_T0"))
        self.kt = 1.0
        self.pmax = acf.f("acf/_power_max_limit") * bt.HP
        self.n_red = acf.f("acf/_RSC_redline_ENGN") / 60.0

    def forces(self, n, V):
        """Thrust (N) and torque (N m) at n rev/s and airspeed V, with axial induction and tip loss."""
        w = 2 * math.pi * n
        T = Q = 0.0
        for r, c, beta in self.el:
            vi = 0.0
            for _ in range(60):
                va = V + vi
                vt = w * r
                phi = math.atan2(va, vt)
                W2 = va * va + vt * vt
                cl, cd = polar(self.afl, beta - math.degrees(phi), math.sqrt(W2) * c / NU)
                dT = 0.5 * RHO * W2 * c * self.B * (cl * math.cos(phi) - cd * math.sin(phi)) * self.dr
                f = self.B / 2.0 * (self.R - r) / (r * max(math.sin(phi), 1e-3))
                F = max(2 / math.pi * math.acos(min(1.0, math.exp(-f))), 0.05)
                vi_new = dT / (4 * math.pi * r * RHO * F * max(abs(va), 0.5) * self.dr)
                if abs(vi_new - vi) < 1e-4:
                    vi = vi_new
                    break
                vi = 0.5 * vi + 0.5 * vi_new
            va = V + vi
            phi = math.atan2(va, w * r)
            W2 = va * va + (w * r) ** 2
            cl, cd = polar(self.afl, beta - math.degrees(phi), math.sqrt(W2) * c / NU)
            T += 0.5 * RHO * W2 * c * self.B * (cl * math.cos(phi) - cd * math.sin(phi)) * self.dr
            Q += 0.5 * RHO * W2 * c * self.B * (cl * math.sin(phi) + cd * math.cos(phi)) * r * self.dr
        return T, Q

    def equilibrium(self, V, thr, law, pmax=None):
        """Steady rpm and thrust (corrected by the calibration factor): motor torque = prop torque."""
        pmax = self.pmax if pmax is None else pmax
        if thr <= 0:
            lo, hi = 1.0, 400.0          # motor free: windmilling prop at zero torque
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                if self.forces(mid, V)[1] > 0:
                    hi = mid
                else:
                    lo = mid
            return lo, self.kt * self.forces(lo, V)[0]
        wred = 2 * math.pi * self.n_red
        lo, hi = 1.0, 600.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            qm = thr * pmax / (2 * math.pi * mid) if law == "A" else thr * pmax / wred
            if self.forces(mid, V)[1] > qm:
                hi = mid
            else:
                lo = mid
        return lo, self.kt * self.forces(lo, V)[0]


def section_area(acf, p):
    s, r = int(acf.get("_part/%d/_s_dim" % p)), int(acf.get("_part/%d/_r_dim" % p))
    best = 0.0
    for i in range(s):
        pts = [(acf.f("_part/%d/_geo_xyz/%d,%d,0" % (p, i, j)), acf.f("_part/%d/_geo_xyz/%d,%d,1" % (p, i, j)))
               for j in range(r)]
        best = max(best, 0.5 * abs(sum(pts[k][0] * pts[k + 1][1] - pts[k + 1][0] * pts[k][1]
                                       for k in range(-1, len(pts) - 1))))
    return best * bt.FT ** 2


class Airframe:
    def __init__(self, acf, xp, s_cal=1.0, cd_fus=None):
        self.m = acf.f("acf/_m_max") * bt.LB
        self.S = bt.wing_area_ft2(acf) * bt.FT ** 2
        self.b = bt.span_ft(acf) * bt.FT
        self.s_cal = s_cal
        self.surf = []      # (planform area, mean chord, polar)
        for p in range(8, 20):
            if acf.part_eq(p):
                sw = math.radians(acf.f("_wing/%d/_sweep_design" % p))
                c = 0.5 * (acf.f("_wing/%d/_Croot" % p) + acf.f("_wing/%d/_Ctip" % p)) * bt.FT
                self.surf.append((acf.f("_wing/%d/_semilen_SEG" % p) * math.cos(sw) * bt.FT * c, c,
                                  afl(xp, acf.get("_wing/%d/_afl_file_R0" % p))))
        self.a_fus = section_area(acf, 56)
        self.cd_fus = acf.f("_part/56/_part_cd") if cd_fus is None else cd_fus
        # the earlier model had a nacelle part; the T-28 built from the blank has none
        self.f_nac = section_area(acf, 77) * acf.f("_part/77/_part_cd") if acf.part_eq(77) else 0.0

    def f_profile(self, V, raw=False):
        f = sum(a * polar(pl, 0.0, max(V, 1.0) * c / NU)[1] for a, c, pl in self.surf)
        return f if raw else self.s_cal * f

    def f_bodies(self):
        return self.cd_fus * self.a_fus + self.f_nac

    def cd0(self, V):
        return (self.f_profile(V) + self.f_bodies()) / self.S

    def drag(self, V):
        q = 0.5 * RHO * V * V
        cl = self.m * G / (q * self.S)
        return q * self.S * (self.cd0(V) + cl * cl / (math.pi * self.b ** 2 / self.S * E_OSW))


def level_throttle(prop, af, V, law, pmax=None):
    D = af.drag(V)
    lo, hi = 0.0, 1.5
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        if prop.equilibrium(V, mid, law, pmax)[1] > D:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def top_speed(prop, af, law, pmax=None):
    lo, hi = 8.0, 70.0
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        if prop.equilibrium(mid, 1.0, law, pmax)[1] > af.drag(mid):
            lo = mid
        else:
            hi = mid
    return lo


def glide(prop, af, V):
    T = prop.equilibrium(V, 0.0, "A")[1]
    return af.m * G / (af.drag(V) - T)


def calibrate(xp, log_path, acf_path):
    """For each law: (thrust factor, profile factor) from the longest level stretch and from the take-off run."""
    acf = bt.Acf(acf_path)
    prop = Prop(acf, xp)
    af = Airframe(acf, xp)
    rows = [r for r in csv.DictReader(open(log_path)) if r["armato"] == "1"]
    t = [float(r["t_s"]) for r in rows]
    v = [float(r["vel_aria"]) for r in rows]
    g = [(float(r["gas_inav"]) - 1000) / 1000.0 for r in rows]
    h = [float(r["quota_agl"]) for r in rows]
    best = None
    for i in range(len(t)):
        j = i
        while j + 1 < len(t) and abs(g[j + 1] - g[i]) < 0.01 and abs(h[j + 1] - h[i]) < 1.5 and h[i] > 20:
            j += 1
        if best is None or t[j] - t[i] > best[1] - best[0]:
            best = (t[i], t[j], i, j)
    i, j = best[2], best[3]
    Vc = sum(v[i:j + 1]) / (j - i + 1)
    gc = sum(g[i:j + 1]) / (j - i + 1)
    vs = (h[j] - h[i]) / (t[j] - t[i])
    pts = []
    for v0 in (8, 12, 16, 20, 24):
        k = min(range(len(v)), key=lambda x: abs(v[x] - v0) + (100 if h[x] > 0.2 or g[x] < 0.7 else 0))
        k0, k1 = max(k - 2, 0), min(k + 2, len(v) - 1)
        pts.append((v[k], g[k], (v[k1] - v[k0]) / (t[k1] - t[k0])))
    out = {"cruise": (t[i], t[j], Vc, gc, vs)}
    q_c = 0.5 * RHO * Vc * Vc
    m, S = af.m, af.S
    for law in ("A", "B"):
        Tc = prop.equilibrium(Vc, gc, law)[1]
        Tr = [prop.equilibrium(vv, gg, law)[1] for vv, gg, _ in pts]
        # unknowns kt and CD0 (at cruise speed); on the take-off run the drag is taken at constant CD0
        cl = m * G / (q_c * S)
        di = q_c * S * cl * cl / (math.pi * af.b ** 2 / S * E_OSW)
        eqs = [(Tc, -q_c * S, di + m * G * vs / Vc)]
        for (vv, gg, acc), tr in zip(pts, Tr):
            eqs.append((tr, -0.5 * RHO * vv * vv * S, m * acc + MU_ROLL * m * G))
        a11 = sum(e[0] * e[0] for e in eqs)
        a12 = sum(e[0] * e[1] for e in eqs)
        a22 = sum(e[1] * e[1] for e in eqs)
        b1 = sum(e[0] * e[2] for e in eqs)
        b2 = sum(e[1] * e[2] for e in eqs)
        det = a11 * a22 - a12 * a12
        kt = (b1 * a22 - b2 * a12) / det
        cd0 = (a11 * b2 - a12 * b1) / det
        res = [e[0] * kt + e[1] * cd0 - e[2] for e in eqs]
        s_cal = (cd0 * S - af.f_bodies()) / af.f_profile(Vc, raw=True)
        out[law] = (kt, cd0, s_cal, res)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xp", default=bt.XP_DEFAULT)
    ap.add_argument("--flight", default=FLIGHT_DEF)
    ap.add_argument("--acf-flight", default=ACF_FLIGHT_DEF)
    ap.add_argument("--acf", help="ACF to evaluate (default: the generated one)")
    ap.add_argument("--explore", action="store_true", help="grid of fuselage cd x power, law B")
    a = ap.parse_args()
    cal = calibrate(a.xp, a.flight, a.acf_flight)
    t0, t1, Vc, gc, vs = cal["cruise"]
    print("calibration on %s: cruise %.1f-%.1f s at %.1f m/s, throttle %.3f, climb rate %.2f m/s"
          % (os.path.basename(a.flight), t0, t1, Vc, gc, vs))
    for law in ("A", "B"):
        kt, cd0, s_cal, res = cal[law]
        print(" law %s: thrust factor %.3f, CD0 of the aircraft in flight %.4f, profile factor %.2f, residuals %s N"
              % (law, kt, cd0, s_cal, " ".join("%.1f" % x for x in res)))

    acf = bt.Acf(a.acf or os.path.join(a.xp, bt.OUT_REL, bt.OUT_NAME + ".acf"))
    prop = Prop(acf, a.xp)
    print("ACF evaluated: wing %.4f m2, span %.3f m, mass %.2f kg, Pmax %.0f W, redline %.0f rpm, "
          "fuselage cd %.3f on %.4f m2"
          % (bt.wing_area_ft2(acf) * bt.FT ** 2, bt.span_ft(acf) * bt.FT, acf.f("acf/_m_max") * bt.LB,
             prop.pmax, prop.n_red * 60, acf.f("_part/56/_part_cd"), section_area(acf, 56)))

    if a.explore:
        prop.kt = cal["B"][0]
        print("law B. fus cd | Pmax W | CD0(18) | throttle 18 m/s | v max | glide 14 m/s | static/weight")
        for cdf in (0.065, 0.2, 0.3, 0.4):
            af = Airframe(acf, a.xp, cal["B"][2], cdf)
            ge = glide(prop, af, 14.0)
            for pw in (250, 300, 350, 400):
                print("  %.3f  %5.0f |  %.4f | %8.0f %% | %5.1f | %14.1f | %.2f" % (
                    cdf, pw, af.cd0(18.0), level_throttle(prop, af, 18.0, "B", pw) * 100,
                    top_speed(prop, af, "B", pw), ge, prop.equilibrium(0.0, 1.0, "B", pw)[1] / (af.m * G)))
        return

    for law in ("A", "B"):
        prop.kt = cal[law][0]
        af = Airframe(acf, a.xp, cal[law][2])
        print("law %s (thrust factor %.3f, profile %.2f)" % (law, prop.kt, af.s_cal))
        print("  CD0: %s" % "  ".join("%d m/s %.4f" % (V, af.cd0(V)) for V in (12, 18, 25, 32)))
        for V in (12, 15, 18, 21, 24, 27, 30):
            thr = level_throttle(prop, af, V, law)
            n, _ = prop.equilibrium(V, min(thr, 1.0), law)
            print("  level flight at %2d m/s: throttle %4.0f %%  (%5.0f rpm)" % (V, thr * 100, n * 60))
        print("  top speed at full throttle: %.1f m/s" % top_speed(prop, af, law))
        n0, T0 = prop.equilibrium(0.0, 1.0, law)
        print("  static thrust at full throttle: %.1f N (%.2f of the weight), %.0f rpm" % (T0, T0 / (af.m * G), n0 * 60))
        print("  glide ratio at idle: %s" % "  ".join("%d m/s %.1f" % (V, glide(prop, af, V)) for V in (11, 13, 15, 18)))


if __name__ == "__main__":
    main()
