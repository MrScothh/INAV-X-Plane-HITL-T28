"""Flap lift, moment and full-flap angle of the X-Plane T-28 against the logs, by a grid on the live datarefs
(sim/aircraft/controls/acf_flap_cl, _cm, _dn[n]; no reload). At 17.5 m/s, level, for each (cl, cm): the half-flap
change of elevator and body angle of attack (half detent fixed), then the full-flap change for each full angle.
Targets from the logs (real_trim.py, log_targets.py): half -50 units and -5.9 deg, full -75 and -7.2.
    python flap_grid.py [speed]      -> flap_grid.txt next to this script"""
import os
import struct
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "aircraft", "generator"))
import time

import match_logs as c

V = float(sys.argv[1]) if len(sys.argv) > 1 else 17.5
CL = (1.2, 1.5, 1.8)
CM = (-0.5, -0.63, -0.8)
FULL = (24.0, 27.0, 30.0)
HALF = 20.0
HERE = os.path.dirname(os.path.abspath(__file__))


def error(ds, da, ts, ta):
    return ((ds - ts) / 10.0) ** 2 + ((da - ta) / 0.5) ** 2


def main():
    pos = c.read_position()
    ground = pos[2] - pos[3]
    for i, n in enumerate(c.NAMES):
        c.s.sendto(b"RREF\x00" + struct.pack("<ii400s", 30, 700 + i, n.encode()), c.XP)
    c.dref("sim/operation/override/override_joystick", 1)
    for a in ("pitch", "roll", "heading"):
        c.dref("sim/joystick/joystick_%s_augment" % a, 0.0)
    lines = []

    def write(r):
        print(r, flush=True)
        lines.append(r)

    try:
        c.dref("sim/aircraft/controls/acf_flap_dn[1]", HALF)
        _, base = c.level(V, 0.0, pos, ground)
        write("== %s, %.1f m/s, flaps up: stick %+.0f, alpha %+.1f" % (time.strftime("%Y-%m-%d %H:%M"), V,
                                                                       base["stick"], base["alpha"]))
        write("cl | cm | half d stick / d alpha (log -50 / -5.9) | full per angle: d stick / d alpha (log -75 / -7.2)")
        best = []
        for cl in CL:
            for cm in CM:
                c.dref("sim/aircraft/controls/acf_flap_cl", cl)
                c.dref("sim/aircraft/controls/acf_flap_cm", cm)
                c.dref("sim/aircraft/controls/acf_flap_dn[2]", 30.0)
                _, m = c.level(V, 0.5, pos, ground)
                ds_h, da_h = m["stick"] - base["stick"], m["alpha"] - base["alpha"]
                full = []
                for dn in FULL:
                    c.dref("sim/aircraft/controls/acf_flap_dn[2]", dn)
                    _, p = c.level(V, 1.0, pos, ground)
                    ds_f, da_f = p["stick"] - base["stick"], p["alpha"] - base["alpha"]
                    full.append((dn, ds_f, da_f))
                    best.append((error(ds_h, da_h, -50, -5.9) + error(ds_f, da_f, -75, -7.2), cl, cm, dn, ds_h, da_h,
                                 ds_f, da_f))
                write("%.1f | %+.2f | %+4.0f / %+4.1f | %s" % (cl, cm, ds_h, da_h, "  ".join(
                    "%.0f: %+4.0f / %+4.1f" % x for x in full)))
        best.sort()
        for e, cl, cm, dn, ds_h, da_h, ds_f, da_f in best[:5]:
            write("error %.1f: cl %.1f cm %+.2f full %.0f deg -> half %+.0f / %+.1f, full %+.0f / %+.1f" % (
                e, cl, cm, dn, ds_h, da_h, ds_f, da_f))
    finally:
        c.dref("sim/cockpit2/controls/flap_ratio", 0.0)
        c.dref("sim/joystick/yoke_roll_ratio", 0.0)
        c.dref("sim/joystick/yoke_pitch_ratio", 0.0)
        c.dref("sim/operation/override/override_joystick", 0)
        for i, n in enumerate(c.NAMES):
            c.s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 700 + i, n.encode()), c.XP)
        with open(os.path.join(HERE, "flap_grid.txt"), "a", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n\n")


if __name__ == "__main__":
    main()
