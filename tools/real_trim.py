"""Elevator trim of the real T-28 against airspeed, flaps up, in steady level flight: servo[0] (1 us = 1 INAV unit) by
airspeed band and a straight line through them, per log and together. MANUAL first (the servo is the stick), ACRO as
a check (the servo holds the trim once the rates are zero). Later logs carry the flap mixer (-20 us constant with
flaps up) and a moved servo centre: the slope is unaffected, the level is not.
    LOG_DIR=<folder> python real_trim.py"""
import glob
import os

import numpy as np
import pandas as pd

LOGS = os.environ.get("LOG_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs"))
COLS = ["AirSpeed", "navVel[0]", "navVel[1]", "navVel[2]", "gyroADC[0] (deg/s)", "gyroADC[1] (deg/s)", "attitude[0]",
        "accSmooth[2]", "servo[0]", "servo[5]", "motor[0]", "flightModeFlags (flags)"]
BANDS = [(13, 16), (16, 19), (19, 22), (22, 25), (25, 28), (28, 32)]

for mode in ("MANUAL", "ACRO"):
    print("\n== %s, level (|vz| < 0.5 m/s, |roll| < 10, rates < 5 deg/s, n 0.9-1.1), flaps up" % mode)
    pooled = []
    for p in sorted(glob.glob(os.path.join(LOGS, "**", "LOG*.csv"), recursive=True)):
        d = pd.read_csv(p, usecols=lambda c: c.strip() in COLS, skipinitialspace=True)
        d.columns = [c.strip() for c in d.columns]
        d = d.iloc[::10]
        f = d["flightModeFlags (flags)"].astype(str)
        m = f.str.contains("MANUAL") if mode == "MANUAL" else ~f.str.contains("MANUAL|ANGLE|HORIZON|NAV")
        gs = np.hypot(d["navVel[0]"], d["navVel[1]"]) / 100
        d = d[f.str.contains("ARM") & m & (gs > 8) & (d["servo[5]"] < 1150) & (d["navVel[2]"].abs() < 80)
              & (d["attitude[0]"].abs() < 120) & (d["gyroADC[0] (deg/s)"].abs() < 8) & (d["gyroADC[1] (deg/s)"].abs() < 8)
              & (d["accSmooth[2]"] / 2048 > 0.85) & (d["accSmooth[2]"] / 2048 < 1.15)]
        if len(d) < 60:
            continue
        v = d.AirSpeed.to_numpy() / 100
        s = d["servo[0]"].to_numpy(float)
        name = os.path.basename(os.path.dirname(p)) + "/" + os.path.basename(p)[:8]
        cells = []
        for lo, hi in BANDS:
            k = (v >= lo) & (v < hi)
            if k.sum() >= 30:
                cells.append("%d-%d: %4.0f (%2.0f s)" % (lo, hi, np.median(s[k]), k.sum() / 100))
        a, b = np.polyfit(v, s, 1)
        print("  %-26s %s | line %+.2f us per m/s, %4.0f at 20 m/s, mean throttle %.0f %%" % (
            name, "  ".join(cells), a, a * 20 + b, (d["motor[0]"].mean() - 1000) / 10))
        pooled.append(pd.DataFrame({"v": v, "s": s - (a * 20 + b), "log": name}))
    if not pooled:
        print("  no stretch found")
        continue
    t = pd.concat(pooled)
    a, b = np.polyfit(t.v, t.s, 1)
    print("  all together (levels aligned at 20 m/s): %+.2f us per m/s, r %.2f, %d s"
          % (a, np.corrcoef(t.v, t.s)[0, 1], len(t) / 100))
    for lo, hi in BANDS:
        k = (t.v >= lo) & (t.v < hi)
        if k.sum() >= 30:
            print("    %d-%d m/s: %+5.0f against 20 m/s (%3.0f s)" % (lo, hi, t.s[k].median(), k.sum() / 100))
