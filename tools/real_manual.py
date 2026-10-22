"""Real T-28 in MANUAL: the airframe's rate per unit of INAV output (in MANUAL the output is rcCommand at
manual_*_rate 100 %, 1 unit = 1 us of servo), per airspeed band, from steady stretches (rate changing slowly, output
away from zero). The same number the HITL MANUAL steps give for the X-Plane model.
    python real_manual.py log.csv ..."""
import os
import sys

import numpy as np
import pandas as pd

COLS = ["AirSpeed", "navVel[0]", "navVel[1]", "gyroADC[0] (deg/s)", "gyroADC[1] (deg/s)", "rcCommand[0]", "rcCommand[1]",
        "attitude[0]", "servo[5]", "flightModeFlags (flags)"]
LIM_ROLL = int(os.environ.get("LIM_ROLL", "300"))  # decidegrees of bank for the pitch stretches
parts = []
for p in sys.argv[1:]:
    d = pd.read_csv(p, usecols=lambda c: c.strip() in COLS)
    d.columns = [c.strip() for c in d.columns]
    for ax in (0, 1):
        d[f"acc{ax}"] = d[f"gyroADC[{ax}] (deg/s)"].diff(100).abs()
        d[f"cmd_steady{ax}"] = d[f"rcCommand[{ax}]"].diff(300).abs() < 8
    f = d["flightModeFlags (flags)"].astype(str)
    d = d[f.str.contains("ARM") & f.str.contains("MANUAL") & (np.hypot(d["navVel[0]"], d["navVel[1]"]) > 1000)
          & (d.AirSpeed > 1000) & (d["servo[5]"] < 1150)]
    parts.append(d)
d = pd.concat(parts)
print(f"MANUAL in flight with flaps up: {len(d) / 1000:.0f} s")
for ax, name in ((0, "roll"), (1, "pitch")):
    for lo, hi in ((12, 18), (18, 22), (22, 26), (26, 32)):
        b = d[(d.AirSpeed >= lo * 100) & (d.AirSpeed < hi * 100)]
        s = b[b[f"cmd_steady{ax}"] & (b[f"acc{ax}"] < 15) & (b[f"rcCommand[{ax}]"].abs() > 20)
              & (b[f"gyroADC[{ax}] (deg/s)"].abs() > 5)]
        if ax == 1:
            s = s[s["attitude[0]"].abs() < LIM_ROLL]
        k = (s[f"gyroADC[{ax}] (deg/s)"] / s[f"rcCommand[{ax}]"]).abs()
        if len(k) > 300:
            print(f"  {name:10s} {lo}-{hi} m/s: {k.median():.3f} deg/s per unit "
                  f"(p25-75 {k.quantile(.25):.3f}-{k.quantile(.75):.3f}, {len(k) / 1000:.1f} s)")
