"""What the real T-28 does in its blackbox logs, as targets for the X-Plane model. Steady level flight is rare in
these logs, so the targets use energy and attitude over 1 s windows of any flight with a bank under 40 degrees:
- specific excess power Ps = climb rate + V dV/dt / g (m/s) per airspeed band and throttle band: thrust minus drag
  over weight, what the model must give at the same speed and throttle;
- body angle of attack (pitch minus flight path angle) against load factor / V^2: the slope is weight over lift slope
  and wing area, the shift between flap settings is what the flaps do to the attitude;
- elevator servo for the same, per flap setting.
Airspeed is INAV's virtual one (GPS speed and wind estimate). Pitch: attitude[1] is positive nose down.
    LOG_DIR=<folder> python log_targets.py [log.csv ...]"""
import glob
import math
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
# folder of the blackbox CSVs (blackbox_decode --unit-rotation deg/s --merge-gps): LOG_DIR in the environment
LOGS = os.environ.get("LOG_DIR", os.path.join(HERE, "..", "logs"))
CACHE = os.path.join(HERE, "tmp", "targets_cache")
COLS = ["time (us)", "navVel[0]", "navVel[1]", "navVel[2]", "AirSpeed", "attitude[0]", "attitude[1]", "motor[0]",
        "amperage (A)", "vbat (V)", "servo[0]", "servo[2]", "servo[5]", "accSmooth[2]", "flightModeFlags (flags)"]
STEP = 20           # 1 kHz logs -> 50 Hz
WINDOW = 50         # 1 s
G = 9.81
V_BANDS = np.arange(10, 36, 3)
THROTTLE_BANDS = [-1, 10, 30, 50, 70, 90, 101]


def load(path):
    os.makedirs(CACHE, exist_ok=True)
    cache = os.path.join(CACHE, os.path.basename(os.path.dirname(path)) + "_" + os.path.basename(path) + ".pkl")
    if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(path):
        return pd.read_pickle(cache)
    d = pd.read_csv(path, usecols=lambda c: c.strip() in COLS, skipinitialspace=True)
    d.columns = [c.strip() for c in d.columns]
    d = d.iloc[::STEP].reset_index(drop=True)
    f = d["flightModeFlags (flags)"].astype(str)
    out = pd.DataFrame({
        "t": (d["time (us)"] - d["time (us)"].iloc[0]) / 1e6,
        "gs": np.hypot(d["navVel[0]"], d["navVel[1]"]) / 100.0,
        "vz": d["navVel[2]"] / 100.0,
        "v": d["AirSpeed"] / 100.0,
        "roll": d["attitude[0]"] / 10.0,
        "pitch": d["attitude[1]"] / 10.0,
        "motor": d["motor[0]"],
        "amp": d["amperage (A)"],
        "volt": d["vbat (V)"],
        "elev": d["servo[0]"],
        "nz": d["accSmooth[2]"] / 2048.0,
        "flap": pd.cut(d["servo[5]"], [0, 1150, 1650, 2100], labels=["up", "half", "down"]),
        "armed": f.str.contains("ARM"),
        "mode": np.where(f.str.contains("MANUAL"), "MANUAL", np.where(f.str.contains("NAV"), "NAV",
                         np.where(f.str.contains("ANGLE|HORIZON"), "ANGLE", "ACRO"))),
    })
    out["log"] = os.path.basename(os.path.dirname(path)) + "/" + os.path.basename(path)[:8]
    out.to_pickle(cache)
    return out


def windows(d):
    """1 s windows every 0.5 s: means, the airspeed slope, the largest bank and the spread of throttle and pitch."""
    r = d[["gs", "vz", "v", "roll", "pitch", "motor", "amp", "volt", "elev", "nz"]].rolling(WINDOW, center=True)
    m = r.mean()
    s = r.std()
    tt = d.t - d.t.mean()
    cov = (tt * d.v).rolling(WINDOW, center=True).mean() - tt.rolling(WINDOW, center=True).mean() * m.v
    var = (tt * tt).rolling(WINDOW, center=True).mean() - tt.rolling(WINDOW, center=True).mean() ** 2
    w = pd.DataFrame({"v": m.v, "dvdt": cov / var, "gs": m.gs, "vz": m.vz, "theta": -m.pitch, "motor": m.motor,
                      "amp": m.amp, "volt": m.volt, "elev": m.elev, "nz": m.nz, "roll": m.roll.abs(),
                      "roll_max": d.roll.abs().rolling(WINDOW, center=True).max(), "motor_sd": s.motor,
                      "theta_sd": s.pitch, "flap": d.flap, "armed": d.armed, "mode": d.mode, "log": d.log})
    w = w.iloc[::25].dropna(subset=["v", "motor", "dvdt"])
    w["throttle"] = (w.motor - 1000) / 10.0
    w["ps"] = w.vz + w.v * w.dvdt / G
    w["n"] = w.nz
    w["alpha"] = w.theta - np.degrees(np.arcsin(np.clip(w.vz / w.v.clip(lower=1), -1, 1)))
    w["x"] = w.n / (w.v ** 2) * 100.0     # load factor / V^2, x100 (m/s)^-2
    return w


def line(x, y):
    a, b = np.polyfit(x, y, 1)
    return a, b, np.corrcoef(x, y)[0, 1]


def main():
    paths = sys.argv[1:] or sorted(glob.glob(os.path.join(LOGS, "**", "LOG*.csv"), recursive=True))
    d = pd.concat([windows(load(p)) for p in paths], ignore_index=True)
    print("logs:", ", ".join(sorted(d.log.unique())))
    air = d[d.armed & (d.gs > 7) & (d.v > 8) & (d.roll_max < 40) & (d.motor_sd < 60)]
    print("1 s windows in flight with bank under 40 degrees and steady throttle: %d (%.0f s)" % (len(air), len(air) * 0.5))

    up = air[air.flap == "up"]
    t = up.assign(bv=pd.cut(up.v, V_BANDS), bg=pd.cut(up.throttle, THROTTLE_BANDS)).pivot_table(
        index="bv", columns="bg", values="ps", aggfunc=["median", "count"], observed=False)
    print("\n== Ps (m/s), flaps up: median per airspeed (rows) and throttle % (columns); below, window counts")
    print(t["median"].round(2).to_string())
    print(t["count"].to_string())
    t = up.assign(bv=pd.cut(up.v, V_BANDS), bg=pd.cut(up.throttle, THROTTLE_BANDS)).pivot_table(
        index="bv", columns="bg", values=["amp", "volt", "throttle"], aggfunc="median", observed=False)
    print("\n== Current (A), voltage (V) and mean throttle (%) in the same cells")
    print(t["amp"].round(1).to_string())
    print(t["volt"].round(2).to_string())
    print(t["throttle"].round(0).to_string())

    print("\n== Body angle of attack (pitch minus flight path angle) against n/V^2 x 100")
    calm = air[(air.theta_sd < 3) & (air.dvdt.abs() < 1.0)]
    for fl in ("up", "half", "down"):
        c = calm[calm.flap == fl]
        if len(c) < 20:
            print("  flaps %-4s: %d windows, too few" % (fl, len(c)))
            continue
        a, b, r = line(c.x, c.alpha)
        a14 = a * 100.0 / 14 ** 2 + b
        a24 = a * 100.0 / 24 ** 2 + b
        print("  flaps %-4s: %4d windows, slope %.2f degrees per (n/V^2 x 100), intercept %+.2f, r %.2f; "
              "at 14 m/s %+.1f degrees, at 24 m/s %+.1f; median elevator %d us"
              % (fl, len(c), a, b, r, a14, a24, c.elev.median()))
    c = calm[calm.flap == "up"]
    a, b, r = line(c.x, c.alpha)
    # slope = (2 m g / (rho S a_w)) in degrees per (1/V^2) / 100 -> mass for a given lift slope
    rho, S = 1.20, 0.2387
    for aw in (4.0, 4.5, 5.0):
        m = math.radians(a) * 100.0 * rho * S * aw / (2 * G)
        print("  mass that would give this slope with a lift slope of %.1f /rad: %.2f kg" % (aw, m))

    # within one speed band the load factor comes from the accelerometer, so wind errors in V do not flatten the slope
    print("\n== Angle of attack against load factor, flaps up, per airspeed band (turns and pull-ups)")
    for lo, hi in ((15, 19), (19, 23), (23, 27)):
        c = calm[(calm.flap == "up") & (calm.v >= lo) & (calm.v < hi)]
        if len(c) < 30 or c.n.std() < 0.05:
            continue
        a, b, r = line(c.n, c.alpha)
        v = c.v.median()
        for aw in (4.5,):
            m = math.radians(a) * 0.5 * rho * v * v * S * aw / G
        print("  %d-%d m/s: %3d windows, n from %.2f to %.2f, %.2f degrees per g (r %.2f): %.2f kg with a lift slope of 4.5 /rad"
              % (lo, hi, len(c), c.n.quantile(.05), c.n.quantile(.95), a, r, m))

    print("\n== Electrical power (W): near-level flight (|Ps| < 1) and full throttle, flaps up; mass = eta dP / (g Ps)")
    for lo, hi in ((16, 20), (20, 24), (24, 28)):
        b = up[(up.v >= lo) & (up.v < hi)]
        lvl = b[b.ps.abs() < 1.0]
        full = b[b.throttle > 90]
        if len(lvl) < 5 or len(full) < 3:
            continue
        dp = (full.amp * full.volt).median() - (lvl.amp * lvl.volt).median()
        ps = full.ps.median() - lvl.ps.median()
        print("  %d-%d m/s: level %.0f W (throttle %.0f %%), full %.0f W, Ps %.1f m/s: mass %.2f kg with eta 0.5, %.2f with 0.6"
              % (lo, hi, (lvl.amp * lvl.volt).median(), lvl.throttle.median(), (full.amp * full.volt).median(), ps,
                 0.5 * dp / (G * ps), 0.6 * dp / (G * ps)))

    print("\n== Airspeed per flap setting, throttle under 10 %, and the lowest seen (5 %)")
    for fl in ("up", "half", "down"):
        c = air[(air.flap == fl)]
        print("  flaps %-4s: median %.1f m/s, 5 %% %.1f m/s, minimum %.1f m/s (%d windows)"
              % (fl, c.v.median(), c.v.quantile(.05), c.v.min(), len(c)))


if __name__ == "__main__":
    main()
