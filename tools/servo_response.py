"""Delay from a quick servo command step to the aircraft's answer (roll or pitch acceleration from the gyro): every
step of the command bigger than the threshold within 20 ms, in flight, aligned on its start and averaged; the delay
is the time the averaged acceleration takes to reach half its peak. Real T-28 blackbox CSVs (servo[2] aileron,
servo[0] elevator, 1 kHz) or a HITL pair of logs (the outputs INAV sent, one row per frame with t_mono, roll, pitch
in servo us; the sensor log as in compare_noise.py; ~120 Hz).
    python servo_response.py real LOG.01.csv [...]
    python servo_response.py hitl outputs.csv sensors.csv"""
import sys

import numpy as np
import pandas as pd


def event_curves(t, u, g, threshold, window=0.4):
    """The acceleration after every quick command step, each on a 1 kHz grid from the step: one row per step."""
    fs = 1 / np.median(np.diff(t))
    acc = np.gradient(g, t)
    du = u[int(0.02 * fs):] - u[:-int(0.02 * fs)]
    idx = np.flatnonzero(np.abs(du) > threshold)
    events, last = [], -1e9
    for i in idx:
        if t[i] - last > 1.0:          # one per manoeuvre
            events.append(i)
            last = t[i]
    n = int(window * fs)
    curves = []
    for i in events:
        if i < int(0.05 * fs) or i + n >= len(t):
            continue
        sign = np.sign(du[i])
        base = acc[i - int(0.05 * fs):i].mean()
        # the step is somewhere inside the sample before i: start the curve at the middle of it
        tt = (np.arange(n) + 0.5) / fs
        tf = np.arange(0, window, 0.001)
        curves.append(np.interp(tf, tt, sign * (acc[i:i + n] - base)))
    return curves


def delay(curves):
    """Half-peak time of the mean curve (ms): with many steps at random phases against the sample grid the mean is
    smooth, and the crossing is read well under the sample period."""
    if len(curves) < 3:
        return None
    m = np.mean(curves, axis=0)
    peak = np.argmax(np.abs(m[:300]))
    return float(np.flatnonzero(np.abs(m[:peak + 1]) >= 0.5 * abs(m[peak]))[0])


def mean_events(t, u, g, threshold, window=0.4):
    curves = event_curves(t, u, g, threshold, window)
    return delay(curves), len(curves)


if sys.argv[1] == "real":
    pooled = {"roll": [], "pitch": []}
    for path in sys.argv[2:]:
        d = pd.read_csv(path, usecols=lambda c: c.strip() in ("time (us)", "servo[0]", "servo[2]", "gyroADC[0] (deg/s)",
                                                               "gyroADC[1] (deg/s)", "navVel[0]", "navVel[1]"))
        d.columns = [c.strip() for c in d.columns]
        t = (d["time (us)"] - d["time (us)"].iloc[0]).to_numpy() / 1e6
        ok = np.hypot(d["navVel[0]"], d["navVel[1]"]).to_numpy() / 100 > 8
        if not ok.any():
            print(f"{path}: no frames in flight, skipped")
            continue
        a, b = np.flatnonzero(ok)[[0, -1]]
        for name, s, g in (("roll", "servo[2]", "gyroADC[0] (deg/s)"), ("pitch", "servo[0]", "gyroADC[1] (deg/s)")):
            curves = event_curves(t[a:b], d[s].to_numpy(float)[a:b], d[g].to_numpy(float)[a:b], 60)
            pooled[name] += curves
            ms = delay(curves)
            print(f"{path} {name}: {len(curves)} steps, half acceleration after {ms if ms is None else round(ms, 1)} ms")
    for name, curves in pooled.items():
        ms = delay(curves)
        print(f"all logs {name}: {len(curves)} steps, half acceleration after {ms if ms is None else round(ms, 1)} ms")
else:
    r = pd.read_csv(sys.argv[2])
    s = pd.read_csv(sys.argv[3])
    t = s["t_mono"].to_numpy()
    g = s[s["gps_nuovo"] == 1]
    ok = np.interp(t, g["t_mono"], np.hypot(g["vn"], g["ve"]) / 100.0) > 8
    if not ok.any():
        raise SystemExit("no frames in flight (ground speed above 8 m/s) in %s" % sys.argv[3])
    a, b = np.flatnonzero(ok)[[0, -1]]
    t = t[a:b]
    for name, col, gyr in (("roll", "roll", "gx_16"), ("pitch", "pitch", "gy_16")):
        u = np.interp(t, r["t_mono"], r[col])
        ms, n = mean_events(t, u, s[gyr].to_numpy(float)[a:b] / 16.0, 60)
        print(f"HITL {name}: {n} steps, half acceleration after {ms if ms is None else round(ms, 1)} ms")
