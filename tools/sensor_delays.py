"""Relative timing of the sensor streams the board receives in HITL, from a sensor log (CSV as in compare_noise.py,
plus X-Plane's attitude roll_xp/pitch_xp/yaw_xp in decidegrees): how much later the GPS shows a manoeuvre than the
accelerometer, the gyro and X-Plane's attitude (positive = GPS later). Only frames in flight (ground speed above
8 m/s) count. The accelerometer's axes and signs are not assumed: every sign combination is tried and the one whose
horizontal acceleration best matches the GPS one is kept.
    python sensor_delays.py sensors.csv"""
import itertools
import sys

import numpy as np
import pandas as pd

d = pd.read_csv(sys.argv[1])
t = d["t_mono"].to_numpy()
g = d[d["gps_nuovo"] == 1]
tg = g["t_mono"].to_numpy()
vn, ve = g["vn"].to_numpy() / 100.0, g["ve"].to_numpy() / 100.0
gs = np.hypot(vn, ve)
flying = gs[1:-1] > 8
# GPS acceleration and course rate by central differences, at the middle sample's time
dt = tg[2:] - tg[:-2]
aN, aE = (vn[2:] - vn[:-2]) / dt, (ve[2:] - ve[:-2]) / dt
course = np.unwrap(np.arctan2(ve, vn))
course_rate = np.degrees(course[2:] - course[:-2]) / dt
tm = tg[1:-1]

phi, theta, psi = (np.radians(d[c].to_numpy() / 10.0) for c in ("roll_xp", "pitch_xp", "yaw_xp"))
acc = d[["ax_mg", "ay_mg", "az_mg"]].to_numpy() / 1000.0 * 9.80665
gz = d["gz_16"].to_numpy() / 16.0
psi_u = np.unwrap(psi)
yaw_rate = np.degrees(np.gradient(psi_u, t))


def window(x, tt, centres, width=0.2):
    """Mean of x over a window centred on each time: the IMU seen at the GPS's rate."""
    c = np.concatenate(([0.0], np.cumsum(x)))
    a = np.searchsorted(tt, centres - width / 2)
    b = np.searchsorted(tt, centres + width / 2)
    n = np.maximum(b - a, 1)
    return (c[b] - c[a]) / n


def best_delay(gps, imu, tt, delays):
    """Delay (s) that best lines the GPS series up with the IMU one, and the correlation there."""
    r = []
    for L in delays:
        x = window(imu, tt, tm - L)
        a, b = gps[flying], x[flying]
        r.append(np.corrcoef(a, b)[0, 1])
    k = int(np.nanargmax(r))
    return delays[k], r[k]


delays = np.arange(-0.4, 0.6001, 0.005)
L, c = best_delay(course_rate, yaw_rate, t, delays)
print(f"GPS against X-Plane's attitude (course against heading): {L * 1000:+.0f} ms (correlation {c:.3f})")
L, c = best_delay(course_rate, gz, t, delays)
L2, c2 = best_delay(course_rate, -gz, t, delays)
L, c = (L, c) if c >= c2 else (L2, c2)
print(f"GPS against the gyro (course against gz):                {L * 1000:+.0f} ms (correlation {c:.3f})")

best = None
for sx, sy, sz, sp, sr in itertools.product((1, -1), repeat=5):
    f = acc * np.array([sx, sy, sz])
    th, ph = sp * theta, sr * phi
    cp, sp_, ct, st, cs, ss = np.cos(ph), np.sin(ph), np.cos(th), np.sin(th), np.cos(psi), np.sin(psi)
    # body (x forward, y right, z down) to NED, aerospace order yaw-pitch-roll
    fN = ct * cs * f[:, 0] + (sp_ * st * cs - cp * ss) * f[:, 1] + (cp * st * cs + sp_ * ss) * f[:, 2]
    fE = ct * ss * f[:, 0] + (sp_ * st * ss + cp * cs) * f[:, 1] + (cp * st * ss - sp_ * cs) * f[:, 2]
    LN, cN = best_delay(aN, fN, t, delays[::4])
    LE, cE = best_delay(aE, fE, t, delays[::4])
    if best is None or cN + cE > best[0]:
        best = (cN + cE, (sx, sy, sz, sp, sr), fN, fE)
_, signs, fN, fE = best
LN, cN = best_delay(aN, fN, t, delays)
LE, cE = best_delay(aE, fE, t, delays)
print(f"GPS against the accelerometer (signs {signs}): north {LN * 1000:+.0f} ms (corr {cN:.3f}), "
      f"east {LE * 1000:+.0f} ms (corr {cE:.3f})")
print(f"GPS samples in flight {flying.sum()}, frames {len(t)}, GPS {len(tg) / (tg[-1] - tg[0]):.1f}/s")
