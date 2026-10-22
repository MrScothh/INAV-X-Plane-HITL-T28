"""HITL sensor stream against the noise model measured on the real T-28 (t28_noise.json): gyro and accelerometer
noise in the 5-55 Hz band against throttle, barometer error against airspeed, compass field against the motor
command. In flight only (ground speed above 6 m/s).
The sensor log is a CSV with one row per HITL frame: t_mono (s), the raw MSP sensor values gx_16/gy_16/gz_16 (1/16
deg/s), ax_mg/ay_mg/az_mg (mg), mx/my/mz, baro_pa, aria (airspeed cm/s), the GPS fix vn/ve (cm/s) and alt (cm) with
gps_nuovo = 1 on the frame of a new fix. The flight CSV has INAV's throttle in gas_inav (1000-2000) at its own rate.
    python compare_noise.py sensors.csv flight.csv"""
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
s = pd.read_csv(sys.argv[1])
flight = pd.read_csv(sys.argv[2])
m = json.load(open(os.path.join(HERE, "t28_noise.json")))
t = s["t_mono"].to_numpy()
fs = 1 / np.median(np.diff(t))
g = s[s["gps_nuovo"] == 1]
gs = np.interp(t, g["t_mono"], np.hypot(g["vn"], g["ve"]) / 100.0)
# INAV's motor command from the flight CSV (10 Hz, its own clock): matched by sample fraction over the flight
throttle_csv = flight["gas_inav"].astype(float).to_numpy()
throttle = np.interp(np.linspace(0, 1, len(t)), np.linspace(0, 1, len(throttle_csv)), throttle_csv)
ok = gs > 6
a, b = np.flatnonzero(ok)[[0, -1]]
seg = slice(a, b)


def band(x, f1=5, f2=55):
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1 / fs)
    X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))


n = int(fs)
print(f"{fs:.0f} frames/s, {ok.sum() / fs:.0f} s in flight")
for name, cols, scale, unit in (("gyro", ("gx_16", "gy_16", "gz_16"), 1 / 16.0, "deg/s"),
                                ("accelerometer", ("ax_mg", "ay_mg", "az_mg"), 9.80665 / 1000, "m/s2")):
    for axis, col in zip("xyz", cols):
        x = band(s[col].to_numpy()[seg] * scale)
        gg = throttle[seg]
        std = np.array([x[k:k + n].std() for k in range(0, len(x) - n, n)])
        gm = np.array([gg[k:k + n].mean() for k in range(0, len(x) - n, n)])
        p = m[name][axis]
        expected = p["at_1000"] + p["per_100_throttle"] * np.maximum(gm - 1000, 0) / 100
        print(f"{name} {axis}: HITL {np.median(std):.3f} {unit}, logs at that throttle {np.median(expected):.3f} "
              f"(ratio {np.median(std) / np.median(expected):.2f})")
# barometric altitude from the pressure against the GPS altitude
p_pa = s["baro_pa"].to_numpy()[seg]
h_baro = 44330.0 * (1 - (p_pa / 101325.0) ** 0.190295)
h_gps = np.interp(t[seg], g["t_mono"], g["alt"] / 100.0)
err = h_baro - h_gps
v = gs[seg]
k = np.polyfit(v ** 2, err - np.median(err[v < 9]) if (v < 9).any() else err, 1)[0]
high = err - np.convolve(err, np.ones(n) / n, "same")
print(f"barometer: slope {k:+.3f} m per (m/s)^2 (logs {-0.03:+.3f}), noise above 1 Hz {high[n:-n].std():.2f} m "
      f"(logs {m['barometer']['noise_m']:.2f})")
mag = np.stack([s[c].to_numpy()[seg] / 20.0 for c in ("mx", "my", "mz")], -1)
mod = np.linalg.norm(mag, axis=1)
med = np.median(mod)
print(f"compass: |field|/median 1 % {np.percentile(mod, 1) / med:.2f}, 99 % {np.percentile(mod, 99) / med:.2f} "
      f"(real flights: 0.57-0.68 and 1.30-2.01)")
# the plugin's own current model, from INAV's motor command and the airspeed
mm = np.maximum(throttle[seg] - 1000, 0) / 1000
vv = s["aria"].to_numpy()[seg] / 100.0 / 20.0
amp = np.maximum(0.3, 3.98 - 8.08 * mm + 92.45 * mm ** 2 - 14.4 * mm ** 3 - 1.12 * vv - 14.38 * mm * vv)
real = {(0, 5): (1.03, 0.94, 1.13), (5, 10): (1.04, 0.82, 1.17), (10, 20): (0.92, 0.72, 1.25), (20, 30): (1.01, 0.57, 1.44),
        (30, 45): (0.94, 0.56, 1.64), (45, 60): (1.71, 0.92, 2.22), (60, 100): (1.69, 0.48, 2.38)}
for (lo, hi), (vm, v5, v95) in real.items():
    q = (amp >= lo) & (amp < hi)
    if q.sum() > 20:
        print(f"   current {lo}-{hi} A: HITL {np.median(mod[q]) / med:.2f} ({np.percentile(mod[q], 5) / med:.2f}-"
              f"{np.percentile(mod[q], 95) / med:.2f}), real flights {vm:.2f} ({v5:.2f}-{v95:.2f}); {q.mean() * 100:.0f} % of the flight")
