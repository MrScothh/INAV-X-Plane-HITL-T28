"""Sensor noise of the real T-28 flights, as a model the HITL plugin can reproduce at its ~120 frames a second:
gyro and accelerometer noise in the 5-60 Hz band per axis against throttle, barometer noise and slow drift against
INAV's estimated altitude, compass noise and its pull by the motor current per axis. In flight only (armed, ground
speed above 6 m/s). Writes the model as JSON beside the printout.
    python noise_from_logs.py LOG00001.01.csv ... [--json model.json]"""
import json
import sys

import numpy as np
import pandas as pd

ACC_1G = 2048.0
args = [a for a in sys.argv[1:] if not a.startswith("--")]
out_path = sys.argv[sys.argv.index("--json") + 1] if "--json" in sys.argv else None
if out_path in args:
    args.remove(out_path)

COL = ["time (us)", "gyroADC[0] (deg/s)", "gyroADC[1] (deg/s)", "gyroADC[2] (deg/s)", "accSmooth[0]", "accSmooth[1]",
       "accSmooth[2]", "accVib", "BaroAlt (cm)", "navPos[2]", "navVel[0]", "navVel[1]", "motor[0]", "amperage (A)",
       "magADC[0]", "magADC[1]", "magADC[2]", "flightModeFlags (flags)", "IMUTemperature", "attitude[0]", "attitude[1]",
       "attitude[2]"]


def band(x, fs, f1, f2):
    """x band-passed to [f1, f2] Hz with an FFT mask (whole segments, so no filter start-up)."""
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1 / fs)
    X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))


def low_pass(x, fs, fc):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / fs)
    X[f > fc] = 0
    return np.fft.irfft(X, len(x))


rows = {k: [] for k in ("throttle", "g0", "g1", "g2", "a0", "a1", "a2", "vib", "amp", "m0", "m1", "m2", "mn0", "mn1",
                        "mn2", "baro_noise", "baro_drift", "temp", "roll", "pitch", "yaw")}
for path in args:
    d = pd.read_csv(path, usecols=lambda c: c.strip() in COL)
    d.columns = [c.strip() for c in d.columns]
    t = (d["time (us)"] - d["time (us)"].iloc[0]).to_numpy() / 1e6
    fs = 1 / np.median(np.diff(t))
    gs = np.hypot(d["navVel[0]"], d["navVel[1]"]).to_numpy() / 100
    flying = (gs > 6) & d["flightModeFlags (flags)"].astype(str).str.contains("ARM").to_numpy()
    # whole seconds of flight, each one a sample of the noise at its throttle
    idx = np.flatnonzero(flying)
    if len(idx) < fs * 10:
        continue
    a, b = idx[0], idx[-1]
    seg = slice(a, b)
    n = int(fs)
    gyro = [band(d[f"gyroADC[{i}] (deg/s)"].to_numpy()[seg], fs, 5, 60) for i in range(3)]
    acc = [band(d[f"accSmooth[{i}]"].to_numpy()[seg] / ACC_1G * 9.80665, fs, 5, 60) for i in range(3)]
    mag = [d[f"magADC[{i}]"].to_numpy()[seg].astype(float) for i in range(3)]
    magn = [band(m, fs, 2, 60) for m in mag]
    baro = d["BaroAlt (cm)"].to_numpy()[seg] / 100.0
    estimate = d["navPos[2]"].to_numpy()[seg] / 100.0
    baro_n = baro - low_pass(baro, fs, 1.0)
    drift = low_pass(baro - estimate, fs, 0.05)
    throttle = d["motor[0]"].to_numpy()[seg]
    amp = d["amperage (A)"].to_numpy()[seg]
    vib = d["accVib"].to_numpy()[seg]
    temp = d["IMUTemperature"].to_numpy()[seg] / 10.0 if "IMUTemperature" in d else np.zeros(len(throttle))
    attitude = [np.radians(d[f"attitude[{i}]"].to_numpy()[seg] / 10.0) for i in range(3)]
    ok = flying[seg]
    for k in range(0, len(throttle) - n, n):
        if not ok[k:k + n].all():
            continue
        s = slice(k, k + n)
        rows["throttle"].append(throttle[s].mean())
        for i in range(3):
            rows[f"g{i}"].append(gyro[i][s].std())
            rows[f"a{i}"].append(acc[i][s].std())
            rows[f"m{i}"].append(mag[i][s].mean())
            rows[f"mn{i}"].append(magn[i][s].std())
        rows["vib"].append(vib[s].mean())
        rows["amp"].append(amp[s].mean())
        rows["baro_noise"].append(baro_n[s].std())
        rows["baro_drift"].append(drift[s].mean())
        rows["temp"].append(temp[s].mean())
        for name, x in zip(("roll", "pitch", "yaw"), attitude):
            rows[name].append(np.angle(np.exp(1j * x[s]).mean()))
    print(f"{path}: {fs:.0f} Hz, {ok.sum() / fs:.0f} s in flight")

r = {k: np.array(v) for k, v in rows.items()}
throttle = r["throttle"]
model = {"seconds": int(len(throttle)), "band_hz": [5, 60]}
print(f"\n{len(throttle)} seconds of flight; throttle {throttle.min():.0f}-{throttle.max():.0f}")
for name, key, unit in (("gyro", "g", "deg/s"), ("accelerometer", "a", "m/s2")):
    model[name] = {}
    for i, axis in enumerate("xyz"):
        y = r[f"{key}{i}"]
        k, q = np.polyfit(throttle, y, 1)
        model[name][axis] = {"at_1000": float(q + k * 1000), "per_100_throttle": float(k * 100)}
        print(f"{name} {axis}: 5-60 Hz noise median {np.median(y):.3f} {unit}, at throttle 1000 {q + k * 1000:.3f}, "
              f"+{k * 100:.3f} per 100 of throttle (correlation {np.corrcoef(throttle, y)[0, 1]:+.2f})")
model["barometer"] = {"noise_m": float(np.median(r["baro_noise"])),
                      "drift_m": [float(np.percentile(r["baro_drift"], 5)), float(np.percentile(r["baro_drift"], 95))]}
print(f"barometer: noise above 1 Hz median {np.median(r['baro_noise']):.3f} m; drift against the estimated altitude "
      f"5-95 % {np.percentile(r['baro_drift'], 5):+.1f}/{np.percentile(r['baro_drift'], 95):+.1f} m")
k, q = np.polyfit(r["temp"], r["baro_drift"], 1) if r["temp"].std() > 0.1 else (0.0, 0.0)
print(f"   drift against the IMU temperature: {k:+.2f} m/C (correlation {np.corrcoef(r['temp'], r['baro_drift'])[0, 1]:+.2f})")
model["compass"] = {"noise": {}, "per_ampere": {}}
amp = r["amp"]


def from_earth(roll, pitch, yaw, sr, sp, sy):
    """Rows of the earth-to-body rotation for each second, with the attitude signs to try."""
    f, th, ps = sr * roll, sp * pitch, sy * yaw
    cf, sf, ct, st, cp, sp_ = np.cos(f), np.sin(f), np.cos(th), np.sin(th), np.cos(ps), np.sin(ps)
    return np.stack([np.stack([ct * cp, ct * sp_, -st], -1),
                     np.stack([sf * st * cp - cf * sp_, sf * st * sp_ + cf * cp, sf * ct], -1),
                     np.stack([cf * st * cp + sf * sp_, cf * st * sp_ - sf * cp, cf * ct], -1)], 1)


# body field = R(attitude) * earth field + offset + per_ampere * current: least squares over every second,
# trying the attitude sign conventions and the axis signs, keeping the best fit
meas = np.stack([r["m0"], r["m1"], r["m2"]], -1)
best = None
for sr in (1, -1):
    for sp in (1, -1):
        for sy in (1, -1):
            R = from_earth(r["roll"], r["pitch"], r["yaw"], sr, sp, sy)
            A = np.zeros((3 * len(amp), 9))
            for i in range(3):
                A[i::3, 0:3] = R[:, i, :]
                A[i::3, 3 + i] = 1.0
                A[i::3, 6 + i] = amp
            x, res, *_ = np.linalg.lstsq(A, meas.reshape(-1), rcond=None)
            rms = np.sqrt(np.mean((A @ x - meas.reshape(-1)) ** 2))
            if best is None or rms < best[0]:
                best = (rms, x, (sr, sp, sy))
rms, x, signs = best
print(f"compass: estimated earth field |B| {np.linalg.norm(x[0:3]):.0f}, model error {rms:.0f} (signs {signs})")
for i, axis in enumerate("xyz"):
    model["compass"]["noise"][axis] = float(np.median(r[f"mn{i}"]))
    model["compass"]["per_ampere"][axis] = float(x[6 + i])
    print(f"compass {axis}: 2-60 Hz noise median {np.median(r[f'mn{i}']):.1f}; offset {x[3 + i]:+.0f}; "
          f"shift with the current {x[6 + i]:+.2f} per A")
model["current_from_throttle"] = [float(c) for c in np.polyfit(throttle, amp, 2)]
print(f"current: {amp.min():.1f}-{amp.max():.1f} A; accVib median {np.median(r['vib']):.0f}")
if out_path:
    json.dump(model, open(out_path, "w"), indent=1)
    print("model written to", out_path)
