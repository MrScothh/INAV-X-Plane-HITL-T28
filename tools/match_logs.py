"""The X-Plane T-28 against the real aircraft's logs (log_targets.py), open loop, no flight controller.
At each airspeed, placed 150 m above wherever the aircraft is, a simple autopilot flies:
- level flight: throttle, pitch stick (INAV units, 1/500 of the stick as in HITL), body angle of attack;
- full throttle and idle with the speed held by pitch: specific excess power Ps = climb rate + V dV/dt / g;
- 0.8 s stick steps around trim: roll and pitch rate per INAV unit, as real_manual.py measures them;
- level flight with half and full flaps: what changes in throttle, stick and angle of attack.
    python match_logs.py [label] [speed m/s ...]       -> match_<label>.txt next to this script
Needs X-Plane running with the T-28 loaded and the HITL plugin disconnected."""
import os
import socket
import struct
import sys
import time

XP = ("127.0.0.1", 49000)
HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = sys.argv[1] if len(sys.argv) > 1 else "t28"
SPEEDS = [float(v) for v in sys.argv[2:]] or [14.5, 17.5, 20.5, 23.5, 26.5]
FLAP_SPEEDS = [v for v in SPEEDS if v <= 21]
G = 9.81
STEP = 0.03
STEP_S = float(os.environ.get("STEP_SECONDS", "0.8"))   # 2 for a longer step; the rate is the mean over its second half
SKIP = set(os.environ.get("SKIP", "").split(","))       # any of: ps, steps, flaps

# Real T-28 from its logs (log_targets.py, real_manual.py, real_trim.py), by the same airspeed bands
LOG = {
    # rates: rate = k (command - trim) per airspeed band in MANUAL, at these speeds by interpolation.
    # ps_full: airborne full-throttle windows at 0.85-1.15 g only (the others are the take-off run and the pull-up
    # after rotation, at 1.4-1.7 g); none under 16 m/s
    14.5: dict(throttle=0.31, ps_full=None, ps_idle=-2.3, roll=0.27, pitch=None),
    17.5: dict(throttle=0.42, ps_full=17.0, ps_idle=-3.5, roll=0.31, pitch=0.17),
    20.5: dict(throttle=0.46, ps_full=15.6, ps_idle=-5.5, roll=0.53, pitch=0.26),
    23.5: dict(throttle=0.57, ps_full=17.0, ps_idle=-7.3, roll=0.66, pitch=0.38),
    26.5: dict(throttle=0.64, ps_full=15.9, ps_idle=None, roll=0.69, pitch=0.49),
}
LOG_FLAP = {0.5: dict(dalpha=-5.9, dstick=-50), 1.0: dict(dalpha=-7.2, dstick=-75)}

NAMES = ["sim/flightmodel/position/phi", "sim/flightmodel/position/theta", "sim/flightmodel/position/P",
         "sim/flightmodel/position/Q", "sim/flightmodel/position/true_airspeed", "sim/flightmodel/position/y_agl",
         "sim/flightmodel/position/vh_ind", "sim/flightmodel/position/alpha", "sim/time/framerate_period",
         "sim/cockpit2/controls/flap_handle_deploy_ratio", "sim/flightmodel/engine/ENGN_tacrad[0]"]
PHI, THETA, P, Q, TAS, AGL, VS, ALPHA, FRAME, FLAP, RPM = range(len(NAMES))
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 0))
s.settimeout(0.02)
val = {}
lines = []


def write(r):
    print(r, flush=True)
    lines.append(r)


def dref(n, v):
    s.sendto(b"DREF\x00" + struct.pack("<f500s", float(v), n.encode()), XP)


def drain():
    """Drains what X-Plane has sent and returns: waiting for a gap in the stream hung for seconds at 100 Hz."""
    s.setblocking(False)
    try:
        for _ in range(200):
            d, _ = s.recvfrom(4096)
            for k in range(5, len(d) - 7, 8):
                i, v = struct.unpack("<if", d[k:k + 8])
                if 700 <= i < 700 + len(NAMES):
                    val[i - 700] = v
    except (BlockingIOError, ConnectionResetError):
        pass
    finally:
        s.settimeout(0.02)


def read_position():
    pos_names = ["sim/flightmodel/position/latitude", "sim/flightmodel/position/longitude",
                 "sim/flightmodel/position/elevation", "sim/flightmodel/position/y_agl", "sim/flightmodel/position/psi"]
    for i, n in enumerate(pos_names):
        s.sendto(b"RREF\x00" + struct.pack("<ii400s", 5, 800 + i, n.encode()), XP)
    pos = {}
    end = time.monotonic() + 60
    while len(pos) < 5 and time.monotonic() < end:
        try:
            d, _ = s.recvfrom(4096)
            for k in range(5, len(d) - 7, 8):
                i, v = struct.unpack("<if", d[k:k + 8])
                if 800 <= i < 805:
                    pos[i - 800] = v
        except (socket.timeout, ConnectionResetError):
            pass
    for i, n in enumerate(pos_names):
        s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 800 + i, n.encode()), XP)
    if len(pos) < 5:
        raise SystemExit("position not read: is X-Plane answering?")
    return pos


class Pilot:
    """Pitch on climb rate (level) or on airspeed (fixed throttle), roll on bank, throttle on airspeed."""

    def __init__(self, v, throttle=0.45):
        self.v = v
        self.i_p, self.i_r, self.i_t, self.i_v = 0.05, 0.0, throttle, 0.0
        self.yp = self.yr = 0.0
        self.throttle = throttle
        self.hold_speed = False
        self.theta_max = 12.0
        self.kp, self.ki, self.kq = 0.004, 0.004, 0.0008   # pitch loop: per degree, per degree-second, per deg/s

    def step(self, dt, pitch=True, roll=True, throttle=True):
        # X-Plane's first samples after a placement hold garbage rates: one of them gave full aileron
        vs = max(-30.0, min(30.0, val[VS]))
        p_, q_ = max(-300.0, min(300.0, val[P])), max(-300.0, min(300.0, val[Q]))
        if self.hold_speed:
            e_v = val[TAS] - self.v
            self.i_v = max(-30.0, min(30.0, self.i_v + 0.6 * e_v * dt))
            theta_ref = max(-35.0, min(45.0, val[ALPHA] + self.i_v + 2.0 * e_v))
        else:
            theta_ref = max(-8.0, min(self.theta_max, val[ALPHA] - 1.5 * vs))
        e_t = theta_ref - val[THETA]
        if pitch:
            self.i_p = max(-1.0, min(1.0, self.i_p + self.ki * e_t * dt))
            self.yp = max(-1.0, min(1.0, self.i_p + self.kp * e_t - self.kq * q_))
        if roll:
            e_r = -val[PHI]
            self.i_r = max(-0.5, min(0.5, self.i_r + 0.002 * e_r * dt))
            self.yr = max(-1.0, min(1.0, self.i_r + 0.003 * e_r - 0.0005 * p_))
        if throttle and not self.hold_speed:
            e_v = self.v - val[TAS]
            self.i_t = max(0.0, min(1.0, self.i_t + 0.03 * e_v * dt))
            self.throttle = max(0.0, min(1.0, self.i_t + 0.08 * e_v))
        dref("sim/joystick/yoke_pitch_ratio", self.yp)
        dref("sim/joystick/yoke_roll_ratio", self.yr)
        dref("sim/cockpit2/engine/actuators/throttle_ratio_all", self.throttle)


def fly(pilot, seconds, **kw):
    samples = []
    t0 = time.monotonic()
    t_prev = t0
    while time.monotonic() - t0 < seconds:
        drain()
        if len(val) < len(NAMES):
            continue
        now = time.monotonic()
        pilot.step(now - t_prev, **kw)
        t_prev = now
        samples.append((now - t0, dict(val), pilot.yp, pilot.yr, pilot.throttle))
        time.sleep(0.005)
    return samples


def mean(c, idx):
    return sum(x[1][idx] for x in c) / len(c)


def slope(c, idx):
    t = [x[0] for x in c]
    y = [x[1][idx] for x in c]
    mt, my = sum(t) / len(t), sum(y) / len(y)
    den = sum((a - mt) ** 2 for a in t) or 1.0
    return sum((a - mt) * (b - my) for a, b in zip(t, y)) / den


def stick_step(pilot, axis, delta):
    base = pilot.yp if axis == "pitch" else pilot.yr
    if axis == "pitch":
        pilot.yp = max(-1.0, min(1.0, base + delta))
        c = fly(pilot, STEP_S, pitch=False, throttle=False)
    else:
        pilot.yr = max(-1.0, min(1.0, base + delta))
        c = fly(pilot, STEP_S, roll=False, throttle=False)
    tail = [x for x in c if x[0] >= STEP_S / 2]
    idx = Q if axis == "pitch" else P
    rate = sum(x[1][idx] for x in tail) / max(1, len(tail))
    if axis == "pitch":
        pilot.yp = base
    else:
        pilot.yr = base
    fly(pilot, 5.0)
    return rate / delta / 500.0


def place(pos, ground, v, flap=0.0):
    s.sendto(b"PREL\x00" + struct.pack("<ii8sii5d", 6, 0, b"", 0, 0, pos[0], pos[1], ground + 150.0, pos[4], v), XP)
    val.clear()
    end = time.monotonic() + 120
    while time.monotonic() < end:
        time.sleep(0.5)
        drain()
        if len(val) == len(NAMES) and val[AGL] > 50:
            break
    dref("sim/cockpit2/controls/gear_handle_down", 0.0)
    dref("sim/cockpit2/controls/flap_ratio", flap)
    # hands off for a moment: the placement takes X-Plane about a second, and its first samples are not to be trusted
    for a in ("pitch", "roll", "heading"):
        dref("sim/joystick/yoke_%s_ratio" % a, 0.0)
    time.sleep(0.6)
    drain()


def level(v, flap, pos, ground):
    place(pos, ground, v, flap)
    pilot = Pilot(v)
    fly(pilot, 14.0)
    c = fly(pilot, 4.0)
    return pilot, dict(throttle=sum(x[4] for x in c) / len(c), stick=sum(x[2] for x in c) / len(c) * 500,
                       alpha=mean(c, ALPHA), theta=mean(c, THETA), v=mean(c, TAS), vs=mean(c, VS),
                       rpm=mean(c, RPM) * 60 / (2 * 3.14159265))


def ps_at_throttle(v, throttle, pos, ground):
    place(pos, ground, v)
    pilot = Pilot(v, throttle)
    fly(pilot, 4.0)                      # settle at level-flight throttle first
    pilot.hold_speed = True
    pilot.throttle = throttle
    fly(pilot, 8.0)
    c = fly(pilot, 4.0)
    return mean(c, VS) + mean(c, TAS) * slope(c, TAS) / G, mean(c, TAS)


def fmt(x, f="%+.2f"):
    return "  -  " if x is None else f % x


def main():
    pos = read_position()
    ground = pos[2] - pos[3]
    for i, n in enumerate(NAMES):
        s.sendto(b"RREF\x00" + struct.pack("<ii400s", 30, 700 + i, n.encode()), XP)
    dref("sim/operation/override/override_joystick", 1)
    for a in ("pitch", "roll", "heading"):
        dref("sim/joystick/joystick_%s_augment" % a, 0.0)
    write("== %s, %s" % (LABEL, time.strftime("%Y-%m-%d %H:%M")))
    write("v m/s | level throttle: model / log | stick | alpha | Ps full throttle: model / log | Ps idle: "
          "model / log | roll deg/s per unit: model / log | pitch: model / log | rpm")
    base = {}
    try:
        for v in SPEEDS:
            pilot, lv = level(v, 0.0, pos, ground)
            base[v] = lv
            if "steps" in SKIP:
                rp = rm = pp = pm = None
            else:
                pp = stick_step(pilot, "pitch", STEP)
                pm = stick_step(pilot, "pitch", -STEP)
                rp = stick_step(pilot, "roll", STEP)
                rm = stick_step(pilot, "roll", -STEP)
            if "ps" in SKIP:
                ps1 = ps0 = None
            else:
                ps1, _ = ps_at_throttle(v, 1.0, pos, ground)
                ps0, _ = ps_at_throttle(v, 0.0, pos, ground)
            o = LOG.get(v, {})
            write("%4.1f (%.1f) | %.2f / %s | %+4.0f | %+5.1f | %s / %s | %s / %s | %s / %s | %s / %s | %.0f" % (
                v, lv["v"], lv["throttle"], fmt(o.get("throttle"), "%.2f"), lv["stick"], lv["alpha"],
                fmt(ps1, "%.1f"), fmt(o.get("ps_full"), "%.1f"), fmt(ps0, "%.1f"), fmt(o.get("ps_idle"), "%.1f"),
                fmt(None if rp is None else (rp + rm) / 2, "%.2f"), fmt(o.get("roll"), "%.2f"),
                fmt(None if pp is None else (pp + pm) / 2, "%.2f"), fmt(o.get("pitch"), "%.2f"), lv["rpm"]))
        if "flaps" not in SKIP:
            write("flap | v m/s | throttle | d stick: model / log | d alpha: model / log")
            for fl in (0.5, 1.0):
                for v in FLAP_SPEEDS:
                    _, lv = level(v, fl, pos, ground)
                    b = base[v]
                    write("%.1f | %4.1f | %.2f | %+4.0f / %+4.0f | %+5.1f / %+5.1f" % (
                        fl, v, lv["throttle"], lv["stick"] - b["stick"], LOG_FLAP[fl]["dstick"],
                        lv["alpha"] - b["alpha"], LOG_FLAP[fl]["dalpha"]))
    finally:
        dref("sim/cockpit2/controls/flap_ratio", 0.0)
        dref("sim/joystick/yoke_roll_ratio", 0.0)
        dref("sim/joystick/yoke_pitch_ratio", 0.0)
        dref("sim/operation/override/override_joystick", 0)
        for i, n in enumerate(NAMES):
            s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 700 + i, n.encode()), XP)
        with open(os.path.join(HERE, "match_%s.txt" % LABEL), "a", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n\n")


if __name__ == "__main__":
    main()
