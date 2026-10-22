"""Take-off run of the X-Plane T-28 against the real one (logs: 1.2 -> 14 m/s in 1.25 s at full throttle, 72 A):
reset on the runway, full throttle in 0.3 s, controls centred, no INAV; ground speed, heading, pitch every 0.1 s.
    python model_takeoff.py"""
import socket
import struct
import time

XP = ("127.0.0.1", 49000)
N = ["sim/flightmodel/position/psi", "sim/flightmodel/position/groundspeed", "sim/flightmodel/position/theta",
     "sim/flightmodel/position/R", "sim/flightmodel/engine/POINT_tacrad[0]", "sim/flightmodel/engine/POINT_thrust[0]",
     "sim/flightmodel/position/y_agl"]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 0))
s.settimeout(0.01)
val = {}


def dref(n, v):
    s.sendto(b"DREF\x00" + struct.pack("<f500s", float(v), n.encode()), XP)


def drain():
    try:
        while True:
            d, _ = s.recvfrom(4096)
            for k in range(5, len(d) - 7, 8):
                i, v = struct.unpack("<if", d[k:k + 8])
                if 600 <= i < 600 + len(N):
                    val[i - 600] = v
    except (socket.timeout, ConnectionResetError):
        pass


s.sendto(b"CMND\x00sim/operation/reset_flight", XP)
time.sleep(8)
for i, n in enumerate(N):
    s.sendto(b"RREF\x00" + struct.pack("<ii400s", 50, 600 + i, n.encode()), XP)
dref("sim/operation/override/override_joystick", 1)
for a in ("pitch", "roll", "heading"):
    dref("sim/joystick/yoke_%s_ratio" % a, 0.0)
dref("sim/cockpit2/controls/parking_brake_ratio", 0.0)
time.sleep(1)
t0 = time.monotonic()
next_print = 0.0
t14 = None
try:
    while time.monotonic() - t0 < 3.0:
        t = time.monotonic() - t0
        dref("sim/cockpit2/engine/actuators/throttle_ratio_all", min(1.0, t / 0.3))
        drain()
        time.sleep(0.005)
        if len(val) == len(N):
            if t14 is None and val[1] >= 14.0:
                t14 = t
            if t >= next_print:
                print("t %.1f gs %5.1f psi %6.1f theta %+5.1f agl %.2f rpm %5.0f thrust %5.1f N" % (
                    t, val[1], val[0], val[2], val[6], val[4] * 9.549, val[5]))
                next_print += 0.1
    print("14 m/s after %s s from the command (real: 1.25 s from 1.2 m/s)" % ("%.2f" % t14 if t14 else "-"))
finally:
    dref("sim/cockpit2/engine/actuators/throttle_ratio_all", 0.0)
    dref("sim/operation/override/override_joystick", 0)
    for i, n in enumerate(N):
        s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 600 + i, n.encode()), XP)
