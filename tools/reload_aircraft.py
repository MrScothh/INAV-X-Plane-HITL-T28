"""Reloads the current aircraft from disk in X-Plane 11 (an ACFN with the same path keeps the old one), and prints
the centre of gravity and elevator travel X-Plane holds before and after, to show the new ACF is in.
    python reload_aircraft.py"""
import socket
import struct
import time

XP = ("127.0.0.1", 49000)
NAMES = ["sim/aircraft/weight/acf_cgZ_original", "sim/aircraft/controls/acf_elev_up", "sim/aircraft/controls/acf_ailn1_up",
         "sim/aircraft/controls/acf_elev_dn"]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 0))
s.settimeout(0.5)


def read():
    for i, n in enumerate(NAMES):
        s.sendto(b"RREF\x00" + struct.pack("<ii400s", 5, 900 + i, n.encode()), XP)
    val = {}
    end = time.monotonic() + 10
    while len(val) < len(NAMES) and time.monotonic() < end:
        try:
            d, _ = s.recvfrom(4096)
            for k in range(5, len(d) - 7, 8):
                i, v = struct.unpack("<if", d[k:k + 8])
                if 900 <= i < 900 + len(NAMES):
                    val[i - 900] = v
        except (socket.timeout, ConnectionResetError):
            pass
    for i, n in enumerate(NAMES):
        s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 900 + i, n.encode()), XP)
    return {NAMES[k].rsplit("/", 1)[1]: round(v, 3) for k, v in sorted(val.items())}


print("before:", read())
s.sendto(b"CMND\x00sim/operation/reload_aircraft", XP)
time.sleep(15)
print("after: ", read())
