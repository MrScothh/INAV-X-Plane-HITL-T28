"""Reads X-Plane datarefs over UDP RREF from its own port and prints them for a few seconds.
    python read_dataref.py <seconds> dataref [dataref ...]"""
import socket
import struct
import sys
import time

seconds, names = float(sys.argv[1]), sys.argv[2:]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 0))
s.settimeout(0.5)
for i, n in enumerate(names):
    s.sendto(b"RREF\x00" + struct.pack("<ii400s", 10, 900 + i, n.encode()), ("127.0.0.1", 49000))
val = {}
end = time.monotonic() + seconds
next_print = 0
while time.monotonic() < end:
    try:
        d, _ = s.recvfrom(4096)
    except (socket.timeout, ConnectionResetError):
        continue
    for k in range(5, len(d) - 7, 8):
        i, v = struct.unpack("<if", d[k:k + 8])
        val[i - 900] = v
    if time.monotonic() >= next_print:
        next_print = time.monotonic() + 0.5
        print("  ".join("%s=%+.2f" % (n.split("/")[-1], val.get(i, float("nan"))) for i, n in enumerate(names)), flush=True)
for i, n in enumerate(names):
    s.sendto(b"RREF\x00" + struct.pack("<ii400s", 0, 900 + i, n.encode()), ("127.0.0.1", 49000))
