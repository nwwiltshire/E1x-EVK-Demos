#!/usr/bin/env python3
"""Frame-link echo test — run against firmware built from echo_main.c.

Sends frame-sized random chunks, verifies the byte-exact echo, and
reports effective round-trip throughput plus the people-counter frame
rate that implies.  Step 1 of hardware bring-up (see README): prove
the port, baud, and byte integrity before flashing the real firmware.

  python3 host/echo_test.py --sim 127.0.0.1:5555          # simulator
  python3 host/echo_test.py --port /dev/ttyACM1 --baud 115200
"""

from __future__ import annotations

import argparse
import random
import socket
import sys
import time

FRAME_BYTES = 30000  # one 200x150 frame
CHUNKS = 4


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", metavar="HOST:PORT")
    g.add_argument("--port", metavar="DEV")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--chunks", type=int, default=CHUNKS)
    args = ap.parse_args()

    if args.sim:
        host, _, port = args.sim.partition(":")
        sock = socket.create_connection((host, int(port or 5555)), timeout=10)
        sock.settimeout(10)
        read, write = (lambda n: sock.recv(n)), sock.sendall
    else:
        import serial

        ser = serial.Serial(args.port, args.baud, timeout=10)
        read, write = (lambda n: ser.read(min(n, 4096))), ser.write

    rng = random.Random(42)
    total = t_total = 0.0
    for i in range(args.chunks):
        payload = bytes(rng.randrange(256) for _ in range(FRAME_BYTES))
        t0 = time.monotonic()
        write(payload)
        got = bytearray()
        while len(got) < len(payload):
            data = read(len(payload) - len(got))
            if not data:
                sys.exit(f"chunk {i}: timeout after {len(got)}/{len(payload)} echoed bytes")
            got += data
        dt = time.monotonic() - t0
        if bytes(got) != payload:
            bad = next(j for j in range(len(payload)) if got[j] != payload[j])
            sys.exit(f"chunk {i}: corruption at byte {bad} "
                     f"(sent {payload[bad]:#04x}, got {got[bad]:#04x})")
        total += len(payload)
        t_total += dt
        print(f"chunk {i}: {len(payload)} bytes round-trip in {dt:.2f}s ok")

    kbs = total / t_total / 1000
    print(f"\necho ok: {args.chunks}x{FRAME_BYTES} bytes, {kbs:.1f} KB/s round-trip")
    print(f"  -> ~{kbs * 1000 / FRAME_BYTES:.2f} fps DEV (frame down + mask up)")
    print(f"  -> ~{kbs * 1000 / FRAME_BYTES * 2:.2f} fps DEPLOY (frame down only)")


if __name__ == "__main__":
    main()
