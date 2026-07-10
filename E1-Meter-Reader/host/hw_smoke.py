#!/usr/bin/env python3
"""Hardware acceptance test for the E1 meter reader — ~1 minute over
the real serial link.  Run after every flash:

    python3 host/hw_smoke.py --port /dev/ttyACM2

Synthetic gauge frames at known angles go down the wire; the readings
come back.  Assertions are robust to a warm device (the frame counter
persists until reflash/power-cycle), and every phase prints ok/FAIL.
Exit code = number of failures.
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

import e1proto
import gauge_sim

FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILURES.append(what)


def err_deg(got: float, want: float) -> float:
    return abs((got - want + 180.0) % 360.0 - 180.0)


class Link:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.05, rtscts=True)
        self.parser = e1proto.Parser()

    def collect(self, want_types: set[int], timeout: float = 5.0) -> dict:
        got: dict = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not want_types <= set(got):
            data = self.ser.read(65536)
            if not data:
                continue
            for m in self.parser.feed(data):
                if m.crc_ok:
                    got[m.type] = m
        return got

    def set_param(self, pid: int, value: int) -> bool:
        self.ser.write(e1proto.build_set_param(pid, value))
        ack = self.collect({e1proto.ACK}, timeout=2.0).get(e1proto.ACK)
        return bool(ack and ack.ok and ack.seq == pid)

    def send_frame(self, seq: int, pixels: bytes, expect_scores: bool,
                   timeout: float = 8.0) -> tuple:
        t0 = time.monotonic()
        self.ser.write(e1proto.build_frame(seq, pixels))
        want = {e1proto.STATUS, e1proto.ACK}
        if expect_scores:
            want.add(e1proto.SCORES)
        got = self.collect(want, timeout=timeout)
        rtt = time.monotonic() - t0
        return (got.get(e1proto.STATUS), got.get(e1proto.SCORES),
                got.get(e1proto.ACK), rtt)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM2")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    link = Link(args.port, args.baud)

    print("--- 1: handshake")
    link.ser.write(e1proto.build_get_status())
    st0 = link.collect({e1proto.STATUS}, timeout=3.0).get(e1proto.STATUS)
    check(st0 is not None, "firmware answers GET_STATUS")
    if st0 is None:
        sys.exit(len(FAILURES))
    frames0 = st0.frames
    print(f"     warm device: frames={frames0}, mode={st0.mode}")

    print("--- 2: params")
    check(link.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEV), "DEV mode set")
    check(link.set_param(e1proto.PARAM_SMOOTH_SHIFT, 0), "smoothing off")
    check(link.set_param(e1proto.PARAM_POLARITY, 0), "polarity dark-needle")
    check(link.set_param(e1proto.PARAM_RESET, 0), "EMA reset")
    link.ser.write(e1proto.build_set_param(99, 0))
    ack = link.collect({e1proto.ACK}, timeout=2.0).get(e1proto.ACK)
    check(bool(ack) and not ack.ok, "unknown param NAKed")

    print("--- 3: readings at known angles")
    rtts, us = [], []
    seq = 0
    for want in (-120.0, -45.0, 0.0, 60.0, 135.0):
        seq += 1
        st, sc, ack, rtt = link.send_frame(
            seq, gauge_sim.render_frame64(want), expect_scores=True)
        ok = st is not None and sc is not None and ack is not None and ack.ok
        check(ok, f"angle {want:+7.1f}: full SCORES+STATUS+ACK reply")
        if not ok:
            continue
        rtts.append(rtt)
        us.append(st.frame_us)
        e = err_deg(st.angle_deg, want)
        check(e < 2.5, f"angle {want:+7.1f} -> {st.angle_deg:+7.2f} (err {e:.2f})")
        check(st.needle, f"angle {want:+7.1f} needle flag")

    print("--- 4: frame counter advances (warm-safe)")
    link.ser.write(e1proto.build_get_status())
    st = link.collect({e1proto.STATUS}, timeout=3.0).get(e1proto.STATUS)
    check(st is not None and (st.frames - frames0) % 65536 == seq,
          f"frames advanced by {seq}")

    print("--- 5: corrupt frame -> NAK -> recovery")
    wire = bytearray(e1proto.build_frame(77, gauge_sim.render_frame64(10.0)))
    wire[500] ^= 0xFF
    link.ser.write(bytes(wire))
    ack = link.collect({e1proto.ACK}, timeout=8.0).get(e1proto.ACK)
    check(bool(ack) and not ack.ok, "corrupt frame NAKed")
    st, sc, ack, _ = link.send_frame(78, gauge_sim.render_frame64(10.0), True)
    check(bool(ack and ack.ok), "clean frame after corrupt ACKed")

    print("--- 6: DEPLOY mode")
    check(link.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEPLOY), "enter DEPLOY")
    st, sc, ack, rtt_dep = link.send_frame(
        79, gauge_sim.render_frame64(-45.0), expect_scores=False)
    check(bool(ack and ack.ok), "frame ACKed in DEPLOY")
    check(st is not None and st.mode == e1proto.MODE_DEPLOY, "STATUS says DEPLOY")
    check(not link.collect({e1proto.SCORES}, timeout=0.5),
          "no SCORES in DEPLOY")
    check(link.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEV), "back to DEV")

    if rtts:
        print(f"\ntiming: E1 compute {sum(us) / len(us) / 1000.0:.2f} ms/frame, "
              f"DEV round trip {sum(rtts) / len(rtts):.2f} s "
              f"(deploy {rtt_dep:.2f} s), "
              f"crc errs {link.parser.crc_errors}")
    print(f"{len(FAILURES)} failures")
    sys.exit(len(FAILURES))


if __name__ == "__main__":
    main()
