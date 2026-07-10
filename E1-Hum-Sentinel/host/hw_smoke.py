#!/usr/bin/env python3
"""Hardware acceptance test for the E1 acoustic sentinel (~1 min).

Drives the flashed firmware over the real serial link with
deterministic synthetic room audio and checks the full protocol +
detector behavior:

  1. GET_STATUS answers
  2. fast relearn (16 chunks) completes
  3. a quiet room stays quiet (no events, low scores, SPECTRUM flowing)
  4. an injected jingle fires a HIGH event; silence clears it
  5. DEPLOY mode: events still fire, zero SPECTRUM leaves
  6. reports E1 compute time (chunk_us) and link timing

Usage:  python3 host/hw_smoke.py --port /dev/ttyACM2
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

import e1proto
from audio_source import SynthSource, ulaw_encode

LEARN_CHUNKS = 16
FAILURES = []


def check(cond: bool, what: str) -> None:
    tag = "ok  " if cond else "FAIL"
    print(f"  {tag} {what}")
    if not cond:
        FAILURES.append(what)


class Link:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.05, rtscts=True)
        self.parser = e1proto.Parser()
        self.source = SynthSource(seed=42)
        self.seq = 0
        self.spectra = 0
        self.chunk_us: list[int] = []
        self.wire_s: list[float] = []

    def collect(self, types: set[int], timeout: float = 6.0) -> dict[int, e1proto.Msg]:
        """Read until one of each requested type arrives (or timeout).
        Returns {type: last Msg of that type}."""
        got: dict[int, e1proto.Msg] = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not types.issubset(got):
            for m in self.parser.feed(self.ser.read(8192)):
                if not m.crc_ok:
                    continue
                if m.type == e1proto.SPECTRUM:
                    self.spectra += 1
                if m.type == e1proto.STATUS and m.chunk_us:
                    self.chunk_us.append(m.chunk_us)
                got[m.type] = m
        return got

    def send_chunk(self, expect_spectrum: bool) -> e1proto.Msg | None:
        """Send one chunk; wait for the reply set; returns the STATUS."""
        pcm = self.source.get_chunk(e1proto.AUDIO_BYTES)
        self.seq = (self.seq + 1) & 0xFF
        t0 = time.monotonic()
        self.ser.write(e1proto.build_audio(self.seq, ulaw_encode(pcm).tobytes()))
        want = {e1proto.ACK, e1proto.STATUS}
        if expect_spectrum:
            want.add(e1proto.SPECTRUM)
        got = self.collect(want)
        self.wire_s.append(time.monotonic() - t0)
        ack = got.get(e1proto.ACK)
        if ack is None or not ack.ok or ack.seq != self.seq:
            return None
        return got.get(e1proto.STATUS)

    def set_param(self, pid: int, value: int) -> bool:
        self.ser.write(e1proto.build_set_param(pid, value))
        ack = self.collect({e1proto.ACK}).get(e1proto.ACK)
        return bool(ack and ack.ok and ack.seq == pid)

    def stream(self, n: int, expect_spectrum: bool) -> list[e1proto.Msg]:
        out = []
        for _ in range(n):
            st = self.send_chunk(expect_spectrum)
            if st is None:
                raise RuntimeError("chunk NAKed or reply timed out")
            out.append(st)
        return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True, help="EVK chunk link (e.g. /dev/ttyACM2)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    t_start = time.monotonic()
    lk = Link(args.port, args.baud)
    lk.ser.reset_input_buffer()

    print("1. handshake")
    lk.ser.write(e1proto.build_get_status())
    st = lk.collect({e1proto.STATUS}).get(e1proto.STATUS)
    check(st is not None, "GET_STATUS answered")
    if st is None:
        sys.exit("no response — is hum_sentinel flashed and BOOT set for SRAM?")

    print("2. learning")
    check(lk.set_param(e1proto.PARAM_LEARN_CHUNKS, LEARN_CHUNKS), "set LEARN_CHUNKS")
    check(lk.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEV), "set MODE=DEV")
    check(lk.set_param(e1proto.PARAM_RELEARN, 0), "RELEARN acked")
    during = lk.stream(LEARN_CHUNKS, expect_spectrum=True)
    check(during[0].learning, "learning flag set")
    check(not during[-1].learning and during[-1].learn_pct == 100,
          "learning completed")
    events0 = during[-1].events  # cumulative since boot; firmware may be warm

    print("3. quiet room")
    statuses = lk.stream(8, expect_spectrum=True)
    scores = [s.score for s in statuses]
    check(max(scores) < 60, f"scores stay low (max={max(scores)})")
    check(not any(s.event for s in statuses), "no events")
    check(lk.spectra >= LEARN_CHUNKS + 8, "SPECTRUM per chunk in DEV")

    print("4. anomaly: jingle")
    lk.source.inject("jingle")
    statuses = lk.stream(5, expect_spectrum=True)
    fired = [s for s in statuses if s.event]
    check(len(fired) > 0, "event fired")
    check(bool(fired) and fired[-1].ev_class == e1proto.CLASS_HIGH,
          f"classed HIGH (got {fired[-1].ev_class if fired else '-'})")
    statuses = lk.stream(10, expect_spectrum=True)
    check(not statuses[-1].event, "event cleared in silence")
    check(statuses[-1].events == events0 + 1, "event counted once")

    print("5. DEPLOY")
    check(lk.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEPLOY), "MODE=DEPLOY acked")
    lk.spectra = 0
    lk.source.inject("voice")
    statuses = lk.stream(6, expect_spectrum=False)
    check(lk.spectra == 0, "zero SPECTRUM in DEPLOY")
    check(any(s.event for s in statuses), "event still fired")
    check(lk.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEV), "back to DEV")

    wall = time.monotonic() - t_start
    n = len(lk.chunk_us)
    avg_us = sum(lk.chunk_us) / max(1, n)
    avg_wire = sum(lk.wire_s) / max(1, len(lk.wire_s))
    print(f"\ntiming: E1 compute {avg_us / 1000:.2f} ms/chunk (n={n}), "
          f"link round-trip {avg_wire * 1000:.0f} ms/chunk "
          f"({min(1.0, 0.128 / avg_wire) * 100:.0f}% of realtime), "
          f"crc errs {lk.parser.crc_errors}, total {wall:.0f}s")

    if FAILURES:
        print(f"hw_smoke: {len(FAILURES)} check(s) FAILED")
        sys.exit(1)
    print("hw_smoke: PASS")


if __name__ == "__main__":
    main()
