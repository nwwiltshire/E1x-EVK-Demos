#!/usr/bin/env python3
"""End-to-end tests: spawn the firmware simulator and drive the real
wire protocol over its TCP "serial port".

Each case gets a fresh firmware process (E1_SIM_PORT=0 = ephemeral
port, scraped from stderr).  Synthetic gauges rendered at known angles
go down the wire; the STATUS reading and SCORES array come back.

Usage: tests/e2e_check.py [--fw firmware/build/firmware_sim]
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "host"))
import e1proto  # noqa: E402
import gauge_sim  # noqa: E402

FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILURES.append(what)


class Firmware:
    """One firmware_sim process on an ephemeral loopback port."""

    def __init__(self, binary: str) -> None:
        self.proc = subprocess.Popen(
            [binary], env={**os.environ, "E1_SIM_PORT": "0"},
            stderr=subprocess.PIPE, text=True,
        )
        self.port = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            line = self.proc.stderr.readline()
            m = re.search(r"listening on 127\.0\.0\.1:(\d+)", line or "")
            if m:
                self.port = int(m.group(1))
                break
        if self.port is None:
            self.proc.kill()
            raise RuntimeError("firmware did not report its port")
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self) -> None:
        for _ in self.proc.stderr:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.proc.kill()
        self.proc.wait()


class Client:
    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.1)
        self.parser = e1proto.Parser()

    def send(self, wire: bytes) -> None:
        self.sock.sendall(wire)

    def collect(self, want_types: set[int], timeout: float = 3.0) -> dict:
        """Read until one of each requested type arrives (or timeout)."""
        got: dict = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not want_types <= set(got):
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                continue
            for m in self.parser.feed(data):
                if m.crc_ok:
                    got[m.type] = m
        return got

    def set_param(self, pid: int, value: int) -> bool:
        """Send SET_PARAM; True iff ACKed (the ACK's seq is the pid)."""
        self.send(e1proto.build_set_param(pid, value))
        got = self.collect({e1proto.ACK})
        ack = got.get(e1proto.ACK)
        return bool(ack and ack.ok and ack.seq == pid)

    def send_frame(self, seq: int, pixels: bytes,
                   expect_scores: bool) -> tuple:
        """One FRAME down; returns (status, scores, ack)."""
        self.send(e1proto.build_frame(seq, pixels))
        want = {e1proto.STATUS, e1proto.ACK}
        if expect_scores:
            want.add(e1proto.SCORES)
        got = self.collect(want)
        return (got.get(e1proto.STATUS), got.get(e1proto.SCORES),
                got.get(e1proto.ACK))


def err_deg(got: float, want: float) -> float:
    return abs((got - want + 180.0) % 360.0 - 180.0)


# ---------------------------------------------------------------- cases


def case_protocol(fw: Firmware) -> None:
    print("--- protocol")
    c = Client(fw.port)

    c.send(e1proto.build_get_status())
    got = c.collect({e1proto.STATUS})
    st = got.get(e1proto.STATUS)
    check(st is not None, "GET_STATUS answers")
    check(st and st.mode == e1proto.MODE_DEV, "boots in DEV mode")
    check(st and st.frames == 0, "boots with zero frames")

    check(c.set_param(e1proto.PARAM_CONF_MIN, 50), "valid SET_PARAM acked")
    c.send(e1proto.build_set_param(99, 1))
    got = c.collect({e1proto.ACK})
    check(bool(got.get(e1proto.ACK)) and not got[e1proto.ACK].ok,
          "unknown param NAKed")
    c.send(e1proto.build_set_param(e1proto.PARAM_MODE, 9))
    got = c.collect({e1proto.ACK})
    check(bool(got.get(e1proto.ACK)) and not got[e1proto.ACK].ok,
          "bad value NAKed")

    # corrupt FRAME -> NAK; clean FRAME after -> ACK (resync works)
    pixels = gauge_sim.render_frame64(30.0)
    wire = bytearray(e1proto.build_frame(1, pixels))
    wire[100] ^= 0x55
    c.send(bytes(wire))
    got = c.collect({e1proto.ACK}, timeout=3.0)
    check(bool(got.get(e1proto.ACK)) and not got[e1proto.ACK].ok,
          "corrupt FRAME NAKed")
    st, sc, ack = c.send_frame(2, pixels, expect_scores=True)
    check(bool(ack and ack.ok and ack.seq == 2), "clean FRAME after corrupt ACKed")
    check(st is not None and sc is not None, "SCORES + STATUS in DEV")


def case_accuracy(fw: Firmware) -> None:
    print("--- accuracy sweep")
    c = Client(fw.port)
    check(c.set_param(e1proto.PARAM_SMOOTH_SHIFT, 0), "smoothing off")

    seq = 0
    worst = 0.0
    for want in range(-170, 180, 23):
        seq += 1
        pixels = gauge_sim.render_frame64(float(want), noise=4.0, seed=want + 180)
        st, sc, ack = c.send_frame(seq, pixels, expect_scores=True)
        if not (st and sc and ack and ack.ok):
            check(False, f"angle {want}: full reply")
            continue
        e = err_deg(st.angle_deg, want)
        worst = max(worst, e)
        check(e < 2.5, f"angle {want:+4d} -> {st.angle_deg:+7.2f} (err {e:.2f})")
        check(st.needle, f"angle {want:+4d} needle flag set")
        scores = sc.scores()
        peak_cdeg = e1proto.index_to_cdeg(scores.index(max(scores)))
        check(err_deg(peak_cdeg / 100.0, want) < 3.0,
              f"angle {want:+4d} SCORES peak agrees")
    print(f"     worst error {worst:.2f} deg")

    # fabric compute time is reported
    check(st.frame_us > 0, "frame_us reported")


def case_calibration(fw: Firmware) -> None:
    print("--- calibration")
    c = Client(fw.port)
    check(c.set_param(e1proto.PARAM_SMOOTH_SHIFT, 0), "smoothing off")

    # a 0..60 PSI gauge over the default -135..+135 sweep
    check(c.set_param(e1proto.PARAM_CAL_ANGLE_MIN, -13500), "cal amin")
    check(c.set_param(e1proto.PARAM_CAL_ANGLE_MAX, 13500), "cal amax")
    check(c.set_param(e1proto.PARAM_CAL_VALUE_MIN, 0), "cal vmin")
    check(c.set_param(e1proto.PARAM_CAL_VALUE_MAX, 60000), "cal vmax")

    for want_deg, want_val in ((-135.0, 0.0), (0.0, 30.0), (135.0, 60.0),
                               (67.5, 45.0)):
        pixels = gauge_sim.render_frame64(want_deg)
        st, _, ack = c.send_frame(1, pixels, expect_scores=True)
        ok = st is not None and ack is not None and ack.ok
        check(ok and abs(st.reading - want_val) < 1.2,
              f"{want_deg:+7.1f} deg reads {st.reading if st else '?':.2f} "
              f"(want {want_val})")


def case_deploy(fw: Firmware) -> None:
    print("--- deploy mode")
    c = Client(fw.port)
    pixels = gauge_sim.render_frame64(45.0)

    check(c.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEPLOY), "enter DEPLOY")
    st, sc, ack = c.send_frame(1, pixels, expect_scores=False)
    check(bool(ack and ack.ok), "frame ACKed in DEPLOY")
    check(st is not None and st.mode == e1proto.MODE_DEPLOY, "STATUS says DEPLOY")
    check(sc is None and not c.collect({e1proto.SCORES}, timeout=0.5),
          "no SCORES leaves the device in DEPLOY")
    check(c.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEV), "back to DEV")
    st, sc, ack = c.send_frame(2, pixels, expect_scores=True)
    check(sc is not None, "SCORES flow again in DEV")


def case_polarity(fw: Firmware) -> None:
    print("--- polarity")
    c = Client(fw.port)
    check(c.set_param(e1proto.PARAM_SMOOTH_SHIFT, 0), "smoothing off")
    check(c.set_param(e1proto.PARAM_POLARITY, 1), "light-needle mode")
    pixels = gauge_sim.render_frame64(-60.0, polarity=1)
    st, _, ack = c.send_frame(1, pixels, expect_scores=True)
    ok = st is not None and ack is not None and ack.ok
    check(ok and err_deg(st.angle_deg, -60.0) < 2.5,
          f"light needle -60 -> {st.angle_deg if st else '?':+.2f}")


def case_burn(fw: Firmware) -> None:
    print("--- burn mode")
    c = Client(fw.port)
    pixels = gauge_sim.render_frame64(45.0)
    st, _, ack = c.send_frame(1, pixels, expect_scores=True)
    check(bool(ack and ack.ok), "frame ACKed before burn")
    check(c.set_param(e1proto.PARAM_BURN, 1), "burn on -> ACK")
    c.send(e1proto.build_get_status())
    got = c.collect({e1proto.STATUS})
    check(e1proto.STATUS in got, "STATUS still answered while burning")
    check(c.set_param(e1proto.PARAM_BURN, 0), "burn off -> ACK")
    st, _, ack = c.send_frame(2, pixels, expect_scores=True)
    check(bool(ack and ack.ok), "frame ACKed after burn off")
    check(st is not None and st.frames == 2,
          "burn left the frame counter untouched")


def case_smoothing_and_reset(fw: Firmware) -> None:
    print("--- smoothing + reset")
    c = Client(fw.port)
    check(c.set_param(e1proto.PARAM_SMOOTH_SHIFT, 3), "smooth shift 3")

    st1, _, _ = c.send_frame(1, gauge_sim.render_frame64(-90.0), True)
    st2, _, _ = c.send_frame(2, gauge_sim.render_frame64(90.0), True)
    check(st1 is not None and st2 is not None, "both frames answered")
    moved = err_deg(st2.angle_deg, st1.angle_deg)
    check(10.0 < moved < 90.0, f"EMA moved partway ({moved:.1f} deg of 180)")

    check(c.set_param(e1proto.PARAM_RESET, 0), "RESET acked")
    st3, _, _ = c.send_frame(3, gauge_sim.render_frame64(90.0), True)
    check(st3 is not None and err_deg(st3.angle_deg, 90.0) < 2.5,
          "direct reading after RESET")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", default=os.path.join(
        os.path.dirname(__file__), "..", "firmware", "build", "firmware_sim"))
    args = ap.parse_args()

    cases = (case_protocol, case_accuracy, case_calibration, case_deploy,
             case_polarity, case_burn, case_smoothing_and_reset)
    for case in cases:
        with Firmware(args.fw) as fw:
            case(fw)

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILURES:")
        for f in FAILURES:
            print("  " + f)
    else:
        print("all e2e checks passed")
    sys.exit(len(FAILURES))


if __name__ == "__main__":
    main()
