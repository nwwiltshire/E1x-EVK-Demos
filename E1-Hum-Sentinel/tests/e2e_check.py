#!/usr/bin/env python3
"""End-to-end test: real firmware binary, synthetic room audio.

For each case this script starts a fresh firmware_sim process (on an
ephemeral port parsed from its stderr), streams u-law chunks over TCP
exactly like the live viewer would, and asserts the detector behavior:
learning completes, a quiet room stays quiet, injected anomalies fire
events of the right class, DEPLOY mode leaks no spectrum.  Also
exercises the protocol edge cases (NAK on CRC error, SET_PARAM
validation).  Needs numpy (for the u-law codec); run via test_e2e.sh.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import socket
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "host"))
import e1proto  # noqa: E402
from audio_source import SynthSource, ulaw_encode  # noqa: E402

FAILURES = []
LEARN_CHUNKS = 8


def check(cond: bool, what: str) -> None:
    tag = "ok  " if cond else "FAIL"
    print(f"  {tag} {what}")
    if not cond:
        FAILURES.append(what)


class Firmware:
    """One firmware_sim process on an ephemeral loopback port."""

    def __init__(self, binary: str) -> None:
        self.proc = subprocess.Popen(
            [binary], env={"E1_SIM_PORT": "0"},
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
        # keep draining stderr so the pipe never fills
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self) -> None:
        for _ in self.proc.stderr:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.proc.kill()
        self.proc.wait()


class Client:
    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(10)
        self.parser = e1proto.Parser()
        self.pending: list[e1proto.Msg] = []
        self.spectra = 0
        self.source = SynthSource(seed=99)
        self.seq = 0

    def _recv_msgs(self) -> list[e1proto.Msg]:
        msgs = self.pending
        self.pending = []
        while not msgs:
            data = self.sock.recv(65536)
            if not data:
                raise RuntimeError("firmware closed the connection")
            msgs = self.parser.feed(data)
        for m in msgs:
            if m.type == e1proto.SPECTRUM:
                self.spectra += 1
        return msgs

    def wait_for(self, mtype: int) -> e1proto.Msg:
        while True:
            msgs = self._recv_msgs()
            for i, m in enumerate(msgs):
                if m.type == mtype:
                    self.pending = msgs[i + 1:] + self.pending
                    return m

    def send_chunk(self):
        """Send one AUDIO chunk; returns (ack_ok, status) once both the
        ACK and the per-chunk STATUS have arrived."""
        pcm = self.source.get_chunk(e1proto.AUDIO_BYTES)
        self.seq = (self.seq + 1) & 0xFF
        self.sock.sendall(e1proto.build_audio(self.seq, ulaw_encode(pcm).tobytes()))
        ack = status = None
        while ack is None or status is None:
            for m in self._recv_msgs():
                if m.type == e1proto.ACK:
                    ack = m
                elif m.type == e1proto.STATUS:
                    status = m
        return ack.ok == 1, status

    def set_param(self, pid: int, value: int) -> bool:
        self.sock.sendall(e1proto.build_set_param(pid, value))
        return self.wait_for(e1proto.ACK).ok == 1

    def stream(self, n: int) -> list[e1proto.Msg]:
        out = []
        for _ in range(n):
            ok, status = self.send_chunk()
            if not ok:
                raise RuntimeError("chunk NAKed")
            out.append(status)
        return out

    def learn(self) -> list[e1proto.Msg]:
        """Fast relearn: LEARN_CHUNKS quiet chunks."""
        assert self.set_param(e1proto.PARAM_LEARN_CHUNKS, LEARN_CHUNKS)
        assert self.set_param(e1proto.PARAM_RELEARN, 0)
        return self.stream(LEARN_CHUNKS)


def case_protocol(binary: str) -> None:
    print("case: protocol behavior")
    with Firmware(binary) as fw:
        c = Client(fw.port)
        c.sock.sendall(e1proto.build_get_status())
        st = c.wait_for(e1proto.STATUS)
        check(st.mode == e1proto.MODE_DEV, "GET_STATUS answered; boots in DEV mode")
        check(st.learning, "boots in learning state")

        check(c.set_param(e1proto.PARAM_THRESHOLD, 60), "SET_PARAM valid -> ACK")
        check(not c.set_param(99, 1), "SET_PARAM unknown id -> NAK")
        check(not c.set_param(e1proto.PARAM_MODE, 7), "SET_PARAM bad value -> NAK")

        pcm = c.source.get_chunk(e1proto.AUDIO_BYTES)
        corrupt = bytearray(e1proto.build_audio(0, ulaw_encode(pcm).tobytes()))
        corrupt[100] ^= 0xFF
        c.sock.sendall(bytes(corrupt))
        check(c.wait_for(e1proto.ACK).ok == 0, "corrupt AUDIO -> NAK")

        ok, _ = c.send_chunk()
        check(ok, "clean AUDIO right after a NAK -> ACK (resync works)")


def case_learn_and_quiet(binary: str) -> None:
    print("case: learning completes; quiet room stays quiet (DEV)")
    with Firmware(binary) as fw:
        c = Client(fw.port)
        during = c.learn()
        check(all(s.learning for s in during[:-1]),
              "learning flag set while learning")
        check(not during[-1].learning and during[-1].learn_pct == 100,
              "learning completes after LEARN_CHUNKS chunks")

        statuses = c.stream(12)
        scores = [s.score for s in statuses]
        check(max(scores) < 60, f"quiet scores stay low (max={max(scores)})")
        check(not any(s.event for s in statuses), "no events in a quiet room")
        check(all(s.chunk_us > 0 for s in statuses), "chunk_us is measured")
        check(c.spectra == LEARN_CHUNKS + 12, "DEV mode: SPECTRUM per chunk")


def case_anomalies(binary: str) -> None:
    print("case: injected anomalies fire classed events (DEV)")
    with Firmware(binary) as fw:
        c = Client(fw.port)
        c.learn()
        c.stream(4)

        c.source.inject("jingle")
        statuses = c.stream(5)
        fired = [s for s in statuses if s.event]
        check(len(fired) > 0, "jingle fires an event")
        check(fired and fired[-1].ev_class == e1proto.CLASS_HIGH,
              f"jingle classed HIGH (got {fired[-1].ev_class if fired else '-'})")
        check(max(s.score for s in statuses) >= 60,
              f"jingle score over threshold (max={max(s.score for s in statuses)})")

        statuses = c.stream(10)
        check(not statuses[-1].event, "event clears in silence")
        check(statuses[-1].events == 1, "event counted once")

        c.source.inject("thump")
        statuses = c.stream(5)
        fired = [s for s in statuses if s.event]
        check(len(fired) > 0, "thump fires an event")
        check(fired and fired[-1].ev_class == e1proto.CLASS_LOW,
              f"thump classed LOW (got {fired[-1].ev_class if fired else '-'})")

        c.stream(10)
        c.source.inject("voice")
        statuses = c.stream(6)
        fired = [s for s in statuses if s.event]
        check(len(fired) > 0, "voice fires an event")
        check(fired and fired[-1].ev_class == e1proto.CLASS_MID,
              f"voice classed MID (got {fired[-1].ev_class if fired else '-'})")


def case_relearn(binary: str) -> None:
    print("case: RELEARN resets the baseline")
    with Firmware(binary) as fw:
        c = Client(fw.port)
        c.learn()
        c.stream(2)
        check(c.set_param(e1proto.PARAM_RELEARN, 0), "RELEARN acked")
        _, st = c.send_chunk()
        check(st.learning, "back in learning state")


def case_deploy(binary: str) -> None:
    print("case: DEPLOY — no spectrum data may leave")
    with Firmware(binary) as fw:
        c = Client(fw.port)
        c.learn()
        check(c.set_param(e1proto.PARAM_MODE, e1proto.MODE_DEPLOY),
              "switch to DEPLOY acked")
        c.spectra = 0
        c.source.inject("jingle")
        statuses = c.stream(5)
        check(c.spectra == 0, "zero SPECTRUM messages received")
        check(any(s.event for s in statuses), "event detection still works")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", required=True, help="path to firmware_sim")
    args = ap.parse_args()

    for case in (case_protocol, case_learn_and_quiet, case_anomalies,
                 case_relearn, case_deploy):
        case(args.fw)

    if FAILURES:
        print(f"\ne2e: {len(FAILURES)} check(s) FAILED")
        sys.exit(1)
    print("\ne2e: all checks passed")


if __name__ == "__main__":
    main()
