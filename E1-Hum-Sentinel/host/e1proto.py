"""Python mirror of firmware/protocol.{h,c} — framing, CRC16, parsing.

Wire format (little-endian), CRC16-CCITT (poly 0x1021, init 0xFFFF)
over everything after the AA 55 magic.  See protocol.h for the message
table.  Reply order per AUDIO chunk: SPECTRUM (DEV) -> STATUS -> ACK,
with the ACK last (the host's licence to transmit on the half-duplex
EVK link).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"\xAA\x55"

AUDIO = 0x01
SET_PARAM = 0x02
GET_STATUS = 0x03
ACK = 0x10
STATUS = 0x11
SPECTRUM = 0x12

RATE = 8000
AUDIO_BYTES = 1024          # u-law samples per chunk = 128 ms
VIZ_BINS = 128              # wire spectrum resolution (4 kHz span)
SPEC_BYTES = 3 * VIZ_BINS   # spectrum + baseline + trigger planes
NBINS = 512                 # on-chip resolution, 7.8125 Hz per bin
HZ_PER_BIN = RATE / 2 / NBINS

PARAM_THRESHOLD = 1
PARAM_K_Q4 = 2
PARAM_MODE = 3
PARAM_ADAPT_SHIFT = 4
PARAM_MARGIN = 5
PARAM_EVENT_HOLD = 6
PARAM_LEARN_CHUNKS = 7
PARAM_RELEARN = 8  # command: value ignored, resets the baseline
PARAM_BURN = 9  # 1 = fabric power soak between chunks (constant workload)

MODE_DEV = 0
MODE_DEPLOY = 1

FLAG_EVENT = 0x01
FLAG_LEARNING = 0x02

CLASS_NONE, CLASS_LOW, CLASS_MID, CLASS_HIGH = 0, 1, 2, 3
CLASS_NAMES = {
    CLASS_NONE: "none",
    CLASS_LOW: "LOW (thump/rumble)",
    CLASS_MID: "MID (voice)",
    CLASS_HIGH: "HIGH (keys/clink)",
}


def _make_table() -> list[int]:
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if (c & 0x8000) else (c << 1)
        table.append(c & 0xFFFF)
    return table


_TABLE = _make_table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def _wrap(body: bytes) -> bytes:
    return MAGIC + body + struct.pack("<H", crc16(body))


def build_audio(seq: int, ulaw: bytes) -> bytes:
    if len(ulaw) != AUDIO_BYTES:
        raise ValueError(f"chunk must be {AUDIO_BYTES} bytes, got {len(ulaw)}")
    return _wrap(struct.pack("<BBH", AUDIO, seq & 0xFF, len(ulaw)) + ulaw)


def build_set_param(param_id: int, value: int) -> bytes:
    return _wrap(struct.pack("<BBi", SET_PARAM, param_id, value))


def build_get_status() -> bytes:
    return _wrap(bytes([GET_STATUS]))


@dataclass
class Msg:
    type: int
    crc_ok: bool = True
    seq: int = 0
    ok: int = 0
    param_id: int = 0
    value: int = 0
    payload: bytes = b""
    # STATUS
    score: int = 0
    flags: int = 0
    ev_class: int = 0
    mode: int = 0
    learn_pct: int = 0
    top_bin: int = 0
    chunk_us: int = 0
    events: int = 0

    @property
    def event(self) -> bool:
        return bool(self.flags & FLAG_EVENT)

    @property
    def learning(self) -> bool:
        return bool(self.flags & FLAG_LEARNING)

    # SPECTRUM payload planes (u8 log units, VIZ_BINS each)
    @property
    def spec(self) -> bytes:
        return self.payload[:VIZ_BINS]

    @property
    def base(self) -> bytes:
        return self.payload[VIZ_BINS:2 * VIZ_BINS]

    @property
    def trig(self) -> bytes:
        return self.payload[2 * VIZ_BINS:3 * VIZ_BINS]


# body sizes for the fixed-length types (type byte excluded)
_FIXED_BODY = {SET_PARAM: 5, GET_STATUS: 0, ACK: 2, STATUS: 14}
_VAR_CAP = {AUDIO: AUDIO_BYTES, SPECTRUM: SPEC_BYTES}


class Parser:
    """Incremental parser; resyncs on garbage by scanning for AA 55."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[Msg]:
        self.buf += data
        out: list[Msg] = []
        while True:
            msg = self._next()
            if msg is None:
                return out
            if not msg.crc_ok:
                self.crc_errors += 1
            out.append(msg)

    def _next(self) -> Msg | None:
        buf = self.buf
        i = buf.find(MAGIC)
        if i < 0:
            # keep a trailing 0xAA: it may be the start of a split magic
            del buf[: len(buf) - 1 if buf[-1:] == b"\xAA" else len(buf)]
            return None
        if i:
            del buf[:i]
        if len(buf) < 3:
            return None

        t = buf[2]
        if t in _FIXED_BODY:
            total = 2 + 1 + _FIXED_BODY[t] + 2
        elif t in _VAR_CAP:
            if len(buf) < 6:
                return None
            length = struct.unpack_from("<H", buf, 4)[0]
            if not 0 < length <= _VAR_CAP[t]:
                del buf[:2]  # bogus length: resync past this magic
                return self._next()
            total = 2 + 1 + 3 + length + 2
        else:
            del buf[:2]  # unknown type: resync past this magic
            return self._next()

        if len(buf) < total:
            return None

        body = bytes(buf[2 : total - 2])
        rx_crc = struct.unpack_from("<H", buf, total - 2)[0]
        del buf[:total]
        return self._parse(body, crc16(body) == rx_crc)

    @staticmethod
    def _parse(body: bytes, crc_ok: bool) -> Msg:
        t = body[0]
        m = Msg(type=t, crc_ok=crc_ok)
        if t in (AUDIO, SPECTRUM):
            m.seq = body[1]
            m.payload = body[4 : 4 + struct.unpack_from("<H", body, 2)[0]]
        elif t == SET_PARAM:
            m.param_id, m.value = struct.unpack_from("<Bi", body, 1)
        elif t == ACK:
            m.seq, m.ok = body[1], body[2]
        elif t == STATUS:
            (m.score, m.flags, m.ev_class, m.mode, m.learn_pct,
             m.top_bin, m.chunk_us, m.events) = struct.unpack_from(
                "<HBBBBHIH", body, 1)
        return m
