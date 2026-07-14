"""Python mirror of firmware/protocol.{h,c} — framing, CRC16, parsing.

Wire format (little-endian), CRC16-CCITT (poly 0x1021, init 0xFFFF)
over everything after the AA 55 magic.  See protocol.h for the message
table.  Reply order per FRAME: SCORES (DEV) -> STATUS -> ACK, with the
ACK last (the host's licence to transmit on the half-duplex EVK link).

Angle convention: 0 = 12 o'clock, clockwise positive, centidegrees in
[-18000, +18000).  SCORES index ai <-> -18000 + ai*STEP_CDEG.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"\xAA\x55"

FRAME = 0x01
SET_PARAM = 0x02
GET_STATUS = 0x03
ACK = 0x10
STATUS = 0x11
SCORES = 0x12

FRAME_W = 64
FRAME_H = 64
FRAME_BYTES = FRAME_W * FRAME_H
N_ANGLES = 240
STEP_CDEG = 36000 // N_ANGLES
SCORE_BYTES = 2 * N_ANGLES

PARAM_MODE = 1
PARAM_POLARITY = 2
PARAM_SMOOTH_SHIFT = 3
PARAM_CONF_MIN = 4
PARAM_CAL_ANGLE_MIN = 5
PARAM_CAL_ANGLE_MAX = 6
PARAM_CAL_VALUE_MIN = 7
PARAM_CAL_VALUE_MAX = 8
PARAM_RESET = 9  # command: value ignored, clears the EMA state
PARAM_BURN = 10  # 1 = fabric power soak between frames (constant workload)

MODE_DEV = 0
MODE_DEPLOY = 1

FLAG_NEEDLE = 0x01


def index_to_cdeg(ai: int) -> int:
    return -18000 + ai * STEP_CDEG


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


def build_frame(seq: int, pixels: bytes) -> bytes:
    if len(pixels) != FRAME_BYTES:
        raise ValueError(f"frame must be {FRAME_BYTES} bytes, got {len(pixels)}")
    return _wrap(struct.pack("<BBH", FRAME, seq & 0xFF, len(pixels)) + pixels)


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
    angle_cdeg: int = 0
    value_milli: int = 0
    confidence: int = 0
    flags: int = 0
    mode: int = 0
    frame_us: int = 0
    frames: int = 0
    mean: int = 0

    @property
    def needle(self) -> bool:
        return bool(self.flags & FLAG_NEEDLE)

    @property
    def angle_deg(self) -> float:
        return self.angle_cdeg / 100.0

    @property
    def reading(self) -> float:
        return self.value_milli / 1000.0

    def scores(self) -> list[int]:
        """SCORES payload as N_ANGLES ints."""
        return list(struct.unpack(f"<{N_ANGLES}H", self.payload))


# body sizes for the fixed-length types (type byte excluded)
_FIXED_BODY = {SET_PARAM: 5, GET_STATUS: 0, ACK: 2, STATUS: 16}
_VAR_CAP = {FRAME: FRAME_BYTES, SCORES: SCORE_BYTES}


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
        if t in (FRAME, SCORES):
            m.seq = body[1]
            m.payload = body[4 : 4 + struct.unpack_from("<H", body, 2)[0]]
        elif t == SET_PARAM:
            m.param_id, m.value = struct.unpack_from("<Bi", body, 1)
        elif t == ACK:
            m.seq, m.ok = body[1], body[2]
        elif t == STATUS:
            (m.angle_cdeg, m.value_milli, m.confidence, m.flags, m.mode,
             m.frame_us, m.frames, m.mean) = struct.unpack_from(
                "<hiBBBIHB", body, 1)
        return m
