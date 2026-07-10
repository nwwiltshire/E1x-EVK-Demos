"""Audio sources for the E1 acoustic sentinel + the G.711 u-law codec.

Everything yields 8 kHz mono int16 numpy arrays via get_chunk(n):

  MicSource   — laptop microphone through an `arecord` subprocess
                (raw S16_LE @ 8 kHz; ALSA's plug layer resamples).
                Keeps a short ring of the most recent audio and always
                hands out the newest n samples, so a link slower than
                real time drops old audio instead of lagging.
  SynthSource — deterministic room simulation: mains-hum harmonics +
                low white noise, with injectable anomalies
                ('thump' | 'voice' | 'jingle') for testing and for
                driving the demo without a working mic.
  WavSource   — a WAV file resampled to 8 kHz, looped.

The u-law codec here is the exact mirror of the firmware's decoder
(detector.c); tests assert the two agree byte-for-byte.
"""

from __future__ import annotations

import subprocess
import threading

import numpy as np

RATE = 8000

# ------------------------------------------------------------------ #
# G.711 u-law                                                        #
# ------------------------------------------------------------------ #

_BIAS = 0x84
_CLIP = 32635


def ulaw_encode(pcm: np.ndarray) -> np.ndarray:
    """int16 -> u-law bytes (Sun/G.711 convention, complemented)."""
    x = pcm.astype(np.int32)
    sign = np.where(x < 0, 0x80, 0)
    mag = np.minimum(np.abs(x), _CLIP) + _BIAS  # 132..32767
    seg = (np.floor(np.log2(mag)).astype(np.int32) - 7).clip(0, 7)
    uval = sign | (seg << 4) | ((mag >> (seg + 3)) & 0x0F)
    return (~uval & 0xFF).astype(np.uint8)


def _decode_one(b: int) -> int:
    u = ~b & 0xFF
    t = (((u & 0x0F) << 3) + _BIAS) << ((u & 0x70) >> 4)
    return (_BIAS - t) if (u & 0x80) else (t - _BIAS)


ULAW_TABLE = np.array([_decode_one(b) for b in range(256)], np.int16)


def ulaw_decode(ulaw: np.ndarray | bytes) -> np.ndarray:
    return ULAW_TABLE[np.frombuffer(bytes(ulaw), np.uint8)]


# ------------------------------------------------------------------ #
# Sources                                                            #
# ------------------------------------------------------------------ #

class Injector:
    """Mixes synthetic anomaly waveforms into any source's chunks —
    phase-continuous, spanning chunk boundaries.  Lets the demo fire a
    thump/voice/jingle on a keypress even when the audio comes from a
    real microphone (or from a dead-silent VM one)."""

    KINDS = ("thump", "voice", "jingle")

    def __init__(self, seed: int = 1234) -> None:
        self.rng = np.random.default_rng(seed)
        self._tail = np.zeros(0, np.float64)  # pending event samples

    def inject(self, kind: str) -> None:
        ev = self._make_event(kind)
        if len(ev) > len(self._tail):
            self._tail = np.pad(self._tail, (0, len(ev) - len(self._tail)))
        self._tail[: len(ev)] += ev

    def _make_event(self, kind: str) -> np.ndarray:
        t = np.arange(int(0.5 * RATE)) / RATE
        if kind == "thump":  # decaying low knock, < 250 Hz
            env = np.exp(-t / 0.08)
            return 9000.0 * env * np.sin(2 * np.pi * 70.0 * t)
        if kind == "voice":  # band-limited noise burst, 300-1500 Hz
            noise = self.rng.standard_normal(len(t))
            spec = np.fft.rfft(noise)
            f = np.fft.rfftfreq(len(t), 1.0 / RATE)
            spec[(f < 300) | (f > 1500)] = 0
            band = np.fft.irfft(spec, len(t))
            band /= max(1e-9, np.max(np.abs(band)))
            # decay slow enough that chunk 2 still clears the trigger:
            # the detector needs 2 consecutive over-threshold chunks
            env = np.minimum(t / 0.02, 1.0) * np.exp(-t / 0.22)
            return 9000.0 * env * band
        if kind == "jingle":  # keys: high tones, AM shimmer, > 2 kHz
            env = np.exp(-t / 0.25) * (1.0 + 0.6 * np.sin(2 * np.pi * 9.0 * t))
            return env * (3500.0 * np.sin(2 * np.pi * 3300.0 * t) +
                          2500.0 * np.sin(2 * np.pi * 3700.0 * t))
        raise ValueError(f"unknown event kind {kind!r} (use {self.KINDS})")

    def mix(self, pcm: np.ndarray) -> np.ndarray:
        if not len(self._tail):
            return pcm
        x = pcm.astype(np.float64)
        k = min(len(x), len(self._tail))
        x[:k] += self._tail[:k]
        self._tail = self._tail[k:]
        return np.clip(x, -32768, 32767).astype(np.int16)


class SynthSource:
    """Room-tone simulator: 120 Hz hum + harmonics + white noise.

    inject() mixes an anomaly into the samples that get_chunk() will
    return next.
    """

    name = "synth"
    KINDS = Injector.KINDS

    def __init__(self, seed: int = 1234) -> None:
        self.rng = np.random.default_rng(seed)
        self.n = 0  # absolute sample counter (keeps hum phase continuous)
        self.injector = Injector(seed)

    def inject(self, kind: str) -> None:
        self.injector.inject(kind)

    def get_chunk(self, n: int) -> np.ndarray:
        t = (self.n + np.arange(n)) / RATE
        self.n += n
        x = (1200.0 * np.sin(2 * np.pi * 120.0 * t) +
             500.0 * np.sin(2 * np.pi * 240.0 * t) +
             250.0 * np.sin(2 * np.pi * 360.0 * t) +
             120.0 * self.rng.standard_normal(n))
        pcm = np.clip(x, -32768, 32767).astype(np.int16)
        return self.injector.mix(pcm)

    def close(self) -> None:
        pass


class MicSource:
    """Microphone via arecord; hands out the newest n samples."""

    name = "mic"

    def __init__(self, device: str = "default") -> None:
        self.proc = subprocess.Popen(
            ["arecord", "-q", "-f", "S16_LE", "-r", str(RATE), "-c", "1",
             "-t", "raw", "-D", device],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        self.lock = threading.Lock()
        self.ring = bytearray()
        self.ring_cap = RATE * 2 * 4  # keep the last ~4 s
        self.total = 0
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        while True:
            data = self.proc.stdout.read(4096)
            if not data:
                return  # arecord died; alive() goes False
            with self.lock:
                self.ring += data
                self.total += len(data)
                if len(self.ring) > self.ring_cap:
                    del self.ring[: len(self.ring) - self.ring_cap]

    def alive(self) -> bool:
        return self.proc.poll() is None

    def get_chunk(self, n: int) -> np.ndarray:
        with self.lock:
            raw = bytes(self.ring[-2 * n:])
        pcm = np.frombuffer(raw, np.int16)
        if len(pcm) < n:  # startup: zero-pad the front
            pcm = np.pad(pcm, (n - len(pcm), 0))
        return pcm

    def close(self) -> None:
        self.proc.kill()


class WavSource:
    """A WAV file resampled to 8 kHz mono, looped."""

    def __init__(self, path: str) -> None:
        import wave

        self.name = f"wav:{path}"
        with wave.open(path, "rb") as w:
            nch, sw, rate = w.getnchannels(), w.getsampwidth(), w.getframerate()
            raw = w.readframes(w.getnframes())
        if sw != 2:
            raise ValueError(f"{path}: need 16-bit PCM, got {8 * sw}-bit")
        pcm = np.frombuffer(raw, np.int16).reshape(-1, nch).mean(axis=1)
        if rate != RATE:
            src_t = np.arange(len(pcm)) / rate
            dst_t = np.arange(int(len(pcm) * RATE / rate)) / RATE
            pcm = np.interp(dst_t, src_t, pcm)
        self.pcm = pcm.astype(np.int16)
        if len(self.pcm) < RATE:
            raise ValueError(f"{path}: shorter than 1 s")
        self.pos = 0

    def get_chunk(self, n: int) -> np.ndarray:
        out = np.empty(n, np.int16)
        got = 0
        while got < n:
            k = min(n - got, len(self.pcm) - self.pos)
            out[got:got + k] = self.pcm[self.pos:self.pos + k]
            self.pos = (self.pos + k) % len(self.pcm)
            got += k
        return out

    def close(self) -> None:
        pass
