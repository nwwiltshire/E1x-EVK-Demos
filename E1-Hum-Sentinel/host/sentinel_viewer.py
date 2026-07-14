#!/usr/bin/env python3
"""Live viewer for the E1 acoustic anomaly detector (Hum Sentinel).

Streams 128 ms u-law audio chunks to the firmware (real EVK over
serial, or the simulator over TCP) and visualizes what comes back:

  DEV mode    — live spectrum vs the learned baseline + trigger
                envelope, scrolling spectrogram, anomaly score,
                classed event log ("watch it think")
  DEPLOY mode — score + events only; the spectrum panels become a
                placard, because no audio-derived data leaves the chip.

Audio comes from the laptop mic (arecord), a synthetic room tone, or a
WAV file.  Whatever the source, keys 1/2/3 inject a synthetic
thump/voice/jingle into the outgoing stream — so the demo works even
with a dead VM microphone.

Usage:
  python3 sentinel_viewer.py --sim 127.0.0.1:5555            # simulator
  python3 sentinel_viewer.py --port /dev/ttyACM2             # EVK
  ... [--source mic|synth|wav:PATH] [--power-port /dev/ttyACM1]
      [--record out.mp4]

Keys:  d = DEV/DEPLOY   r = relearn baseline   t/T = threshold -/+10
       1/2/3 = inject thump/voice/jingle   q = quit
       b = burn (constant-workload power soak; streaming pauses)
"""

from __future__ import annotations

import argparse
import collections
import queue
import signal
import socket
import sys
import threading
import time

try:
    import cv2
    import numpy as np
except ImportError as e:
    sys.exit(f"missing dependency: {e.name}. Install with: pip install -r host/requirements.txt")

import e1proto
from audio_source import Injector, MicSource, SynthSource, WavSource, ulaw_encode
from power_monitor import (CHIP_RAIL, RAILS, MockPowerMonitor,
                           SerialPowerMonitor, battery_hours, duty_avg_mw,
                           duty_battery_hours, fmt_mw, fmt_runtime)

CHUNK_SECS = e1proto.AUDIO_BYTES / e1proto.RATE  # 0.128 s

M = 10
CANVAS_W, CANVAS_H = 1120, 720
HDR_H = 36
LEFT_W = 640
SPEC_H = 280
FALL_H = 200
RIGHT_X = M + LEFT_W + M
RIGHT_W = CANVAS_W - RIGHT_X - M
CHART_H = 180
EVLOG_H = 190
BOTTOM_Y = M + HDR_H + M + SPEC_H + M + FALL_H + M  # shared by both columns

GREEN = (80, 220, 80)
WHITE = (235, 235, 235)
GRAY = (150, 150, 150)
DARKGRAY = (90, 90, 90)
YELLOW = (60, 200, 240)
RED = (70, 70, 235)
BG = (24, 24, 24)
PANEL = (34, 34, 34)
FONT = cv2.FONT_HERSHEY_SIMPLEX

SCORE_CAP = 5000.0  # log-scale ceiling of the score chart
DISP_FLOOR = 48     # u8 log units hidden below the display floor: the
                    # firmware's mag^2 floor parks silent bins at ~64,
                    # so zoom the interesting 48..255 range to full height


def disp(v) -> np.ndarray:
    """u8 spectrum values -> display scale (floor-subtracted)."""
    a = np.asarray(v, np.int32)
    return np.clip((a - DISP_FLOOR) * 255 // (255 - DISP_FLOOR), 0, 255)


def fmt_hz(hz: float) -> str:
    return f"{hz:.0f} Hz" if hz < 1000 else f"{hz / 1000:.2f} kHz"


class LinkError(Exception):
    pass


class TcpLink:
    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.05)

    def read(self) -> bytes:
        try:
            data = self.sock.recv(65536)
        except socket.timeout:
            return b""
        except OSError as e:
            raise LinkError(str(e))
        if not data:
            raise LinkError("firmware closed the connection")
        return data

    def write(self, data: bytes) -> None:
        try:
            self.sock.sendall(data)
        except OSError as e:
            raise LinkError(str(e))


class SerialLink:
    def __init__(self, port: str, baud: int) -> None:
        import serial

        self.ser = serial.Serial(port, baud, timeout=0.05, rtscts=True)

    def read(self) -> bytes:
        try:
            return self.ser.read(65536)
        except Exception as e:
            raise LinkError(str(e))

    def write(self, data: bytes) -> None:
        try:
            self.ser.write(data)
        except Exception as e:
            raise LinkError(str(e))


class Receiver(threading.Thread):
    """Owns the read side of the link: parses everything the firmware
    sends, queues ACKs for the sender, keeps the latest STATUS/SPECTRUM
    plus ordered queues for the score history and the waterfall."""

    def __init__(self, link) -> None:
        super().__init__(daemon=True)
        self.link = link
        self.parser = e1proto.Parser()
        self.acks: queue.Queue = queue.Queue()
        self.statuses: queue.Queue = queue.Queue()  # every STATUS, in order
        self.spectra: queue.Queue = queue.Queue()   # every SPECTRUM, in order
        self.lock = threading.Lock()
        self.status = None    # last STATUS Msg
        self.spectrum = None  # last SPECTRUM Msg
        self.n_spectra = 0
        self.error: str | None = None

    def run(self) -> None:
        while True:
            try:
                data = self.link.read()
            except LinkError as e:
                self.error = str(e)
                self.acks.put(None)  # wake the sender
                return
            if not data:
                continue
            for m in self.parser.feed(data):
                if not m.crc_ok:
                    continue
                if m.type == e1proto.ACK:
                    self.acks.put((m.seq, m.ok))
                elif m.type == e1proto.STATUS:
                    with self.lock:
                        self.status = m
                    self.statuses.put(m)
                elif m.type == e1proto.SPECTRUM:
                    with self.lock:
                        self.spectrum = m
                        self.n_spectra += 1
                    self.spectra.put(m)


def make_source(spec: str):
    if spec == "synth":
        return SynthSource()
    if spec == "mic":
        try:
            src = MicSource()
        except FileNotFoundError:
            print("warning: arecord not found — falling back to synthetic audio",
                  file=sys.stderr)
            return SynthSource()
        time.sleep(0.3)
        if not src.alive():
            print("warning: arecord exited (no capture device?) — "
                  "falling back to synthetic audio", file=sys.stderr)
            return SynthSource()
        return src
    if spec.startswith("wav:"):
        return WavSource(spec[4:])
    sys.exit(f"unknown --source {spec!r} (use mic, synth, or wav:PATH)")


class App:
    def __init__(self, args) -> None:
        if args.sim:
            host, _, port = args.sim.partition(":")
            self.link = TcpLink(host, int(port or 5555))
        else:
            self.link = SerialLink(args.port, args.baud)
        self.rx = Receiver(self.link)
        self.rx.start()
        # All link writes happen on this thread.  A serial write can
        # block for seconds (kernel CDC buffer full while the
        # half-duplex bridge holds the line), and any multi-second
        # stall on the GUI thread makes the window manager pop the
        # "not responding" dialog.
        self._tx: queue.Queue = queue.Queue()
        self.tx_error: str | None = None
        threading.Thread(target=self._tx_writer, daemon=True).start()

        if args.power_port:
            self.power = SerialPowerMonitor(args.power_port, args.power_baud)
        else:
            self.power = MockPowerMonitor()
        self.power.start()

        self.source = make_source(args.source)
        self.injector = Injector(seed=7)

        self.writer = None
        if args.record:
            self.writer = cv2.VideoWriter(
                args.record, cv2.VideoWriter_fourcc(*"mp4v"), 10,
                (CANVAS_W, CANVAS_H),
            )
            if not self.writer.isOpened():
                sys.exit(f"cannot open --record output {args.record}")
        self.record_path = args.record

        self.seq = 0
        self.chunk_rate = 0.0  # chunks/s EMA
        self._last_ack_t = None
        self.threshold = 60
        self.retries = 0
        self.chunks_sent = 0
        self._next_send = 0.0
        # burn mode: the firmware loops the fabric pipeline and we stop
        # streaming audio (16-byte EVK rx FIFO — see README burn section)
        self.burn = False
        self.workload_mw = args.workload_mw
        self.workload_runtime = args.workload_runtime
        self.workload_period = args.workload_period
        self.sleep_mw = args.sleep_mw
        # one message in flight — a chunk or a param — resolved by
        # polling, never by blocking: the GUI thread must keep pumping
        # events.  On the EVK a DEV chunk's ACK takes ~150-190 ms (1KB
        # up + 0.5KB back at ~10.8KB/s; the ACK is sent last).
        # {"kind": "chunk"|"param", "wire": bytes, "t": float,
        #  "attempt": int, "pid": int, "value": int}
        self._inflight: dict | None = None
        self._timeouts = {"chunk": 10.0, "param": 2.0}
        # param changes wait until nothing is in flight: the EVK link
        # is half-duplex, so never transmit while a reply is inbound
        self._pending_params: list = []

        # score history + event log, fed from the STATUS queue
        self.history: collections.deque = collections.deque(maxlen=400)
        self.events: collections.deque = collections.deque(maxlen=8)
        self._ev_open = False
        self.chunk_us_avg = 0.0
        self.level_db = -120.0

        # waterfall: newest row at the bottom
        self.fall = np.zeros((FALL_H, e1proto.VIZ_BINS), np.uint8)

        self._relearn_on_connect = not args.no_relearn
        self._window_seen = False
        self._quit = False
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, "_quit", True))
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "_quit", True))

    # --- protocol helpers -------------------------------------------

    def _tx_writer(self) -> None:
        """Owns the (possibly blocking) write side of the link."""
        while True:
            data = self._tx.get()
            try:
                self.link.write(data)
            except LinkError as e:
                self.tx_error = str(e)
                return

    def handshake(self) -> None:
        self._tx.put(e1proto.build_get_status())
        for _ in range(60):
            with self.rx.lock:
                if self.rx.status is not None:
                    return
            time.sleep(0.05)
        sys.exit("no STATUS from firmware — is it running? "
                 "(sim: make sim && firmware/build/firmware_sim; EVK: flash hum_sentinel)")

    def start_chunk(self, now: float) -> None:
        pcm = self.injector.mix(self.source.get_chunk(e1proto.AUDIO_BYTES))
        rms = float(np.sqrt(np.mean(pcm.astype(np.float64) ** 2)))
        self.level_db = 20.0 * np.log10(max(rms, 1e-3) / 32768.0)
        # pace against real time without quantizing to a 128 ms grid:
        # when the link is the bottleneck (EVK), send the moment the
        # ACK clears; when it isn't (sim), hold to one chunk per 128 ms
        self._next_send = now + CHUNK_SECS
        self.seq = (self.seq + 1) & 0xFF
        wire = e1proto.build_audio(self.seq, ulaw_encode(pcm).tobytes())
        self._inflight = {"kind": "chunk", "wire": wire, "t": now, "attempt": 1}
        self._tx.put(wire)

    def start_param(self, pid: int, value: int, now: float) -> None:
        wire = e1proto.build_set_param(pid, value)
        self._inflight = {"kind": "param", "wire": wire, "t": now,
                          "attempt": 1, "pid": pid, "value": value}
        self._tx.put(wire)

    def poll_ack(self, now: float) -> None:
        """Non-blocking: resolve the in-flight message if its ACK arrived."""
        inf = self._inflight
        try:
            ack = self.rx.acks.get_nowait()
        except queue.Empty:
            if now - inf["t"] > self._timeouts[inf["kind"]]:
                self._retry_or_drop(now)
            return
        if ack is None:  # receiver died
            sys.exit(f"link error: {self.rx.error}")
        seq, ok = ack
        if inf["kind"] == "param":
            if seq == inf["pid"]:
                if not ok:
                    print(f"warning: SET_PARAM {inf['pid']}={inf['value']} "
                          f"rejected", file=sys.stderr)
                self._inflight = None
            # else: stale chunk ACK from an earlier attempt — ignore
            return
        if ok and seq == self.seq:
            self._inflight = None
            self.chunks_sent += 1
            done = time.monotonic()
            if self._last_ack_t is not None:
                inst = 1.0 / max(done - self._last_ack_t, 1e-6)
                self.chunk_rate = (inst if self.chunk_rate == 0
                                   else 0.8 * self.chunk_rate + 0.2 * inst)
            self._last_ack_t = done
        elif not ok:
            self._retry_or_drop(now)
        # else: stale ACK from an earlier attempt — keep waiting

    def _retry_or_drop(self, now: float) -> None:
        inf = self._inflight
        if inf["attempt"] >= 3:
            self._inflight = None  # dropped; keep the demo alive
            if inf["kind"] == "param":
                print(f"warning: SET_PARAM {inf['pid']}={inf['value']} "
                      f"not acked", file=sys.stderr)
        else:
            self.retries += 1
            inf["t"] = now
            inf["attempt"] += 1
            self._tx.put(inf["wire"])

    def send_param(self, pid: int, value: int) -> None:
        """Queued; sent by the main loop when the line is quiet."""
        self._pending_params.append((pid, value))

    # --- incoming data -----------------------------------------------

    def drain_rx(self) -> None:
        while True:
            try:
                st = self.rx.statuses.get_nowait()
            except queue.Empty:
                break
            self.history.append(st)
            if st.chunk_us:
                self.chunk_us_avg = (st.chunk_us if self.chunk_us_avg == 0 else
                                     0.9 * self.chunk_us_avg + 0.1 * st.chunk_us)
            if st.event and not self._ev_open:
                self._ev_open = True
                self.events.appendleft({
                    "t": time.strftime("%H:%M:%S"),
                    "cls": st.ev_class, "peak": st.score,
                    "hz": st.top_bin * e1proto.HZ_PER_BIN, "chunks": 1,
                })
            elif st.event and self.events:
                ev = self.events[0]
                ev["chunks"] += 1
                if st.score >= ev["peak"]:
                    ev.update(peak=st.score, cls=st.ev_class,
                              hz=st.top_bin * e1proto.HZ_PER_BIN)
            elif not st.event:
                self._ev_open = False
        while True:
            try:
                sp = self.rx.spectra.get_nowait()
            except queue.Empty:
                break
            self.fall = np.roll(self.fall, -1, axis=0)
            self.fall[-1] = np.frombuffer(sp.spec, np.uint8)

    # --- keys --------------------------------------------------------

    def handle_key(self, key: int, mode: int) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("d"):
            self.send_param(
                e1proto.PARAM_MODE,
                e1proto.MODE_DEPLOY if mode == e1proto.MODE_DEV else e1proto.MODE_DEV,
            )
        elif key == ord("r"):
            self.send_param(e1proto.PARAM_RELEARN, 0)
        elif key == ord("t"):
            self.threshold = max(10, self.threshold - 10)
            self.send_param(e1proto.PARAM_THRESHOLD, self.threshold)
        elif key == ord("T"):
            self.threshold = min(2000, self.threshold + 10)
            self.send_param(e1proto.PARAM_THRESHOLD, self.threshold)
        elif key == ord("b"):
            self.burn = not self.burn
            self.send_param(e1proto.PARAM_BURN, 1 if self.burn else 0)
        elif key in (ord("1"), ord("2"), ord("3")):
            self.injector.inject(Injector.KINDS[key - ord("1")])
        return True

    # --- rendering ---------------------------------------------------

    @staticmethod
    def _panel(canvas, x, y, w, h, title=None, color=GRAY):
        cv2.rectangle(canvas, (x, y), (x + w, y + h), PANEL, -1)
        if title:
            cv2.putText(canvas, title, (x + 8, y + 20), FONT, 0.5, color, 1)

    @staticmethod
    def _score_y(score: float, y0: int, h: int) -> int:
        f = np.log10(1.0 + max(0.0, score)) / np.log10(1.0 + SCORE_CAP)
        return y0 + h - int(min(1.0, f) * (h - 24)) - 4

    def _draw_header(self, canvas, status) -> None:
        cv2.putText(canvas, "E1 HUM SENTINEL", (M + 8, M + 26), FONT, 0.85, WHITE, 2)
        cv2.putText(canvas, "acoustic anomaly detection on the Electron E1",
                    (280, M + 25), FONT, 0.5, GRAY, 1)
        if status is None:
            return
        if status.learning:
            badge, color = f"LEARNING BASELINE {status.learn_pct}%", YELLOW
        elif status.event:
            badge = f"ANOMALY: {e1proto.CLASS_NAMES.get(status.ev_class, '?')}"
            color = RED if (time.monotonic() * 2) % 1 < 0.6 else WHITE
        else:
            badge, color = "WATCHING", GREEN
        (tw, _), _ = cv2.getTextSize(badge, FONT, 0.7, 2)
        cv2.putText(canvas, badge, (CANVAS_W - M - tw - 8, M + 26), FONT, 0.7, color, 2)

    def _draw_spectrum(self, canvas, spectrum) -> None:
        x0, y0, w, h = M, M + HDR_H + M, LEFT_W, SPEC_H
        self._panel(canvas, x0, y0, w, h, "LIVE SPECTRUM  0-4 kHz   (green: now, "
                                          "gray: baseline, yellow: trigger)")
        for khz in (1, 2, 3):
            gx = x0 + int(w * khz * 1000 / 4000)
            cv2.line(canvas, (gx, y0 + 26), (gx, y0 + h - 16), DARKGRAY, 1)
            cv2.putText(canvas, f"{khz}k", (gx - 10, y0 + h - 4), FONT, 0.4, GRAY, 1)
        if spectrum is None:
            return
        spec_raw = np.frombuffer(spectrum.spec, np.uint8)
        trig_raw = np.frombuffer(spectrum.trig, np.uint8)
        spec = disp(spec_raw)
        base = disp(np.frombuffer(spectrum.base, np.uint8))
        trig = disp(trig_raw)
        bw = w // e1proto.VIZ_BINS  # 5 px per bin
        top, bot = y0 + 28, y0 + h - 18
        span = bot - top

        def ly(v):
            return bot - int(int(v) * span / 255)

        for j in range(e1proto.VIZ_BINS):
            bx = x0 + j * bw
            color = GREEN if spec_raw[j] <= trig_raw[j] else RED
            cv2.rectangle(canvas, (bx + 1, ly(spec[j])), (bx + bw - 1, bot),
                          color, -1)
        for arr, color in ((base, GRAY), (trig, YELLOW)):
            pts = np.array([[x0 + j * bw + bw // 2, ly(arr[j])]
                            for j in range(e1proto.VIZ_BINS)], np.int32)
            cv2.polylines(canvas, [pts], False, color, 1, cv2.LINE_AA)

    def _draw_waterfall(self, canvas) -> None:
        x0, y0, w, h = M, M + HDR_H + M + SPEC_H + M, LEFT_W, FALL_H
        img = cv2.applyColorMap(
            cv2.resize(disp(self.fall).astype(np.uint8), (w - 2, h - 2),
                       interpolation=cv2.INTER_NEAREST),
            cv2.COLORMAP_INFERNO)
        canvas[y0 + 1:y0 + h - 1, x0 + 1:x0 + w - 1] = img
        cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), DARKGRAY, 1)
        cv2.putText(canvas, "SPECTROGRAM (last ~%.0f s)" %
                    (FALL_H * CHUNK_SECS), (x0 + 8, y0 + 20), FONT, 0.5, WHITE, 1)

    def _draw_deploy_placard(self, canvas, status) -> None:
        x0, y0 = M, M + HDR_H + M
        w, h = LEFT_W, SPEC_H + M + FALL_H
        cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), (45, 30, 20), -1)
        ev = status is not None and status.event
        lines = (
            ("DEPLOY MODE", y0 + h // 2 - 80, 1.5, YELLOW, 3),
            ("audio is analyzed on-chip", y0 + h // 2 - 20, 0.8, WHITE, 2),
            ("only score + events leave the device", y0 + h // 2 + 15, 0.65, GRAY, 1),
            (f"events so far: {status.events if status else 0}",
             y0 + h // 2 + 70, 0.9, RED if ev else GREEN, 2),
        )
        for text, y, scale, color, thick in lines:
            (tw, _), _ = cv2.getTextSize(text, FONT, scale, thick)
            cv2.putText(canvas, text, (x0 + (w - tw) // 2, y), FONT, scale, color, thick)

    def _draw_score_chart(self, canvas) -> None:
        x0, y0, w, h = RIGHT_X, M + HDR_H + M, RIGHT_W, CHART_H
        self._panel(canvas, x0, y0, w, h, "ANOMALY SCORE (log scale)")
        ty = self._score_y(self.threshold, y0, h)
        cv2.line(canvas, (x0 + 4, ty), (x0 + w - 4, ty), YELLOW, 1)
        cv2.putText(canvas, f"thr {self.threshold}", (x0 + w - 74, ty - 4),
                    FONT, 0.45, YELLOW, 1)
        hist = list(self.history)[-(w - 12):]
        pts = []
        for i, st in enumerate(hist):
            x = x0 + 6 + i
            y = self._score_y(st.score, y0, h)
            pts.append((x, y))
            if st.event:
                cv2.line(canvas, (x, y0 + h - 6), (x, y0 + h - 2), RED, 1)
            if st.learning:
                cv2.line(canvas, (x, y0 + 26), (x, y0 + 29), YELLOW, 1)
        if len(pts) > 1:
            cv2.polylines(canvas, [np.array(pts, np.int32)], False, GREEN, 1,
                          cv2.LINE_AA)

    def _draw_event_log(self, canvas) -> None:
        x0, y0, w, h = RIGHT_X, M + HDR_H + M + CHART_H + M, RIGHT_W, EVLOG_H
        self._panel(canvas, x0, y0, w, h, "EVENT LOG")
        if not self.events:
            cv2.putText(canvas, "(no anomalies yet)", (x0 + 12, y0 + 48),
                        FONT, 0.5, GRAY, 1)
        for i, ev in enumerate(self.events):
            if i >= 7:
                break
            y = y0 + 44 + 20 * i
            name = e1proto.CLASS_NAMES.get(ev["cls"], "?")
            live = " <-" if (i == 0 and self._ev_open) else ""
            text = (f"{ev['t']}  {name:<18s} peak {ev['peak']:<5d} "
                    f"{fmt_hz(ev['hz'])}  {ev['chunks'] * CHUNK_SECS:.1f}s{live}")
            cv2.putText(canvas, text, (x0 + 12, y), FONT, 0.45,
                        RED if i == 0 and self._ev_open else WHITE, 1)

    def _draw_status(self, canvas, status, mode) -> None:
        x0 = RIGHT_X
        y0 = M + HDR_H + M + CHART_H + M + EVLOG_H + M
        h = BOTTOM_Y - y0
        self._panel(canvas, x0, y0, RIGHT_W, h, "E1 / LINK")
        score = status.score if status else 0
        events = status.events if status else 0
        coverage = min(100.0, self.chunk_rate * CHUNK_SECS * 100.0)
        lines = [
            f"mode {'DEV' if mode == e1proto.MODE_DEV else 'DEPLOY'}   "
            f"score {score}   events {events}",
            f"E1 compute {self.chunk_us_avg / 1000:.1f} ms/chunk   "
            f"link {self.chunk_rate:.1f} chunks/s ({coverage:.0f}% of realtime)",
            f"source {self.source.name}   level {self.level_db:.0f} dBFS   "
            f"retries {self.retries}   crc errs {self.rx.parser.crc_errors}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(canvas, text, (x0 + 12, y0 + 44 + 22 * i), FONT, 0.5, WHITE, 1)

    def _draw_bottom(self, canvas) -> None:
        y0 = BOTTOM_Y
        h = CANVAS_H - y0 - M
        self._panel(canvas, M, y0, CANVAS_W - 2 * M, h)
        # power: mW comes measured from the EVK's current sensors;
        # VDDIO (= CHIP_RAIL) is the E1x chip alone, SYS the whole board
        ma, mw = self.power.latest_ma(), self.power.latest_mw()
        tag = " (MOCK)" if self.power.is_mock else ""
        cv2.putText(canvas, f"POWER{tag}", (M + 12, y0 + 24), FONT, 0.55, YELLOW, 2)
        if self.burn:
            btxt = "[BURN - constant workload, streaming paused]"
            (tw, _), _ = cv2.getTextSize(btxt, FONT, 0.5, 2)
            cv2.putText(canvas, btxt, (CANVAS_W - M - tw - 46, y0 + 24),
                        FONT, 0.5, RED, 2)
        if mw:
            avg = self.power.avg_mw()
            chip = avg.get(CHIP_RAIL, mw.get(CHIP_RAIL, 0.0))
            rail_txt = "  ".join(f"{r} {mw[r]:.1f}" for r in RAILS)
            runtime = fmt_runtime(battery_hours(ma.get(CHIP_RAIL, 0.0)))
            cv2.putText(canvas, f"{rail_txt}  mW", (M + 175, y0 + 24), FONT, 0.5, WHITE, 1)
            cv2.putText(canvas,
                        f"E1x chip {chip:.1f} mW (10s avg)   "
                        f"on 1x AA (2500 mAh): {runtime}",
                        (M + 12, y0 + 52), FONT, 0.6, GREEN, 2)
            w = self.workload_mw if self.workload_mw is not None else chip
            avg_w = duty_avg_mw(w, self.workload_runtime,
                                self.workload_period, self.sleep_mw)
            hours = duty_battery_hours(w, self.workload_runtime,
                                       self.workload_period, self.sleep_mw)
            cv2.putText(canvas,
                        f"workload {fmt_mw(w)} x {self.workload_runtime:g}s "
                        f"every {self.workload_period:g}h -> "
                        f"avg {fmt_mw(avg_w)} -> 1x AA {fmt_runtime(hours)}",
                        (M + 12, y0 + 80), FONT, 0.5, WHITE, 1)
        else:
            cv2.putText(canvas, "waiting for data...", (M + 130, y0 + 24),
                        FONT, 0.5, GRAY, 1)
        cv2.putText(canvas,
                    "d: DEV/DEPLOY   r: relearn   t/T: threshold -/+   "
                    "1/2/3: inject thump/voice/jingle   b: burn   q: quit",
                    (M + 12, y0 + h - 12), FONT, 0.5, GRAY, 1)
        if self.writer is not None:
            cv2.circle(canvas, (CANVAS_W - 30, y0 + 24), 9, (60, 60, 230), -1)

    def render(self) -> np.ndarray:
        with self.rx.lock:
            status, spectrum = self.rx.status, self.rx.spectrum
        mode = status.mode if status else e1proto.MODE_DEV

        canvas = np.full((CANVAS_H, CANVAS_W, 3), BG, np.uint8)
        self._draw_header(canvas, status)
        if mode == e1proto.MODE_DEV:
            self._draw_spectrum(canvas, spectrum)
            self._draw_waterfall(canvas)
        else:
            self._draw_deploy_placard(canvas, status)
        self._draw_score_chart(canvas)
        self._draw_event_log(canvas)
        self._draw_status(canvas, status, mode)
        self._draw_bottom(canvas)
        return canvas

    # --- main loop ----------------------------------------------------

    def run(self) -> None:
        self.handshake()
        if self._relearn_on_connect:
            # the firmware may hold a baseline learned from an earlier
            # session's audio; start from the room as it sounds now
            self.send_param(e1proto.PARAM_RELEARN, 0)
        print("connected — streaming. Focus the window; q quits.")
        while not self._quit:
            if self.tx_error:
                sys.exit(f"link error: {self.tx_error}")
            # independent ifs, not elif: one render pass may both clear
            # an ACK and start the next message — at ~186 ms link RTT an
            # extra pass of latency per chunk costs real throughput
            now = time.monotonic()
            if self._inflight is not None:
                self.poll_ack(now)
            if self._inflight is None and self._pending_params:
                pid, value = self._pending_params.pop(0)
                self.start_param(pid, value, now)
            if self._inflight is None and not self._pending_params \
                    and not self.burn and now >= self._next_send:
                self.start_chunk(now)

            self.drain_rx()
            canvas = self.render()
            if self.writer is not None:
                self.writer.write(canvas)
            cv2.imshow("E1 Hum Sentinel", canvas)

            key = cv2.waitKey(20) & 0xFF
            with self.rx.lock:
                mode = self.rx.status.mode if self.rx.status else e1proto.MODE_DEV
            if key != 0xFF and not self.handle_key(key, mode):
                break
            # window-close detection: VISIBLE reads <1 until the WM maps
            # the window, so only trust it after it has been seen open
            visible = cv2.getWindowProperty("E1 Hum Sentinel",
                                            cv2.WND_PROP_VISIBLE) >= 1
            if visible:
                self._window_seen = True
            elif self._window_seen:
                break  # was open, now closed

        self.source.close()
        if self.writer is not None:
            self.writer.release()
            print(f"recording saved to {self.record_path}")
        with self.rx.lock:
            st = self.rx.status
        print(f"session: {self.chunks_sent} chunks at {self.chunk_rate:.1f}/s, "
              f"events={st.events if st else '?'}, "
              f"e1 {self.chunk_us_avg / 1000:.1f} ms/chunk, "
              f"spectra {self.rx.n_spectra}, retries {self.retries}, "
              f"crc errs {self.rx.parser.crc_errors}")
        cv2.destroyAllWindows()


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", metavar="HOST:PORT", help="connect to the firmware simulator")
    g.add_argument("--port", metavar="DEV", help="serial device of the EVK chunk link")
    ap.add_argument("--baud", type=int, default=115200,
                    help="link baud rate (115200 is the only rate the "
                         "EVK bridge supports — see README)")
    ap.add_argument("--source", default="mic",
                    help="audio source: mic, synth, or wav:PATH "
                         "(mic falls back to synth if capture fails)")
    ap.add_argument("--power-port", metavar="DEV",
                    help="serial device streaming the EVK power CSV (mock if omitted)")
    ap.add_argument("--power-baud", type=int, default=115200)
    ap.add_argument("--workload-mw", type=float, default=None,
                    help="workload power for the AA projection "
                         "(default: the live 10 s chip average — with burn "
                         "on, the measured constant-workload draw)")
    ap.add_argument("--workload-runtime", type=float, default=2.0,
                    help="seconds the workload runs per wake-up")
    ap.add_argument("--workload-period", type=float, default=1.0,
                    help="hours between wake-ups")
    ap.add_argument("--sleep-mw", type=float, default=0.0,
                    help="sleep power between wake-ups")
    ap.add_argument("--record", metavar="OUT.MP4", help="record the session video")
    ap.add_argument("--no-relearn", action="store_true",
                    help="keep the baseline the firmware already has "
                         "instead of relearning on connect")
    args = ap.parse_args()

    try:
        App(args).run()
    except LinkError as e:
        sys.exit(f"link error: {e}")
    except ConnectionRefusedError:
        sys.exit("connection refused — start the firmware first "
                 "(make sim && ./firmware/build/firmware_sim)")


if __name__ == "__main__":
    main()
