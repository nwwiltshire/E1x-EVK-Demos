#!/usr/bin/env python3
"""Webcam sender + live viewer for the E1 analog meter reader.

Point the webcam at an analog gauge, drag the circle over the dial,
and the E1 reads it: the host crops/downsamples the ROI to a 64x64
grayscale frame and ships it down the serial link; ALL image analysis
(blur, normalization, 240-angle ray-cast) runs on the E1's dataflow
fabric.  What comes back is the reading — and, in DEV mode, the raw
per-angle evidence array, drawn as a polar plot so the audience can
watch the chip think.

Usage:
  python3 meter_viewer.py --sim 127.0.0.1:5555 --source synth   # no hardware
  python3 meter_viewer.py --port /dev/ttyACM2                   # EVK + webcam
  ... [--power-port /dev/ttyACM1] [--camera 2] [--units PSI]
      [--cal-min 0 --cal-max 100]

Mouse (left panel): drag = move the gauge circle, wheel = resize.
Keys:
  d  DEV/DEPLOY          p  needle polarity      s  smoothing 0/1/2/3
  [  needle is at scale MIN (--cal-min)          r  reset smoothing
  ]  needle is at scale MAX (--cal-max)          q  quit
  +/- resize the circle
"""

from __future__ import annotations

import argparse
import math
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
from power_monitor import (CHIP_RAIL, RAILS, MockPowerMonitor,
                           SerialPowerMonitor, battery_hours, fmt_runtime)

MARGIN = 10
CAM_W, CAM_H = 640, 480
SEE_S = 256                       # "what the E1 sees" panel (64 x4)
POLAR_S = 384                     # polar thinking panel
INFO_H = 240
CANVAS_W = MARGIN * 4 + CAM_W + SEE_S + POLAR_S
CANVAS_H = MARGIN * 3 + max(CAM_H, POLAR_S) + INFO_H

GREEN = (80, 220, 80)
WHITE = (235, 235, 235)
GRAY = (150, 150, 150)
YELLOW = (60, 200, 240)
ORANGE = (60, 140, 255)
RED = (70, 70, 230)
BG = (24, 24, 24)
PANEL_BG = (36, 36, 36)
FONT = cv2.FONT_HERSHEY_SIMPLEX

WIN = "E1 Meter Reader"


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
    sends, queues ACKs for the sender, keeps the latest STATUS/SCORES."""

    def __init__(self, link) -> None:
        super().__init__(daemon=True)
        self.link = link
        self.parser = e1proto.Parser()
        self.acks: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.status = None                  # last STATUS Msg
        self.n_status = 0
        self.scores: np.ndarray | None = None  # last SCORES as u16 array
        self.n_scores = 0
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
                        self.n_status += 1
                elif m.type == e1proto.SCORES:
                    arr = np.frombuffer(m.payload, np.uint16).copy()
                    with self.lock:
                        self.scores = arr
                        self.n_scores += 1


class TxWriter(threading.Thread):
    """All link writes happen on this thread: a serial write can block
    for seconds while the half-duplex bridge holds the line, and any
    multi-second stall on the GUI thread makes the window manager pop
    the 'not responding' dialog."""

    def __init__(self, link) -> None:
        super().__init__(daemon=True)
        self.link = link
        self.q: queue.Queue = queue.Queue()
        self.error: str | None = None

    def run(self) -> None:
        while True:
            data = self.q.get()
            try:
                self.link.write(data)
            except LinkError as e:
                self.error = str(e)
                return

    def send(self, data: bytes) -> None:
        self.q.put(data)


def peak_cdeg_from_scores(scores: np.ndarray) -> int:
    """Host-side mirror of the firmware peak picker (for calibration
    capture: the raw, unsmoothed needle angle)."""
    k = int(np.argmax(scores))
    s0 = int(scores[(k - 1) % e1proto.N_ANGLES])
    s1 = int(scores[k])
    s2 = int(scores[(k + 1) % e1proto.N_ANGLES])
    denom = s0 - 2 * s1 + s2
    frac_q8 = 0
    if denom != 0:
        frac_q8 = max(-128, min(128, (128 * (s0 - s2)) // denom))
    cdeg = -18000 + k * e1proto.STEP_CDEG + ((e1proto.STEP_CDEG * frac_q8) >> 8)
    return ((cdeg + 18000) % 36000) - 18000


class App:
    def __init__(self, args) -> None:
        if args.sim:
            host, _, port = args.sim.partition(":")
            self.link = TcpLink(host, int(port or 5555))
        else:
            self.link = SerialLink(args.port, args.baud)
        self.rx = Receiver(self.link)
        self.rx.start()
        self.tx = TxWriter(self.link)
        self.tx.start()

        if args.power_port:
            self.power = SerialPowerMonitor(args.power_port, args.power_baud)
        else:
            self.power = MockPowerMonitor()
        self.power.start()

        if args.source == "synth":
            from gauge_sim import SynthGauge
            self.cap = SynthGauge()
        else:
            self.cap = cv2.VideoCapture(args.camera)
            if not self.cap.isOpened():
                sys.exit(f"cannot open camera {args.camera} "
                         "(try --camera N; check ls /dev/video*)")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

        self.units = args.units
        self.cal_min = args.cal_min
        self.cal_max = args.cal_max

        # ROI circle in camera coordinates
        self.roi_x, self.roi_y = CAM_W // 2, CAM_H // 2
        self.roi_r = int(min(CAM_W, CAM_H) * 0.44)
        self._drag = False

        self.seq = 0
        self.fps = 0.0
        self._last_t = None
        self.retries = 0
        self.smooth = 2
        self.polarity = 0
        # one frame in flight, polled without blocking so the preview
        # stays live: [wire bytes, sent_at, attempt]
        self._inflight = None
        self._ack_timeout = 6.0
        # param changes wait until nothing is in flight: the EVK link
        # is half-duplex, so never transmit while a reply is inbound
        self._pending_params: list = []

        self.snapshot = args.snapshot
        self._next_snap = 0.0
        self.history: list[float] = []   # reading over time (strip chart)
        self._seen_status = 0
        self.flash_msg = ""
        self.flash_until = 0.0

        self._window_seen = False
        self._quit = False
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, "_quit", True))
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "_quit", True))

    # --- protocol helpers -------------------------------------------

    def _await_ack(self, timeout: float = 2.0):
        try:
            ack = self.rx.acks.get(timeout=timeout)
        except queue.Empty:
            return None
        if ack is None:
            sys.exit(f"link error: {self.rx.error}")
        return ack

    def handshake(self) -> None:
        self.tx.send(e1proto.build_get_status())
        for _ in range(40):
            with self.rx.lock:
                if self.rx.status is not None:
                    return
            time.sleep(0.05)
        sys.exit("no STATUS from firmware — is it running/flashed?")

    def start_frame(self, payload: bytes, now: float) -> None:
        self.seq = (self.seq + 1) & 0xFF
        wire = e1proto.build_frame(self.seq, payload)
        self._inflight = [wire, now, 1]
        self.tx.send(wire)

    def poll_frame_ack(self, now: float) -> None:
        try:
            ack = self.rx.acks.get_nowait()
        except queue.Empty:
            if now - self._inflight[1] > self._ack_timeout:
                self._retry_or_drop(now)
            return
        if ack is None:
            sys.exit(f"link error: {self.rx.error}")
        seq, ok = ack
        if ok and seq == self.seq:
            self._inflight = None
            done = time.monotonic()
            if self._last_t is not None:
                inst = 1.0 / max(done - self._last_t, 1e-6)
                self.fps = inst if self.fps == 0 else 0.8 * self.fps + 0.2 * inst
            self._last_t = done
        elif not ok:
            self._retry_or_drop(now)
        # else: stale ACK from an earlier attempt — keep waiting

    def _retry_or_drop(self, now: float) -> None:
        wire, _, attempt = self._inflight
        if attempt >= 3:
            self._inflight = None  # frame dropped; keep the demo alive
        else:
            self.retries += 1
            self._inflight = [wire, now, attempt + 1]
            self.tx.send(wire)

    def send_param(self, pid: int, value: int) -> None:
        """Queued; flushed by the main loop when the line is quiet."""
        self._pending_params.append((pid, value))

    def _flush_params(self) -> None:
        while self._pending_params:
            pid, value = self._pending_params.pop(0)
            self.tx.send(e1proto.build_set_param(pid, value))
            ack = self._await_ack()
            if not (ack and ack[1]):
                print(f"warning: SET_PARAM {pid}={value} not acked",
                      file=sys.stderr)

    # --- interaction --------------------------------------------------

    def flash(self, msg: str) -> None:
        self.flash_msg = msg
        self.flash_until = time.monotonic() + 2.5

    def on_mouse(self, event, x, y, flags, _param) -> None:
        # map canvas -> camera panel coordinates
        cx, cy = x - MARGIN, y - MARGIN
        inside = 0 <= cx < CAM_W and 0 <= cy < CAM_H
        if event == cv2.EVENT_LBUTTONDOWN and inside:
            self._drag = True
        if event == cv2.EVENT_LBUTTONUP:
            self._drag = False
        if event == cv2.EVENT_MOUSEMOVE and self._drag and inside:
            self.roi_x, self.roi_y = cx, cy
        if event == cv2.EVENT_MOUSEWHEEL and inside:
            self.roi_r += 10 if flags > 0 else -10
            self.roi_r = max(40, min(self.roi_r, min(CAM_W, CAM_H)))

    def _raw_needle_cdeg(self):
        with self.rx.lock:
            scores = self.rx.scores
        if scores is None or scores.max() == 0:
            return None
        return peak_cdeg_from_scores(scores)

    def handle_key(self, key: int, mode: int) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("d"):
            self.send_param(
                e1proto.PARAM_MODE,
                e1proto.MODE_DEPLOY if mode == e1proto.MODE_DEV else e1proto.MODE_DEV,
            )
        elif key == ord("p"):
            self.polarity ^= 1
            self.send_param(e1proto.PARAM_POLARITY, self.polarity)
            self.flash(f"polarity: {'light' if self.polarity else 'dark'} needle")
        elif key == ord("s"):
            self.smooth = (self.smooth + 1) % 4
            self.send_param(e1proto.PARAM_SMOOTH_SHIFT, self.smooth)
            self.flash(f"smoothing: {self.smooth}")
        elif key == ord("r"):
            self.send_param(e1proto.PARAM_RESET, 0)
            self.flash("smoothing state reset")
        elif key == ord("["):
            raw = self._raw_needle_cdeg()
            if raw is None:
                self.flash("no needle evidence yet")
            else:
                self.send_param(e1proto.PARAM_CAL_ANGLE_MIN, raw)
                self.send_param(e1proto.PARAM_CAL_VALUE_MIN,
                                int(round(self.cal_min * 1000)))
                self.flash(f"calibrated MIN: {raw / 100.0:+.1f} deg = {self.cal_min:g}")
        elif key == ord("]"):
            raw = self._raw_needle_cdeg()
            if raw is None:
                self.flash("no needle evidence yet")
            else:
                self.send_param(e1proto.PARAM_CAL_ANGLE_MAX, raw)
                self.send_param(e1proto.PARAM_CAL_VALUE_MAX,
                                int(round(self.cal_max * 1000)))
                self.flash(f"calibrated MAX: {raw / 100.0:+.1f} deg = {self.cal_max:g}")
        elif key in (ord("+"), ord("=")):
            self.roi_r = min(self.roi_r + 10, min(CAM_W, CAM_H))
        elif key == ord("-"):
            self.roi_r = max(40, self.roi_r - 10)
        return True

    # --- frame prep ---------------------------------------------------

    def crop_roi(self, frame: np.ndarray) -> np.ndarray:
        """Square crop around the ROI circle -> 64x64 grayscale."""
        r = self.roi_r
        x0, x1 = self.roi_x - r, self.roi_x + r
        y0, y1 = self.roi_y - r, self.roi_y + r
        # clamp by shifting so the crop stays square
        x0 = max(0, min(x0, CAM_W - 2 * r))
        y0 = max(0, min(y0, CAM_H - 2 * r))
        if 2 * r > CAM_H:  # circle bigger than the frame: fit what we can
            y0, y1 = 0, CAM_H
            x0 = max(0, min(self.roi_x - CAM_H // 2, CAM_W - CAM_H))
            x1 = x0 + CAM_H
        else:
            x1, y1 = x0 + 2 * r, y0 + 2 * r
        gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        return cv2.resize(gray, (e1proto.FRAME_W, e1proto.FRAME_H),
                          interpolation=cv2.INTER_AREA)

    # --- rendering ---------------------------------------------------

    @staticmethod
    def _dir(cdeg: float) -> tuple[float, float]:
        """Unit direction for a gauge angle (0 = up, CW+), image coords."""
        a = math.radians(cdeg / 100.0)
        return math.sin(a), -math.cos(a)

    def render(self, frame: np.ndarray, small: np.ndarray) -> np.ndarray:
        with self.rx.lock:
            status = self.rx.status
            scores = self.rx.scores
            n_status = self.rx.n_status
        mode = status.mode if status else e1proto.MODE_DEV
        needle_live = bool(status and status.needle)

        if status and n_status != self._seen_status:
            self._seen_status = n_status
            self.history.append(status.value_milli / 1000.0)
            del self.history[:-160]

        canvas = np.full((CANVAS_H, CANVAS_W, 3), BG, np.uint8)

        # ---- left: camera + ROI + needle overlay
        cam = frame.copy()
        col = GREEN if needle_live else GRAY
        cv2.circle(cam, (self.roi_x, self.roi_y), self.roi_r, YELLOW, 2)
        cv2.circle(cam, (self.roi_x, self.roi_y), 3, YELLOW, -1)
        if status:
            dx, dy = self._dir(status.angle_cdeg)
            tip = (int(self.roi_x + 0.86 * self.roi_r * dx),
                   int(self.roi_y + 0.86 * self.roi_r * dy))
            cv2.line(cam, (self.roi_x, self.roi_y), tip, col, 3)
            cv2.circle(cam, tip, 6, col, 2)
        cv2.putText(cam, "WEBCAM - drag circle onto the dial", (12, 26),
                    FONT, 0.62, WHITE, 2)
        canvas[MARGIN:MARGIN + CAM_H, MARGIN:MARGIN + CAM_W] = cam

        # ---- middle: what the E1 sees
        x0 = MARGIN * 2 + CAM_W
        see = cv2.cvtColor(
            cv2.resize(small, (SEE_S, SEE_S), interpolation=cv2.INTER_NEAREST),
            cv2.COLOR_GRAY2BGR)
        cv2.putText(see, "E1 INPUT 64x64", (10, 22), FONT, 0.55, YELLOW, 2)
        canvas[MARGIN:MARGIN + SEE_S, x0:x0 + SEE_S] = see

        # under it: chip vitals
        vy = MARGIN + SEE_S + 30
        if status:
            live = "LIVE" if needle_live else "NO NEEDLE"
            vit = [
                (f"needle {status.angle_deg:+7.2f} deg", col),
                (f"conf {status.confidence}/255 {live}", col),
                (f"fabric {status.frame_us / 1000.0:6.2f} ms/frame", WHITE),
                (f"frames {status.frames}   mean {status.mean}", GRAY),
                (f"mode {'DEV' if mode == e1proto.MODE_DEV else 'DEPLOY'}",
                 YELLOW if mode == e1proto.MODE_DEV else ORANGE),
            ]
            for i, (text, c) in enumerate(vit):
                cv2.putText(canvas, text, (x0, vy + 28 * i), FONT, 0.62, c, 2)

        # ---- right: the thinking (polar evidence plot) or placard
        px0 = MARGIN * 3 + CAM_W + SEE_S
        panel = np.full((POLAR_S, POLAR_S, 3), PANEL_BG, np.uint8)
        if mode == e1proto.MODE_DEV:
            pc = POLAR_S // 2
            rmax = POLAR_S // 2 - 24
            cv2.circle(panel, (pc, pc), rmax, (70, 70, 70), 1)
            cv2.circle(panel, (pc, pc), rmax // 2, (55, 55, 55), 1)
            cv2.putText(panel, "E1 EVIDENCE / angle", (10, 22), FONT, 0.55,
                        YELLOW, 2)
            if scores is not None and scores.max() > 0:
                norm = scores.astype(np.float64) / float(scores.max())
                pts = []
                for ai in range(e1proto.N_ANGLES):
                    dx, dy = self._dir(e1proto.index_to_cdeg(ai))
                    rr = 8 + norm[ai] * (rmax - 8)
                    pts.append((int(pc + rr * dx), int(pc + rr * dy)))
                cv2.polylines(panel, [np.array(pts, np.int32)], True, GREEN, 2)
                # peak ray
                k = int(np.argmax(scores))
                dx, dy = self._dir(e1proto.index_to_cdeg(k))
                cv2.line(panel, (pc, pc),
                         (int(pc + rmax * dx), int(pc + rmax * dy)),
                         (120, 255, 120), 1)
                if status:
                    dx, dy = self._dir(status.angle_cdeg)
                    cv2.arrowedLine(panel, (pc, pc),
                                    (int(pc + (rmax - 14) * dx),
                                     int(pc + (rmax - 14) * dy)),
                                    WHITE, 2, tipLength=0.08)
        else:
            for text, y, sc, c, th in (
                ("DEPLOY MODE", POLAR_S // 2 - 30, 1.1, ORANGE, 3),
                ("no image data leaving the chip", POLAR_S // 2 + 15, 0.6, WHITE, 2),
                ("reading + status only", POLAR_S // 2 + 50, 0.55, GRAY, 1),
            ):
                (tw, _), _ = cv2.getTextSize(text, FONT, sc, th)
                cv2.putText(panel, text, ((POLAR_S - tw) // 2, y), FONT, sc, c, th)
        canvas[MARGIN:MARGIN + POLAR_S, px0:px0 + POLAR_S] = panel

        # ---- bottom strip: reading, chart, link, power
        iy = MARGIN * 2 + max(CAM_H, POLAR_S)

        # big reading
        if status:
            val = f"{status.value_milli / 1000.0:.1f}"
            cv2.putText(canvas, val, (MARGIN + 14, iy + 118),
                        FONT, 3.4, col, 8)
            cv2.putText(canvas, self.units or "READING", (MARGIN + 20, iy + 158),
                        FONT, 0.7, GRAY, 2)

        # strip chart of the reading
        chx, chw, chh = MARGIN + 330, 420, 130
        cv2.rectangle(canvas, (chx, iy + 16), (chx + chw, iy + 16 + chh),
                      PANEL_BG, -1)
        if len(self.history) >= 2:
            lo, hi = min(self.history), max(self.history)
            span = (hi - lo) or 1.0
            pts = [(chx + int(i * chw / max(len(self.history) - 1, 1)),
                    iy + 16 + chh - 6 - int((v - lo) / span * (chh - 12)))
                   for i, v in enumerate(self.history)]
            cv2.polylines(canvas, [np.array(pts, np.int32)], False, GREEN, 2)
            cv2.putText(canvas, f"{hi:g}", (chx + 4, iy + 32), FONT, 0.45, GRAY, 1)
            cv2.putText(canvas, f"{lo:g}", (chx + 4, iy + 12 + chh), FONT, 0.45,
                        GRAY, 1)
        cv2.putText(canvas, "reading history", (chx, iy + 16 + chh + 24),
                    FONT, 0.55, GRAY, 1)

        # link + keys
        lx = chx + chw + 30
        rate = f"tx {self.fps:4.2f} fps"
        lines = [
            rate + f"   retries {self.retries}   crc errs {self.rx.parser.crc_errors}",
            "d DEV/DEPLOY   p polarity   s smooth   r reset",
            f"[ needle at MIN ({self.cal_min:g})   ] needle at MAX ({self.cal_max:g})",
            "drag circle onto dial, wheel/+/- to resize   q quit",
        ]
        for i, text in enumerate(lines):
            cv2.putText(canvas, text, (lx, iy + 40 + 30 * i), FONT, 0.55,
                        WHITE if i == 0 else GRAY, 1)

        # power (measured by the EVK's own sensors; VDDIO = the chip),
        # in the free space under the polar panel
        pxp, pyy = px0, MARGIN + POLAR_S + 30
        ma, mw = self.power.latest_ma(), self.power.latest_mw()
        tag = " (MOCK)" if self.power.is_mock else ""
        cv2.putText(canvas, f"POWER{tag}", (pxp, pyy), FONT, 0.62, YELLOW, 2)
        if mw:
            rail_txt = "  ".join(f"{r} {mw[r]:.1f}" for r in RAILS)
            cv2.putText(canvas, f"{rail_txt}  mW", (pxp, pyy + 28), FONT, 0.45,
                        WHITE, 1)
            runtime = fmt_runtime(battery_hours(ma.get(CHIP_RAIL, 0.0)))
            cv2.putText(canvas,
                        f"E1x chip {mw.get(CHIP_RAIL, 0.0):.1f} mW"
                        f"   on 1x AA: {runtime}",
                        (pxp, pyy + 58), FONT, 0.55, GREEN, 2)
        else:
            cv2.putText(canvas, "waiting for data...", (pxp, pyy + 28), FONT,
                        0.5, GRAY, 1)

        # transient message
        if self.flash_msg and time.monotonic() < self.flash_until:
            (tw, _), _ = cv2.getTextSize(self.flash_msg, FONT, 0.8, 2)
            cv2.putText(canvas, self.flash_msg,
                        ((CANVAS_W - tw) // 2, MARGIN + CAM_H - 16),
                        FONT, 0.8, YELLOW, 2)

        return canvas

    # --- main loop ----------------------------------------------------

    def run(self) -> None:
        self.handshake()
        # start from a known state (device may be warm)
        self.send_param(e1proto.PARAM_SMOOTH_SHIFT, self.smooth)
        self.send_param(e1proto.PARAM_POLARITY, self.polarity)
        self.send_param(e1proto.PARAM_RESET, 0)
        print("connected — streaming. Drag the circle onto the dial; q quits.")

        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, self.on_mouse)

        while not self._quit:
            ok, frame = self.cap.read()
            if not ok:
                print("camera read failed", file=sys.stderr)
                break
            if frame.shape[:2] != (CAM_H, CAM_W):
                frame = cv2.resize(frame, (CAM_W, CAM_H))
            small = self.crop_roi(frame)

            now = time.monotonic()
            if self._inflight is not None:
                self.poll_frame_ack(now)
            elif self._pending_params:
                self._flush_params()
            else:
                self.start_frame(small.tobytes(), now)

            canvas = self.render(frame, small)
            if self.snapshot and now >= self._next_snap:
                self._next_snap = now + 1.0
                cv2.imwrite(self.snapshot, canvas)
            cv2.imshow(WIN, canvas)

            key = cv2.waitKey(20) & 0xFF
            with self.rx.lock:
                mode = self.rx.status.mode if self.rx.status else e1proto.MODE_DEV
            if key != 0xFF and not self.handle_key(key, mode):
                break
            visible = cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) >= 1
            if visible:
                self._window_seen = True
            elif self._window_seen:
                break  # was open, now closed

        self.cap.release()
        with self.rx.lock:
            st = self.rx.status
        if st:
            print(f"session: tx {self.fps:.2f} fps, last reading "
                  f"{st.value_milli / 1000.0:g} ({st.angle_deg:+.2f} deg), "
                  f"retries {self.retries}, crc errs {self.rx.parser.crc_errors}")
        cv2.destroyAllWindows()


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", metavar="HOST:PORT", help="connect to the firmware simulator")
    g.add_argument("--port", metavar="DEV", help="serial device of the EVK frame link")
    ap.add_argument("--baud", type=int, default=115200,
                    help="frame-link baud (115200 is the only rate the EVK "
                         "bridge supports)")
    ap.add_argument("--source", choices=("camera", "synth"), default="camera",
                    help="synth = built-in animated gauge, no webcam needed")
    ap.add_argument("--camera", type=int, default=0, help="webcam index")
    ap.add_argument("--units", default="", help="unit label for the reading")
    ap.add_argument("--cal-min", type=float, default=0.0,
                    help="scale value at the '[' calibration point")
    ap.add_argument("--cal-max", type=float, default=100.0,
                    help="scale value at the ']' calibration point")
    ap.add_argument("--power-port", metavar="DEV",
                    help="serial device streaming the EVK power CSV (mock if omitted)")
    ap.add_argument("--power-baud", type=int, default=115200)
    ap.add_argument("--snapshot", metavar="OUT.PNG",
                    help="save the canvas to this file once per second")
    args = ap.parse_args()

    try:
        App(args).run()
    except LinkError as e:
        sys.exit(f"link error: {e}")
    except ConnectionRefusedError:
        sys.exit("connection refused — start the firmware simulator first "
                 "(make sim && ./firmware/build/firmware_sim)")


if __name__ == "__main__":
    main()
