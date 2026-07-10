"""Synthetic analog gauge renderer.

Shared by three consumers:
  - tests/e2e_check.py     renders frames at known angles and asserts
                           the firmware reads them back correctly
  - host/hw_smoke.py       same assertions over the real serial link
  - host/meter_viewer.py   --source synth: a hardware-free demo camera

Angle convention matches the firmware: 0 deg = 12 o'clock, clockwise
positive.
"""

from __future__ import annotations

import math

import cv2
import numpy as np


def render_gauge(angle_deg: float, size: int = 480, polarity: int = 0,
                 sweep: tuple[float, float] = (-135.0, 135.0),
                 label: str = "PSI") -> np.ndarray:
    """A believable gauge face (BGR) with the needle at angle_deg."""
    face = 235 if polarity == 0 else 40
    ink = 30 if polarity == 0 else 220
    img = np.full((size, size, 3), 200, np.uint8)
    c = (size // 2, size // 2)
    radius = int(size * 0.47)
    cv2.circle(img, c, radius, (face,) * 3, -1)
    cv2.circle(img, c, radius, (ink,) * 3, 3)

    a0, a1 = sweep
    n_major = 11
    for i in range(n_major * 3 - 2):
        a = math.radians(a0 + i * (a1 - a0) / (n_major * 3 - 3))
        dx, dy = math.sin(a), -math.cos(a)
        major = i % 3 == 0
        r0 = radius - (22 if major else 12)
        x0, y0 = int(c[0] + r0 * dx), int(c[1] + r0 * dy)
        x1, y1 = int(c[0] + (radius - 5) * dx), int(c[1] + (radius - 5) * dy)
        cv2.line(img, (x0, y0), (x1, y1), (ink,) * 3, 3 if major else 1)
        if major:
            xt = int(c[0] + (radius - 45) * dx)
            yt = int(c[1] + (radius - 45) * dy)
            t = str((i // 3) * 10)
            (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.putText(img, t, (xt - tw // 2, yt + th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (ink,) * 3, 2)
    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.putText(img, label, (c[0] - tw // 2, c[1] + radius // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (ink,) * 3, 2)

    a = math.radians(angle_deg)
    dx, dy = math.sin(a), -math.cos(a)
    tip = (int(c[0] + radius * 0.80 * dx), int(c[1] + radius * 0.80 * dy))
    tail = (int(c[0] - radius * 0.12 * dx), int(c[1] - radius * 0.12 * dy))
    ncol = ((25,) * 3) if polarity == 0 else ((245,) * 3)
    cv2.line(img, tail, tip, ncol, 6)
    cv2.circle(img, c, 10, ncol, -1)
    return img


def render_frame64(angle_deg: float, polarity: int = 0,
                   noise: float = 0.0, seed: int | None = None) -> bytes:
    """A 64x64 grayscale wire payload with the needle at angle_deg."""
    big = render_gauge(angle_deg, size=480, polarity=polarity)
    gray = cv2.cvtColor(big, cv2.COLOR_BGR2GRAY)
    if noise > 0:
        rng = np.random.default_rng(seed)
        gray = np.clip(gray.astype(np.float64) +
                       rng.normal(0, noise, gray.shape), 0, 255).astype(np.uint8)
    small = cv2.resize(gray, (64, 64), interpolation=cv2.INTER_AREA)
    return small.tobytes()


class SynthGauge:
    """A wandering-needle gauge that acts like a camera for the viewer."""

    def __init__(self, polarity: int = 0) -> None:
        self.polarity = polarity
        self.t = 0.0

    def read(self) -> tuple[bool, np.ndarray]:
        # slow sweep + a little wobble, like a real process variable
        self.t += 0.03
        ang = 115.0 * math.sin(self.t * 0.35) + 6.0 * math.sin(self.t * 2.1)
        frame = render_gauge(ang, size=480, polarity=self.polarity)
        # place the gauge off-centre on a bench-like background so the
        # ROI-alignment workflow is exercised even in synth mode
        canvas = np.full((480, 640, 3), 178, np.uint8)
        canvas[:, 80:560] = frame
        return True, canvas

    def release(self) -> None:
        pass
