#!/usr/bin/env python3
"""A standalone animated gauge window — a demo target when no physical
gauge is handy.  Show it on a second monitor (or a phone pointed the
right way) and aim the webcam at it.

Usage: python3 test_gauge.py [--size 700] [--polarity 0]
Keys:  left/right or a/l = nudge the needle    space = auto-sweep
       q = quit
"""

from __future__ import annotations

import argparse
import math
import time

import cv2

from gauge_sim import render_gauge


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--size", type=int, default=700)
    ap.add_argument("--polarity", type=int, default=0, choices=(0, 1))
    args = ap.parse_args()

    angle = 0.0
    auto = True
    t0 = time.monotonic()
    while True:
        if auto:
            t = time.monotonic() - t0
            angle = 115.0 * math.sin(t * 0.3) + 8.0 * math.sin(t * 1.7)
        img = render_gauge(angle, size=args.size, polarity=args.polarity)
        cv2.imshow("Test Gauge", img)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            auto = not auto
        if key in (81, ord("a")):  # left
            auto, angle = False, max(-175.0, angle - 2.5)
        if key in (83, ord("l")):  # right
            auto, angle = False, min(175.0, angle + 2.5)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
