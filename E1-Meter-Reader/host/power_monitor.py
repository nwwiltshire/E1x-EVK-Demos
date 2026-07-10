#!/usr/bin/env python3
"""EVK power telemetry: parse the current-sensor CSV stream, or mock it.

The EVK streams its four current sensors (SYS, 1V8, VDDIO, VDDVAR)
over the second USB serial port (typically /dev/ttyACM1, or the
/dev/eff-power udev symlink).  Real format, verified on hardware —
one header line, then rows of

  timestamp(us), then mA, mV, mW for each of SYS, 1V8, VDDIO, VDDVAR,
  then AON4, AON5 (digital profiling outputs)

Power arrives measured in mW; no nominal-voltage assumptions needed.
Rail meanings (EVK Getting Started guide): SYS = whole board at the
power source (DC/DCs, LEDs, housekeeping MCU included), 1V8 = chip +
MRAM + MCU rail, VDDIO = the E1x chip alone, VDDVAR = the E1x
scalar core + fabric + peripherals only.

Standalone:  python3 power_monitor.py --mock
             python3 power_monitor.py --port /dev/ttyACM1
"""

from __future__ import annotations

import argparse
import math
import threading
import time

RAILS = ("SYS", "1V8", "VDDIO", "VDDVAR")

# column index of each rail's (mA, mW) pair in a CSV row
_COLS = {"SYS": (1, 3), "1V8": (4, 6), "VDDIO": (7, 9), "VDDVAR": (10, 12)}

AA_CAPACITY_MAH = 2500.0

# the rail that represents "the chip" for headline figures
CHIP_RAIL = "VDDIO"


def battery_hours(ma: float) -> float:
    """Projected runtime on 1x AA (2500 mAh) at the given current."""
    return AA_CAPACITY_MAH / max(ma, 1e-3)


def fmt_runtime(hours: float) -> str:
    if hours >= 2 * 24 * 365:
        return f"~{hours / (24 * 365):.1f} years"
    if hours >= 2 * 24:
        return f"~{hours / 24:.0f} days"
    return f"~{hours:.0f} h"


class _Monitor(threading.Thread):
    """Base: background thread keeping the latest per-rail readings."""

    is_mock = False

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._ma: dict[str, float] = {}
        self._mw: dict[str, float] = {}
        self._stop = threading.Event()

    def latest_ma(self) -> dict[str, float]:
        with self._lock:
            return dict(self._ma)

    def latest_mw(self) -> dict[str, float]:
        with self._lock:
            return dict(self._mw)

    def _publish(self, ma: dict[str, float], mw: dict[str, float]) -> None:
        with self._lock:
            self._ma, self._mw = ma, mw

    def stop(self) -> None:
        self._stop.set()


class MockPowerMonitor(_Monitor):
    """Numbers shaped like the real board at idle (measured 2026-07-09)
    with a gentle wander, so the demo UI works without hardware."""

    is_mock = True
    _BASE = {  # rail: (mA, mW)
        "SYS": (16.5, 83.6), "1V8": (2.3, 4.1),
        "VDDIO": (1.9, 3.4), "VDDVAR": (1.8, 1.0),
    }

    def run(self) -> None:
        t0 = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic() - t0
            wob = {
                r: 1.0 + 0.06 * math.sin(0.7 * t + i) + 0.02 * math.sin(3.1 * t * (i + 1))
                for i, r in enumerate(RAILS)
            }
            self._publish(
                {r: self._BASE[r][0] * wob[r] for r in RAILS},
                {r: self._BASE[r][1] * wob[r] for r in RAILS},
            )
            time.sleep(0.1)


class SerialPowerMonitor(_Monitor):
    def __init__(self, port: str, baud: int = 115200) -> None:
        super().__init__()
        import serial  # deferred so mock mode works without pyserial

        self._ser = serial.Serial(port, baud, timeout=0.5)

    def run(self) -> None:
        # The CSV streams ~200 rows/s but the UI needs ~5: parsing every
        # row in Python taxes the GIL enough to slow the chunk link's
        # receiver thread.  Poll 5x/s and parse only the newest complete
        # line; everything older is thrown away unread.
        buf = bytearray()
        while not self._stop.is_set():
            time.sleep(0.2)
            try:
                buf += self._ser.read(max(self._ser.in_waiting, 1))
            except Exception:
                break
            if len(buf) > 4096:
                del buf[:-4096]
            lines = buf.split(b"\n")
            if len(lines) < 2:
                continue
            buf = bytearray(lines[-1])  # partial tail stays buffered
            for raw in reversed(lines[:-1]):
                tokens = raw.decode("ascii", "replace").strip().split(",")
                try:
                    vals = [float(t) for t in tokens]
                except ValueError:
                    continue  # the header line
                if len(vals) >= 13:
                    self._publish(
                        {r: vals[c[0]] for r, c in _COLS.items()},
                        {r: vals[c[1]] for r, c in _COLS.items()},
                    )
                    break


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--port", help="serial port carrying the EVK power CSV")
    g.add_argument("--mock", action="store_true", help="synthesize readings")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    mon = MockPowerMonitor() if args.mock else SerialPowerMonitor(args.port, args.baud)
    mon.start()
    try:
        while True:
            time.sleep(0.5)
            ma, mw = mon.latest_ma(), mon.latest_mw()
            if not mw:
                print("waiting for data...")
                continue
            rails = "  ".join(f"{r} {mw[r]:6.2f}mW" for r in RAILS)
            runtime = fmt_runtime(battery_hours(ma.get(CHIP_RAIL, 0.0)))
            tag = " (MOCK)" if mon.is_mock else ""
            print(f"{rails} | chip {mw.get(CHIP_RAIL, 0.0):.2f}mW"
                  f" | E1x on 1x AA {runtime}{tag}")
    except KeyboardInterrupt:
        pass
    finally:
        mon.stop()


if __name__ == "__main__":
    main()
