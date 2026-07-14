# E1x EVK Demos

Two live demos for the [Efficient Computer](https://www.efficient.computer)
**Electron E1** EVK.  Both put the hot loops on the E1's dataflow
fabric, keep the branchy control logic on the RISC-V control core, and
show the audience the difference — compute time, privacy story, and
*measured* chip power — in a live GUI.

| Demo | What it does | Fabric compute | vs scalar | Chip power (VDDIO) |
|---|---|---|---|---|
| [**E1 Hum Sentinel**](E1-Hum-Sentinel/) | Learns a room's acoustic baseline, flags and classes anomalies (1024-pt FFT + spectral novelty on-chip) | 2.08 ms / 128 ms chunk (62x realtime) | 4.0x faster | ~2.6 mW |
| [**E1 Meter Reader**](E1-Meter-Reader/) | Reads any analog gauge from a webcam (blur + 240-angle ray-cast needle search on-chip) | 0.76 ms / frame | 9.7x faster | ~2.7 mW |

Each demo runs with **no hardware** (the firmware builds as a Linux
process; the serial link becomes a loopback TCP socket) — see each
README's quick start.  On hardware, the host talks to the EVK over the
USB bridge (fixed ~115200 baud, half-duplex) and reads the board's own
current sensors for the power panel.

## Shared architecture

Both demos are the same shape, by design:

- **Portable C99 firmware** — a superloop over a 6-function HAL:
  `hal_posix.c` (simulator) and `hal_e1.c` (EVK UART3 via the SDK).
- **Fabric kernels** — the DSP/image hot loops are annotated
  `__efficient__`; everything else stays on the control core.  Each
  STATUS reply carries the measured on-chip compute time, so a
  mis-annotated (silently all-scalar) build is obvious in the GUI.
- **One wire protocol family** — magic `AA 55`, CRC16-CCITT, one
  message in flight, ACK last.  Mirrored in Python (`host/e1proto.py`).
- **DEV / DEPLOY modes** — DEV streams the chip's intermediate
  evidence (spectrum / per-angle scores) so you can watch it think;
  DEPLOY sends readings only — no raw-data-derived payload leaves the
  chip.
- **Measured power** — `host/power_monitor.py` parses the EVK's power
  CSV (ttyACM1); VDDIO = the E1x chip alone, the headline number.

## Power measurement and burn mode

The headline mW figures above are **averages at each demo's duty
cycle**: the fabric finishes a chunk/frame in ~1–2 ms but the
fixed-115200 link delivers work only every ~0.2–0.5 s, so the rail
mostly shows idle power.  Efficient Computer's internal benchmarks run
*constant* workloads (fft4k ~4.8 mW, conv3x3 ~9 mW) — a different
kind of number.

**Burn mode** bridges the two: press `b` in either viewer (or send
`SET_PARAM burn=1`) and the firmware re-runs its fabric pipeline back
to back on the last chunk/frame — ~99% fabric duty, streaming paused,
zero link traffic — so the VDDIO rail reads the constant-workload
draw.  Detector/reading state is untouched; press `b` again to resume
the demo.  Each demo README has the full 3-step measurement procedure
(idle floor → duty-cycled average → constant workload).

Both the viewers and standalone `power_monitor.py` also project life
on one AA cell (3000 mWh) for a duty-cycled deployment — *workload of
W mW run for R s every P hours* — via `--workload-mw`
`--workload-runtime` `--workload-period` `--sleep-mw`.  The workload
power defaults to the live 10 s chip average, so with burn on the
measured constant-workload draw fills in automatically.

## Going deeper

- [E1-Hum-Sentinel/docs/HOW_IT_WORKS.md](E1-Hum-Sentinel/docs/HOW_IT_WORKS.md)
  and [E1-Meter-Reader/docs/HOW_IT_WORKS.md](E1-Meter-Reader/docs/HOW_IT_WORKS.md)
  — each demo's full pipeline and the DEV vs DEPLOY story.
- [DEVELOPING_ON_E1.md](E1-Hum-Sentinel/docs/DEVELOPING_ON_E1.md) —
  EVK field notes (link physics, fabric rules, board switches, port
  map): start here to build the next E1 application.

Tests: each demo has `make test` (C unit tests + end-to-end against
the real firmware binary over the sim's TCP serial port).
