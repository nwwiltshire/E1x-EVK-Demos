# E1x EVK Demos

Two self-contained demo applications for the [Efficient Computer](https://www.efficient.computer)
**Electron E1** (E1x silicon, EVK board).  Both follow the same shape:
a laptop streams sensor data down the EVK serial link, the E1 does the
real signal processing **on its dataflow fabric**, and the chip sends
back an answer plus — in DEV mode — the raw evidence behind it, so an
audience can watch the chip think.

Both demos exist to make one point concrete: the fabric is where the
speedup lives, and the power number is small enough to change what you
can build.

## The demos

### [E1-Meter-Reader](E1-Meter-Reader/) — computer vision

Point a webcam at any analog gauge and the E1 reads it.  The host crops
the dial to a 64x64 grayscale frame; the chip runs denoise,
normalization, and a 240-angle ray-cast needle search, and returns a
calibrated reading with a confidence figure.  DEV mode streams the
per-angle evidence array, drawn live as a polar plot.

| | |
|---|---|
| Fabric compute / frame | **0.76 ms** (scalar build: 7.39 ms — **9.7x**) |
| E1x chip power (VDDIO) | **2.7 mW** |
| Accuracy, hardware sweep | worst 0.75 deg (~0.3 % of a 270-deg scale) |

`./run_demo.sh` — no hardware needed.

### [E1-Hum-Sentinel](E1-Hum-Sentinel/) — acoustic anomaly detection

A privacy-first acoustic monitor.  The host streams 8 kHz u-law audio;
the chip learns the room's baseline hum, runs a 1024-point FFT +
spectral novelty detector, and flags anomalies within a few hundred ms —
classed LOW (thump), MID (voice), or HIGH (keys/clink).  In DEPLOY mode
the audio is analyzed on-chip and **no spectrum or audio-derived data
leaves the device**.

| | |
|---|---|
| Fabric compute / 128 ms chunk | **2.08 ms** (62x realtime; scalar: 8.26 ms — **4.0x**) |
| E1x chip power (VDDIO) | **~2.6 mW** → ~72 days on one AA |

`make sim` + `sentinel_viewer.py --sim` — no hardware needed.

## Shared architecture

Both projects are built the same way, deliberately — the second was
scaffolded from the first, and the pattern is the recommended starting
point for a third:

```
firmware/     portable C99 core + two HAL backends behind a 6-function seam
  main.c        superloop: rx -> process -> tx
  *.c/.h        control-core logic (branchy, sequential)
  dsp/kernels   __efficient__ fabric kernels (the hot loops)
  *_tables.c    generated const tables (see tools/)
  protocol.c/h  AA 55 framing, CRC16, parser, builders
  hal_posix.c   simulator: "serial" = loopback TCP  -> runs on any laptop
  hal_e1.c      EVK: UART3 through the USB bridge
  echo_main.c   gap-echo link bring-up app
host/         Python GUI, protocol mirror, power telemetry, smoke tests
tests/        C unit tests vs reference implementations + sim e2e
tools/        table generators
docs/         HOW_IT_WORKS.md + DEVELOPING_ON_E1.md
```

Every demo runs end-to-end **with no hardware**: `hal_posix.c` builds the
real firmware as a Linux process with a TCP loopback standing in for the
serial port.  The protocol, the fabric kernels (compiled by gcc via the
`__efficient__` guard macro), and all the DSP are the same code that
ships to the chip.

## Requirements

- **Simulator (no hardware):** Python 3.10+, gcc, make.  Each project has
  its own `host/requirements.txt`.
- **Hardware:** an Electron E1 EVK and the Efficient SDK at `~/effcc`
  (version 25.4 — what these numbers were measured against).

## Building the next one

Start from `docs/DEVELOPING_ON_E1.md` in either project, copy the
firmware skeleton (HAL seam, protocol, echo bring-up app, dual
fabric/scalar CMake build), and bring the link up with `echo_test.py`
before writing a line of application code.  The bring-up ladder —
link_echo → echo_test → hw_smoke → the demo — is what makes a bad day on
the EVK debuggable.
