# E1 Meter Reader

Point a webcam at any analog gauge and the **Electron E1** reads it.
The host crops the dial to a 64x64 grayscale frame and ships it down
the EVK serial link; **every pixel of image analysis — denoise,
normalization, and a 240-angle ray-cast needle search — runs on the
E1's dataflow fabric**.  What comes back is a calibrated reading, a
confidence figure, and (in DEV mode) the chip's raw per-angle evidence
array, drawn live as a polar plot so an audience can watch the chip
think.

Two modes make the product story:

- **DEV** — the E1 streams its evidence array with every reading: the
  GUI shows the dial the chip sees and the polar "thinking" plot.
- **DEPLOY** — readings only.  No image-derived data leaves the chip;
  the return traffic visibly collapses to a 28-byte status.

Docs: [How it works](docs/HOW_IT_WORKS.md) (full pipeline + DEV vs
DEPLOY in depth) · [Developing on the E1](docs/DEVELOPING_ON_E1.md)
(EVK field notes: link physics, fabric rules, board switches).

## Measured on the EVK (SDK 25.4, 2026-07-10)

| What | Number |
|---|---|
| Fabric compute per frame | **0.76 ms** |
| Same code, scalar (control-core) build | 7.39 ms (**9.7x slower**) |
| DEV round trip (frame down + scores/status/ack back) | 0.46 s (~2.2 fps) |
| E1x chip power while reading (VDDIO rail, measured) | **2.7 mW** |
| Reading accuracy, hardware sweep | worst 0.75 deg (~0.3 % of a 270-deg scale) |
| Link errors over the full test suite | 0 |

## Layout

```
firmware/            portable C99 core + two HAL backends
  main.c             superloop: rx frame -> gauge reader -> tx reading
  gauge.c/.h         control core: argmax, interpolation, EMA, calibration
  kernels.c/.h       __efficient__ fabric kernels: blur, sum, ray scores
  ray_tables.c/.h    generated gather table (tools/gen_ray_tables.py)
  protocol.c/.h      AA 55 framing, CRC16, parser, builders
  hal.h              6-function platform seam
  hal_posix.c        simulator backend: "serial" = loopback TCP
  hal_e1.c           EVK backend: UART3 through the USB bridge
  echo_main.c        gap-echo link bring-up app
host/
  meter_viewer.py    the GUI (webcam or synthetic source)
  test_gauge.py      animated gauge window to aim the webcam at
  gauge_sim.py       synthetic gauge renderer (GUI + all tests)
  e1proto.py         Python mirror of protocol.h
  power_monitor.py   ttyACM1 power-CSV telemetry (+ mock)
  hw_smoke.py        ~1 min hardware acceptance test
  echo_test.py       link bring-up check
tests/
  test_gauge.c       C unit tests vs double-precision references
  e2e_check.py       spawns the sim binary, drives the real protocol
tools/
  gen_ray_tables.py  regenerates the fabric gather table
docs/
  HOW_IT_WORKS.md    the full pipeline + DEV vs DEPLOY, in depth
  DEVELOPING_ON_E1.md  EVK field notes (start here for a new E1 app)
```

## Quick start (no hardware)

```sh
./run_demo.sh
```

That builds the firmware as a Linux process (the serial link is a
loopback TCP socket), starts it, and opens the viewer with a built-in
animated gauge.  Everything — protocol, fabric kernels (compiled with
gcc via the `__efficient__` guard macro), smoothing, calibration — is
the real firmware code.

Tests: `make test` (C unit tests + end-to-end against the sim binary).

## Going to hardware

Board: J16 USB, SW1 = USB power, SW2 = ON, SW9 ON (routes UART3 to the
USB bridge), SW11 position 1 (user button SW12), BOOT `101` (SRAM).

```sh
cd firmware && cmake -B bld && cmake --build bld   # needs ~/effcc
~/effcc/bin/eff-flash bld/fabric/meter_reader.hex sram
```

SRAM is volatile — reflash after every power cycle (or flash `mram` +
BOOT `010` to persist).  Bring-up ladder, first time or when in doubt:

```sh
~/effcc/bin/eff-flash firmware/bld/scalar/link_echo.hex sram
python3 host/echo_test.py --port /dev/ttyACM2        # link byte-exact?
~/effcc/bin/eff-flash firmware/bld/fabric/meter_reader.hex sram
python3 host/hw_smoke.py --port /dev/ttyACM2         # ~1 min acceptance
```

Then the demo:

```sh
python3 host/meter_viewer.py --port /dev/ttyACM2 --power-port /dev/ttyACM1 \
    --camera 2 --units PSI
```

## Demo runbook

1. Point the webcam at the gauge.  No gauge handy?  Run
   `python3 host/test_gauge.py` on a second screen and aim at that.
2. **Drag the yellow circle onto the dial face** (mouse), resize with
   the wheel or `+`/`-`.  The 64x64 panel shows exactly what the E1
   receives.
3. White needle on a black face?  Press `p` (polarity).
4. Calibrate in two keys: move the needle to the scale minimum (or
   wait for it), press `[`; at the scale maximum press `]`.  The
   values those points mean come from `--cal-min/--cal-max` (default
   0..100).  Uncalibrated, a standard 270-degree sweep reads 0..100.
5. `d` toggles DEV/DEPLOY (the EVK user button SW12 does too).  In
   DEPLOY the polar panel becomes the "nothing leaves the chip"
   placard — that's the pitch.
6. `s` cycles smoothing, `r` resets it, `q` quits.

The power panel is *measured* by the EVK's own current sensors
(`--power-port /dev/ttyACM1`): SYS is the whole dev board, VDDIO the
E1x chip alone — the number to quote.

## The fabric is where the speedup lives

effcc puts a function on the dataflow fabric only if its *definition*
is marked `__efficient__` — and forgetting the keyword produces **no
error**, just a silent 100%-control-core build.  This project defends
itself the standard way:

- every STATUS carries `frame_us` (on-chip compute time, measured
  around the kernel calls) — a mis-annotated build jumps from ~0.8 ms
  to ~7.4 ms and is obvious in the GUI;
- the CMake build produces both `fabric` and `scalar` images, so the
  ratio can be re-measured any time.

What runs where: `gk_blur3x3`, `gk_pixel_sum`, `gk_ray_scores`
(6.6 KB of gather-table lookups per frame) on the fabric; argmax,
parabolic interpolation, EMA smoothing, and calibration on the
control core (branchy, tiny — the wrong shape for spatial hardware).

## Link facts (measured, see docs/DEVELOPING_ON_E1.md)

The EVK USB bridge runs a fixed ~115200 baud (~10.8 KB/s each way)
and is only clean **half-duplex** — hence the protocol rules: one
message in flight, ACK sent last, everything framed with CRC16 +
resync, firmware stall reset at 500 ms.  A 64x64 frame is 4096 bytes
=> ~0.4 s down, which sets the ~2 fps ceiling; the demo interpolates
nothing and still feels live because the needle overlay and polar
plot update with every chip reply.

Ports: ttyACM0 = programmer (eff-flash), ttyACM1 = power CSV (starts
only when opened with DTR asserted; looks dead to `cat`),
ttyACM2 = the frame link.

## Protocol (see firmware/protocol.h for the byte layout)

```
FRAME      host->fw   4096-byte 64x64 grayscale, seq, CRC16
SET_PARAM  host->fw   param id + i32 (mode, polarity, smoothing,
                      conf floor, 4 calibration params, reset)
GET_STATUS host->fw
SCORES     fw->host   240 x u16 per-angle evidence   (DEV only)
STATUS     fw->host   angle (cdeg), value (x1000), confidence,
                      flags, mode, frame_us, frames, mean
ACK/NAK    fw->host   always last in every exchange
```

Angle convention everywhere: 0 = 12 o'clock, clockwise positive,
centidegrees in [-18000, +18000).

## How the reading works

Per frame, on the fabric: 3x3 blur (webcam denoise) -> mean
brightness (reduction) -> for each of 240 candidate angles, sum the
needle-side contrast `max(0, sign*(pixel - mean))` over 24 samples
along a precomputed ray from the dial centre.  The needle is the
angle whose ray accumulates the most contrast.  On the control core:
argmax + parabolic interpolation between neighbouring angles
(sub-half-degree resolution), a peak-vs-average confidence with an
absolute floor (a blank dial scores no confidence), an EMA smoother
that steps by shortest arc, and the linear angle->value calibration
(handles sweeps crossing the +/-180 wrap and clamps off-scale
needles to the nearer end).
