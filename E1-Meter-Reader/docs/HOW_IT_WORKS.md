# How the E1 Meter Reader works

This document walks the whole pipeline — webcam to reading — and then
explains the two operating modes, DEV and DEPLOY, in detail.  For the
hands-on runbook see the [README](../README.md); for the hard-won EVK
facts (link physics, fabric rules, board switches) see
[DEVELOPING_ON_E1.md](DEVELOPING_ON_E1.md).

## The big picture

```
                HOST (laptop)                          E1 EVK
 ┌────────────────────────────────────┐   serial   ┌─────────────────────────────┐
 │ webcam 640x480                     │  115200 8N1 │  RISC-V control core        │
 │   └─ ROI crop (user-dragged       │  half-duplex│    superloop: parse, ack,   │
 │      circle) -> 64x64 grayscale   ─┼────────────┼─>  argmax, smooth, calibrate│
 │ GUI: needle overlay, polar         │             │  dataflow FABRIC            │
 │      "thinking" plot, reading,    <┼─────────────┼─   blur -> mean -> 240-angle│
 │      power telemetry               │  scores +   │    ray-cast evidence        │
 └────────────────────────────────────┘  reading    └─────────────────────────────┘
```

The division of labour is deliberate:

- **The host does acquisition only.**  It never analyses the image —
  it crops the gauge to a square, downsamples to 64x64 grayscale, and
  ships raw pixels.  (The link physically cannot carry full frames:
  see "Why 64x64" below.)
- **The E1's dataflow fabric does all image analysis** — three
  kernels totalling ~10k arithmetic ops per frame, in 0.76 ms.
- **The E1's control core does the tiny sequential part** — picking
  the winning angle and turning it into a calibrated number — and
  owns the protocol.

## Host side: from webcam to wire

1. Frames are captured at 640x480 (or synthesized — `--source synth`
   renders an animated gauge so the whole stack runs with no camera).
2. The user drags a circle over the dial face.  The square bounding
   that circle is cropped, converted to grayscale, and resized to
   64x64 with area interpolation.  The GUI shows this exact image
   ("E1 INPUT 64x64") so there is never a question about what the
   chip received.
3. The 4096-byte frame goes down the wire as one framed message.
   **One message is ever in flight**; the host does not transmit
   again until the chip's ACK arrives (the link is only clean
   half-duplex — simultaneous traffic gets silently dropped by the
   EVK's USB bridge).  A frame that gets NAKed (CRC error) or times
   out is retried twice, then dropped — a live demo must never wedge
   on a link hiccup.

### Why 64x64

The EVK's USB bridge runs its UART at a fixed ~115200 baud, ~10.8 KB/s
each way, and no host-side setting can change that.  Frame sizes are
therefore a straight latency trade:

| geometry | bytes | time on the wire | demo feel |
|---|---|---|---|
| 200x150 (people-counter demo) | 30,000 | ~2.8 s | slideshow |
| 96x96 | 9,216 | ~0.9 s | sluggish |
| **64x64** | **4,096** | **~0.38 s** | **~2 fps, feels live** |

At 64x64 the dial is still ~30 pixels in radius.  With 240 candidate
angles and sub-step interpolation the reader resolves the needle to
well under a degree (measured: worst 0.75°), which on a 270° scale is
~0.3 % of full range — better than most people can read a gauge.

## The wire protocol

Little-endian, every message framed as `AA 55 | type | body | CRC16`
(CRC16-CCITT over everything after the magic).  Full byte layout in
`firmware/protocol.h`; `host/e1proto.py` is its Python mirror.

| message | direction | payload |
|---|---|---|
| `FRAME` (0x01) | host → E1 | seq, 4096 px |
| `SET_PARAM` (0x02) | host → E1 | param id, i32 value |
| `GET_STATUS` (0x03) | host → E1 | — |
| `ACK/NAK` (0x10) | E1 → host | seq, ok |
| `STATUS` (0x11) | E1 → host | angle, value, confidence, flags, mode, frame_us, frames, mean |
| `SCORES` (0x12) | E1 → host | 240 × u16 evidence array — **DEV mode only** |

One frame exchange, in order:

```
host:  FRAME(seq, 4096 bytes)          ~380 ms on the wire
E1:    SCORES(seq, 480 bytes)          DEV mode only, ~45 ms
E1:    STATUS(reading)                 21 bytes
E1:    ACK(seq)                        last — the host's licence to send again
```

The ACK coming *last* is a rule, not a convention: it is what keeps
the half-duplex link collision-free.  Robustness against a noisy or
interrupted link comes from three mechanisms: CRC16 + NAK + host
retry; parser resync (both sides scan forward for the next `AA 55`);
and a firmware stall reset that abandons a half-received message
after 500 ms of line silence.

Parameters (all set from the GUI, ACKed individually): mode,
polarity, smoothing strength, confidence floor, and the four
calibration values.  Parameter writes are queued on the host and only
sent when no frame is in flight — same half-duplex discipline.

## On the chip

### The fabric pipeline (kernels.c — all `__efficient__`)

Everything data-parallel runs on the E1's spatial fabric.  The three
kernels are plain C loops over contiguous arrays — the shape the
fabric compiler wants — with `restrict` pointers, integer math only,
no libc, and constant tables passed in as arguments:

1. **`gk_blur3x3`** — a [1 2 1; 2 4 2; 1 2 1]/16 weighted blur.
   Webcam sensors are noisy at 64x64; one denoise pass costs almost
   nothing on the fabric and visibly stabilises the evidence array.
2. **`gk_pixel_sum`** — a full-frame brightness reduction.  The mean
   (sum >> 12) becomes the reference the needle must contrast
   against, which makes the reader indifferent to ambient light
   level: nothing is hard-coded about "dark" or "bright".
3. **`gk_ray_scores`** — the heart of the reader.  A build-time
   Python script (`tools/gen_ray_tables.py`) precomputes, for each of
   240 candidate needle angles, the 24 pixel indices along a ray from
   the dial centre (skipping the hub, stopping inside the rim).  The
   kernel scores every angle:

   ```
   score[a] = Σ over the 24 ray samples of  max(0, sign·(pixel − mean))
   ```

   `sign` is −1 for a dark needle on a light face, +1 for the
   inverse (the `p` key).  A needle is a long, high-contrast radial
   feature, so the ray that lies along it accumulates far more
   contrast than any tick mark (short) or printed label (off-axis).
   The `dst[i] = src[table[i]]` gather pattern is exactly what the
   fabric executes well, and the fixed 240×24 loop structure gives
   the spatial hardware a static dataflow graph.

The full fabric pass is measured on-chip at **0.76 ms/frame**; the
identical source compiled for the control core takes 7.39 ms (9.7×).
Every STATUS carries that measurement (`frame_us`), which is also the
tripwire for the classic E1 footgun: forgetting an `__efficient__`
annotation compiles silently and just runs slow.

### The control core (gauge.c)

The remaining work is branchy and tiny — the wrong shape for spatial
hardware, and cheap enough not to matter:

- **Peak pick + parabolic interpolation.**  Argmax over the 240
  scores, then a three-point parabola through the peak's neighbours
  refines the angle to a fraction of the 1.5° step (integer Q8
  math).
- **Confidence.**  `255·(peak − mean_score)/peak`, derated linearly
  when the absolute peak is below a floor (~24 samples × 25 grey
  levels).  The ratio alone would let a blank dial's noise spread
  masquerade as signal; the floor kills that.  Below the confidence
  threshold the reading freezes at the last good angle and the GUI
  shows NO NEEDLE.
- **EMA smoothing.**  Exponential moving average on the angle,
  stepping by the *shortest signed arc* so it behaves at the ±180°
  wrap, with strength 0–6 (`s` key; 0 = raw).
- **Calibration.**  The linear map from angle to value.  Angles are
  handled as "degrees travelled clockwise from the scale minimum",
  which makes sweeps that cross straight-down (±180°) work, and
  off-scale needles clamp to the nearer end of the scale.  The host
  never computes the value: it sends the four calibration constants
  once and the chip reports engineering units from then on.

## DEV mode vs DEPLOY mode

The two modes run **exactly the same analysis** on exactly the same
frames — same kernels, same reading, same accuracy.  The only
difference is what the chip is willing to say about its reasoning.

### DEV — "watch it think"

In DEV mode, every frame's reply includes the `SCORES` message: the
raw 240-entry evidence array straight out of the fabric kernel.  The
GUI draws it as the polar plot — 240 spokes, one per candidate angle,
radius proportional to accumulated contrast.  A locked-on needle
shows as a single sharp spike that swings with the dial; a cluttered
scene shows as petals of competing evidence.  This is the mode for
development, calibration (the host derives the raw uninterpolated
peak from this array when you press `[` / `]`), and the first half of
a customer demo: it proves the chip is doing real measurement, not
theatre.

Cost: 488 extra bytes per frame on the return path, worth ~45 ms of
link time (measured DEV round trip: 0.46 s).

### DEPLOY — "nothing leaves the chip"

In DEPLOY mode the SCORES stream stops.  Per frame, the chip returns
a 21-byte STATUS and a 7-byte ACK — the reading, its confidence, and
housekeeping — and **no image-derived data of any kind**.  The GUI
replaces the polar plot with a placard saying exactly that, and the
visible collapse of return traffic *is* the product pitch:

- **Privacy/security story.**  The camera pixels' journey ends at the
  chip.  A deployed meter-reading node exposes a 28-byte-per-reading
  telemetry surface, not a video feed — there is no image to leak,
  subpoena, or exfiltrate.  (In a real product the camera would be
  wired directly to the E1 and raw pixels would never exist outside
  it; on this demo the host necessarily sees the webcam, so the
  placard describes the return path.)
- **Bandwidth/power story.**  28 bytes per reading is LPWAN-sized.  A
  gauge that needs one reading a minute could duty-cycle the whole
  system aggressively; the E1x itself was measured at 2.7 mW *while
  streaming 2 fps* — the fabric is busy for 0.76 ms and idle the
  other 99.8 % of each frame period.

Switching modes: press `d` in the GUI, press the EVK user button
SW12, or send `SET_PARAM mode`.  The current mode is echoed in every
STATUS, so the GUI always displays the chip's actual state rather
than what it last requested — after a reflash or power cycle the
truth wins within one status message.

| | DEV | DEPLOY |
|---|---|---|
| analysis performed | identical | identical |
| reading + confidence | yes | yes |
| SCORES evidence array | every frame (488 B) | never |
| return traffic per frame | ~516 B | ~28 B |
| measured round trip | 0.46 s | 0.44 s |
| GUI right panel | polar thinking plot | "nothing leaves the chip" placard |
| intended use | development, calibration, demo act 1 | production posture, demo act 2 |

## Testing architecture

The firmware core is portable C99 behind a six-function HAL, which
buys the whole test ladder without hardware:

1. **C unit tests** (`tests/test_gauge.c`) — gcc compiles the *exact*
   fabric kernels (the `__efficient__` keyword is neutralised by a
   guard macro) and checks them against double-precision reference
   implementations, including a full needle sweep on synthetically
   rendered dials at both polarities.
2. **Simulator e2e** (`tests/e2e_check.py`) — the firmware builds as
   a Linux process whose "serial port" is a loopback TCP socket; the
   test drives the real wire protocol through it: accuracy sweep,
   calibration, CRC corruption/resync, DEPLOY leak check, polarity,
   smoothing.
3. **Hardware smoke** (`host/hw_smoke.py`) — the same scenarios over
   the real serial link in ~1 minute, robust to a warm (already
   running) device.  Run it after every flash.

`make test` runs tiers 1–2; the GUI was written only after all three
tiers passed — the same order that worked for the previous two E1
demos.
