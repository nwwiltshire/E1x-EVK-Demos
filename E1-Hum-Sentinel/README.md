# E1 Hum Sentinel — acoustic anomaly detection on the Electron E1

A privacy-first acoustic monitor on the [Efficient Computer](https://www.efficient.computer)
**Electron E1** (EVK board).  The laptop streams 8 kHz u-law audio to
the E1 over serial; the chip learns the room's baseline hum, runs a
1024-point FFT + spectral novelty detector **on the dataflow fabric**,
and reports back:

- **DEV mode** — live spectrum, learned baseline, trigger envelope,
  score, classed events: *watch it think*.
- **DEPLOY mode** — score + events only.  The audio is analyzed
  on-chip; no spectrum or audio-derived data leaves the device.

Tap a bearing, jingle keys, speak — the sentinel flags it within a few
hundred ms and classes it LOW (thump/rumble), MID (voice), or
HIGH (keys/clink) by the dominant deviant frequency band.

**Measured on the EVK (2026-07-09):**

| | |
|---|---|
| E1 compute, fabric build | **2.08 ms / 128 ms chunk** (62x realtime) |
| E1 compute, scalar build | 8.26 ms / chunk (fabric is **4.0x faster**) |
| Link (fixed 115200, half-duplex) | ~165–186 ms/chunk round trip, ~5 chunks/s |
| E1x chip power (VDDIO rail) | **~2.6 mW** → ~72 days on one AA |

**Docs:** [How it works](docs/HOW_IT_WORKS.md) — the signal chain,
what runs on the fabric, and DEV vs DEPLOY in detail.
[Developing on the E1 EVK](docs/DEVELOPING_ON_E1.md) — the field
notes to start from when building the next application.

## Layout

```
firmware/   portable C99 core + two HAL backends
  main.c        superloop: rx chunk -> detector -> tx replies
  detector.[ch] baseline model, event hysteresis, classification
  dsp.[ch]      fabric stages: window, FFT, log-mag, per-bin excess
  fft_tables.c  generated const tables (tools/gen_tables.py)
  protocol.[ch] framing, CRC16, parser/builders
  hal_posix.c   simulator: TCP loopback serial   (make sim)
  hal_e1.c      EVK: UART_3 link via SDK          (make e1)
  echo_main.c   step-1 link bring-up app (gap-echo)
host/
  sentinel_viewer.py  the GUI (spectrum, waterfall, score, events, power)
  audio_source.py     mic (arecord) / synth / wav sources + u-law codec
  e1proto.py          Python mirror of protocol.h
  power_monitor.py    EVK power-CSV telemetry (or mock)
  hw_smoke.py         ~1 min hardware acceptance test
  echo_test.py        link bring-up check against link_echo
tests/      unit tests (FFT vs DFT reference, detector) + sim e2e
tools/      gen_tables.py — regenerates fft_tables.[ch]
```

## Quick start (no hardware)

```sh
python3 -m venv venv && venv/bin/pip install -r host/requirements.txt
make sim                          # gcc build of the firmware
./firmware/build/firmware_sim &   # "serial" = TCP 127.0.0.1:5555
venv/bin/python3 host/sentinel_viewer.py --sim 127.0.0.1:5555 --source synth
```

Give it ~5 s to learn the baseline, then press `1`/`2`/`3` to inject a
thump/voice/jingle.  Keys: `d` DEV/DEPLOY, `r` relearn, `t/T`
threshold, `q` quit.  `--source mic` uses the laptop microphone
(anything you do near the laptop becomes the room), `--source
wav:file.wav` loops a recording.  The inject keys work on any source.

Tests: `make test` (369 unit checks + protocol/detector e2e against
the real sim binary).

## Going to hardware

Build (needs the Efficient SDK at `~/effcc`, version 25.4):

```sh
cd firmware && cmake -B bld && cmake --build bld
```

Produces `bld/scalar/hum_sentinel.hex` (all control-core),
`bld/fabric/hum_sentinel.hex` (DSP loops on the dataflow fabric — use
this one), and `bld/scalar/link_echo.hex` (bring-up).

Flash (BOOT switches 101 for SRAM; volatile — reflash after power
cycle.  `mram` + BOOT 010 to persist for demo day):

```sh
~/effcc/bin/eff-flash bld/fabric/hum_sentinel.hex sram
```

Bring-up sequence:

```sh
venv/bin/python3 host/echo_test.py --port /dev/ttyACM2   # link_echo flashed
venv/bin/python3 host/hw_smoke.py  --port /dev/ttyACM2   # hum_sentinel flashed
venv/bin/python3 host/sentinel_viewer.py --port /dev/ttyACM2 \
    --power-port /dev/ttyACM1                            # the demo
```

### The fabric is where the speedup lives

Every hot loop (`au_window`, `au_bitrev_gather`, `au_fft`,
`au_logmag`, `au_excess`, `au_baseline_update`) is annotated
`__efficient__` — effcc's keyword marking a function for
dataflow-fabric compilation.  **An unannotated "fabric" build silently
runs 100% on the RISC-V control core** and benchmarks identical to the
scalar build; the SDK neutralizes the keyword everywhere else
(`-D__efficient__=`), so forgetting it produces no error.  Fabric
functions here follow the SDK examples: plain loops over contiguous
arrays, `restrict` pointers, no libc, const tables passed in as
arguments.  Result: the full chunk pipeline (decode, window, 1024-pt
int16 FFT, log-magnitude, per-bin scoring) drops from 8.26 ms to
2.08 ms.

Sequential, branchy work (event hysteresis, argmax/sum reductions,
u-law LUT decode) stays on the control core by design.

### Link facts (measured; see docs/DEVELOPING_ON_E1.md for the full story)

- The EVK USB bridge runs its UART at a **fixed ~115200** regardless
  of the host's CDC request — 115200 is both floor and ceiling
  (~10.8 KB/s each way).
- The link is **only clean half-duplex**: the firmware replies
  SPECTRUM → STATUS → **ACK last** (the host's licence to transmit),
  and the host never transmits while a reply is in flight.
- One 1 KB audio chunk + DEV replies round-trips in ~165–186 ms →
  ~5 chunks/s ≈ 60–75% audio coverage.  The host always sends the
  *newest* audio and drops the backlog, so detection latency stays at
  a few hundred ms regardless.  (DEPLOY replies are smaller; coverage
  is higher.)

## EVK ports and power telemetry

Official port map (EVK Getting Started guide): **ttyACM0** =
programmer (`eff-flash` finds it itself), **ttyACM1** = power CSV,
**ttyACM2** = E1x stdio = the chunk link.  With effcc's udev rule
installed these also appear as `/dev/eff-prog`, `/dev/eff-power`,
`/dev/eff-console`.

The power stream starts only when the port is opened with DTR asserted
(pyserial default) — it looks silent to `cat`.  Rows carry *measured*
mA/mV/mW per rail: `SYS` = whole board (~82 mW), `1V8` = chip + MRAM +
MCU, `VDDIO` = **the E1x chip alone (~2.6 mW here)**, `VDDVAR` = the
scalar core + fabric + peripherals (~1 mW).  The viewer's power panel
(`--power-port /dev/ttyACM1`) shows the chip-only headline and the
one-AA projection.

## Burn mode (constant-workload power measurement)

The ~2.6 mW above is the **average power at this demo's duty cycle**:
compute is 2.08 ms per chunk but the fixed-115200 link delivers a
chunk only every ~186 ms, so the fabric is busy ~1% of the time and
the VDDIO rail reads as nearly all idle power.  Efficient Computer's
internal benchmarking runs *constant* workloads instead — their fft4k
reference measures **~4.8 mW** (and a conv3x3 workload ~9 mW) — so
the two kinds of number are only comparable if you say which one you
are quoting.

Burn mode produces the constant-workload number.  Press `b` in the
viewer (or send `SET_PARAM burn=1`, id 9) and the firmware re-runs
the fabric pipeline (window → FFT → log-magnitude → excess) back to
back on the last chunk's PCM, one iteration per serial poll — ~99%
fabric duty.  The viewer pauses audio streaming while burn is on: the
EVK's UART rx FIFO is only 16 bytes deep, so a 1 KB chunk landing
mid-iteration would overflow it (a 10-byte SET_PARAM fits).  That
pause also makes the measurement purer — zero link traffic, exactly
like a benchmark run.  Detector state (baseline, score, events) is
untouched; press `b` again and the demo resumes where it left off.

Measurement procedure (give each reading 30+ s; use the 10 s average
shown in the power panel):

1. **Idle floor** — firmware flashed, viewer connected, burn off, no
   audio streaming: VDDIO = P_idle.
2. **Normal demo** — the duty-cycled average (~2.6 mW here).  Quote
   it as "average power at this demo's duty cycle".
3. **Burn** — the constant-workload draw; compare against EC's
   ~4.8 mW fft4k benchmark and quote it as "constant workload".

Derived: energy per chunk ≈ (P_burn − P_idle) × 2.08 ms.

### Battery projection flags

`power_monitor.py` (standalone) and the viewer project life on one AA
cell (3000 mWh ≈ 2500 mAh at ~1.2 V, energy-based so no voltage
domains get mixed) for a duty-cycled deployment — *workload of W mW
run for R s every P hours*:

    --workload-mw       workload power (default: the live 10 s chip
                        average, so with burn ON the measured
                        constant-workload draw fills in automatically)
    --workload-runtime  seconds per wake-up (default 2)
    --workload-period   hours between wake-ups (default 1)
    --sleep-mw          sleep power between wake-ups (default 0)

## Protocol

Little-endian, magic `AA 55`, CRC16-CCITT over everything after the
magic.  Host → firmware: `AUDIO` (seq, 1024 u-law bytes = 128 ms),
`SET_PARAM` (id, i32), `GET_STATUS`.  Firmware → host: `ACK/NAK`,
`STATUS` (score, event/learning flags, class, mode, learn %, top bin,
chunk µs, event count), `SPECTRUM` (DEV only: 128-bin spectrum +
baseline + trigger envelope, u8 log units).  One chunk in flight; NAK
or 10 s timeout → retry (3x), then drop and continue with fresh audio.

Params (`SET_PARAM` ids): 1 threshold, 2 k (Q4 dev multiplier),
3 mode, 4 adapt shift, 5 margin, 6 event hold, 7 learn chunks,
8 relearn (command), 9 burn (constant-workload power soak).

## How detection works

Per chunk: u-law decode → Hann window → 1024-pt fixed-point FFT →
`8*log2(mag² + floor)` per bin (u8, ~0.75 dB/unit; the mag² floor
stops empty bins from flapping in the log domain).  Each bin keeps an
EMA mean `mu` and mean-abs-deviation `dev` (Q8): fast EMA for the
first `learn_chunks` chunks (~5 s), slow afterwards.  Per-bin excess =
`max(0, x - mu - k*dev - margin)`; score = sum of excesses.  Two
consecutive over-threshold chunks open an event (classed by the
argmax bin's band: <250 Hz LOW, <2 kHz MID, else HIGH); `event_hold`
quiet chunks close it.  While the score is anywhere near the
threshold, baseline adaptation freezes — the model never learns a
brief anomaly.  The freeze is bounded (~3 s): a sound that persists
past it is absorbed as the new normal, so the detector always heals
itself instead of alarming forever.  The viewer sends RELEARN on
connect (`--no-relearn` to keep the firmware's existing baseline).
