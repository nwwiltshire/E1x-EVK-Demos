# How the Hum Sentinel works

This document explains the full signal path, exactly which code runs
on the E1's dataflow fabric (and why), and what DEV and DEPLOY modes
do differently.  Every number in here was measured on the EVK.

## System overview

```
 laptop                                Electron E1 (EVK)
┌────────────────────────┐  serial   ┌─────────────────────────────────┐
│ mic / synth / wav      │ 115200 8N1│  RISC-V control core            │
│   ↓ 8 kHz int16 PCM    │           │   u-law decode (LUT)            │
│ u-law encode           │  AUDIO    │   reductions, event logic       │
│ 1024-sample chunks ────┼──────────►│      ↓        ↑                 │
│ (128 ms each)          │           │  ┌─────────────────────┐        │
│                        │ SPECTRUM  │  │   dataflow fabric   │        │
│ viewer GUI:            │◄──────────┼─ │ window · FFT · log  │        │
│  spectrum + baseline   │  STATUS   │  │ magnitude · per-bin │        │
│  waterfall, score,     │◄──────────┼─ │ excess & baseline   │        │
│  event log, power      │  ACK      │  └─────────────────────┘        │
│                        │◄──────────┤   2.08 ms per chunk             │
└────────────────────────┘           └─────────────────────────────────┘
```

The laptop is a stand-in for a field sensor: it captures audio,
compresses it to 8-bit u-law, and ships 128 ms chunks to the chip.
All analysis happens on the E1.  What comes back depends on the mode.

## The per-chunk pipeline

Each `AUDIO` message carries 1024 u-law samples (128 ms at 8 kHz).
The firmware processes it in seven stages:

1. **u-law decode** — 256-entry lookup table, byte → int16 PCM.
   u-law gives ~14-bit dynamic range in 8 wire bits, which is what
   lets a quiet room *and* a loud clap survive the 115200-baud link.

2. **Hann window** (`au_window`) — Q15 multiply per sample.  Without
   it, spectral leakage from the strong hum lines would smear across
   the spectrum and bury small anomalies.

3. **Bit-reversal permutation** (`au_bitrev_gather`) — table-driven
   gather that also zeroes the imaginary buffer, feeding the
   decimation-in-time FFT.

4. **1024-point radix-2 FFT** (`au_fft`) — int16 in-place, Q15
   twiddles, and a `>>1` at every one of the 10 stages so the math
   can never overflow (net 1/N scaling).  Yields 512 usable bins,
   7.8125 Hz each, DC–4 kHz.

5. **Log magnitude** (`au_logmag`) — `spec[i] = 8·log2(re² + im² +
   floor)` packed into a u8 (≈0.75 dB per unit).  The constant
   *mag² floor* matters: without it an empty bin's log value swings
   ~25 units when arithmetic noise wiggles its magnitude by 2 LSB,
   and the baseline can never settle.  With it, silence reads as a
   flat, stable value and real signals (mag² ≫ floor) are unaffected.

6. **Baseline compare** (`au_excess`) — each bin keeps a learned mean
   `mu` and mean-absolute-deviation `dev` (Q8 log units).  The
   trigger envelope is `mu + k·dev + margin`; per-bin excess is how
   far the current spectrum pokes above it.  The **score** is the sum
   of all excesses; the **top bin** is the argmax.

7. **Baseline update** (`au_baseline_update`) — exponential moving
   averages of `mu` and `dev`.  Fast (shift 2) during the learning
   phase, slow (shift 6) while watching, **frozen** while an anomaly
   is in progress — with a bound (see below).

### Learning, events, self-healing

- **Learning phase** — the first `learn_chunks` chunks (~5 s) after
  boot or a RELEARN: fast adaptation, score forced to 0, no events.
  The first chunk seeds `mu` directly from the observed spectrum.
- **Event hysteresis** — two consecutive over-threshold chunks open an
  event (debounces single-chunk clicks); `event_hold` consecutive
  quiet chunks close it.  Each event is classed by the frequency of
  the strongest deviant bin: **LOW** < 250 Hz (thump, rumble, bearing
  knock), **MID** 250 Hz–2 kHz (voice), **HIGH** > 2 kHz (keys,
  clink, hiss).
- **Bounded freeze** — adaptation freezes while the score is anywhere
  near the threshold, so a brief anomaly is never learned into the
  baseline.  But the freeze lasts at most ~3 s of audio
  (`DET_FREEZE_LIMIT`): a sound that persists longer is absorbed as
  the new normal and its event closes.  Without the bound, any
  lasting change to the room parks the score over the freeze gate
  forever and the detector flaps events indefinitely.

All tunables (threshold, k, margin, adaptation rate, hold, learning
length) are runtime parameters settable over the wire (`SET_PARAM`).

## What runs on the fabric — and why

The E1 pairs a RISC-V control core with a spatial **dataflow
fabric**.  Code only runs on the fabric if the function is marked
with effcc's `__efficient__` keyword — and the split is a design
decision, not an afterthought:

| Stage | Where | Why |
|---|---|---|
| `au_window` | fabric | flat loop, one multiply per sample |
| `au_bitrev_gather` | fabric | regular gather via const index table |
| `au_fft` | fabric | the compute bulk: ~5k butterflies × 4 multiplies |
| `au_logmag` | fabric | per-bin, fixed-step log approximation |
| `au_excess` | fabric | per-bin compare, no cross-bin dependencies |
| `au_baseline_update` | fabric | per-bin EMA, same shape |
| u-law decode | control core | 1024 table lookups, trivial |
| score sum / argmax | control core | reductions, cheap at n=512 |
| event hysteresis, classification | control core | branchy, stateful, tiny |

Fabric functions follow the SDK's rules: plain loops over contiguous
arrays, `restrict` pointers, no libc calls, const tables passed in as
pointer arguments.  The log2 approximation deliberately uses a
fixed 5-step binary search instead of a data-dependent `while` loop —
fixed structure maps better onto spatial hardware.

**Measured result** (identical firmware source, two builds):

| Build | Per-chunk compute | Realtime headroom |
|---|---|---|
| `hum_sentinel_scalar` (all control core) | 8.26 ms | 15× |
| `hum_sentinel_fabric` | **2.08 ms** | **62×** |

The 4.0× speedup is the fabric executing the six annotated stages
spatially.  The headroom is the demo's real story: at 2 ms per 128 ms
chunk the chip is ~98 % idle, which is why the whole E1x package sits
at ~2.6 mW (measured on the EVK's VDDIO rail) — about 72 days of
continuous listening on one AA cell.

One warning worth repeating: **an unannotated "fabric" build compiles
and runs without any error — entirely on the control core.** The SDK
defines `__efficient__` away for every non-fabric target, so
forgetting the keyword produces a working binary that is silently 4×
slower.  Always verify with the on-chip timer (`STATUS.chunk_us`).

## DEV mode vs DEPLOY mode

The two modes run **exactly the same detection pipeline**.  They
differ only in what the firmware transmits back — which is the point
of the demo: the analysis quality is identical whether or not the
data leaves the chip.

### DEV — "watch it think"

Per chunk the firmware replies, in order:

1. `SPECTRUM` (392 bytes on the wire) — three 128-bin u8 planes,
   max-pooled 4:1 from the internal 512 bins:
   - the current spectrum,
   - the learned baseline mean `mu`,
   - the trigger envelope `mu + k·dev + margin`.
2. `STATUS` (19 bytes) — score, event flag + class, learning state
   and progress, mode, top bin, on-chip compute time in µs,
   cumulative event count.
3. `ACK` (7 bytes) — always **last**: the EVK's USB bridge is only
   clean half-duplex, and the ACK is the host's licence to transmit
   the next chunk, so it must not overtake the reply data.

The viewer uses this to draw the live spectrum with baseline/trigger
overlays, the scrolling spectrogram, and the score chart — you can
see the yellow trigger envelope hug the hum lines, watch a jingle
poke red bars through it, and watch the baseline absorb a persistent
sound.

Wire cost: 1032 bytes up + 418 bytes down ≈ 134 ms of line time,
~165–186 ms round-trip in practice → ~5 chunks/s, about 60–75 % of
realtime audio.  The host compensates by always sending the *newest*
audio and dropping the backlog, so detection latency stays at a few
hundred ms.

### DEPLOY — "nothing leaves but the verdict"

Per chunk the firmware replies only `STATUS` + `ACK` (26 bytes).  No
spectrum, no baseline, no audio-derived signal data of any kind
leaves the device — the only outputs are the anomaly score, the
event flag/class, and counters.  The viewer replaces the spectrum
panels with a placard showing the event state and count.

Two side effects worth knowing:

- DEPLOY is *faster*: the reply shrinks ~16×, so the link approaches
  full realtime coverage.
- The privacy claim is precise: in DEPLOY the host could reconstruct
  nothing about the room's audio beyond "how anomalous was it, in
  which coarse band" — 4 bytes of information per 128 ms.

Toggling: press `d` in the viewer, send `SET_PARAM mode`, or press
the EVK user button (SW12).  The mode is reported in every STATUS so
the UI always reflects the chip's actual state.

## Wire protocol at a glance

Little-endian, framed `AA 55`, CRC16-CCITT over everything after the
magic.  The parser resyncs on garbage by scanning for the magic, and
the firmware NAKs bad CRCs so the host retries (3 attempts, then the
chunk is dropped and fresh audio takes its place).

| Message | Direction | Payload |
|---|---|---|
| `AUDIO` 0x01 | host → E1 | seq, 1024 u-law bytes |
| `SET_PARAM` 0x02 | host → E1 | param id, i32 value |
| `GET_STATUS` 0x03 | host → E1 | — |
| `ACK/NAK` 0x10 | E1 → host | seq (or param id), ok |
| `STATUS` 0x11 | E1 → host | score, flags, class, mode, learn %, top bin, chunk µs, events |
| `SPECTRUM` 0x12 | E1 → host | 128-bin spectrum + baseline + trigger (DEV only) |

One message is in flight at any moment, in either direction — that
discipline, plus ACK-last, is what keeps the half-duplex USB bridge
from dropping bytes.
