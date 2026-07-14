# Task: Burn mode + duty-cycle AA battery projection for both EVK demos

Implement everything described here, then run the verification steps at the end.

## Background — the problem

This repo holds two demos for the Efficient Computer **Electron E1 (E1x)** EVK:

- **E1-Hum-Sentinel** — streams 8 kHz u-law audio chunks to the chip; a 1024-pt FFT +
  spectral novelty detector runs on the dataflow fabric. Measured E1x chip power
  (VDDIO rail): **~2.6 mW**.
- **E1-Meter-Reader** — streams 64x64 webcam frames; blur + 240-angle ray-cast needle
  search runs on the fabric. Measured: **~2.7 mW**.

Efficient Computer engineering reviewed those numbers and said they are LOW versus
their internal benchmarks, because we are measuring a time-averaged number at a tiny
duty cycle. Their feedback, verbatim:

> As far as the power number accuracy, I checked these against the power figures in
> our internal benchmarking system with similar workloads:
> Audio demo: ~2.6 mW — This number is a bit low, but it may be because you're looking
> at a time-averaged number. You may be working with a very small duty cycle, so the
> chip idles most of the time. Our benchmarking tests a constant workload for fft4k
> and results in ~4.8 mW for this type of work. You should still be ok using that
> number as long as you frame it as the average power for that demo's duty cycle.
> Video demo: ~3.6 mW — This trends in the right direction since these workloads do
> use a bit more power. Still the same caveat where you might be seeing a
> time-averaged number influenced by duty cycle. Our benchmarking tests a conv3x3
> constant workload and we see ~9 mW.

The analysis confirms this quantitatively:

- Hum Sentinel: compute is **2.08 ms/chunk** but the fixed-115200 link delivers a chunk
  only every ~186 ms → **~1.1% fabric duty cycle**. If idle floor is ~2.55 mW and
  constant-workload compute adds ~2.2 mW (consistent with EC's 4.8 mW fft4k), then
  2.55 + 1.1% x 2.2 ≈ 2.58 mW — almost exactly what we measured.
- Meter Reader: **0.76 ms/frame** at ~0.46 s/frame round trip → **~0.17% duty**; the
  2.7 mW reading is essentially all idle floor.
- The link **cannot** be driven faster: the EVK USB bridge runs a fixed ~115200 baud
  (floor AND ceiling, see each README's "Link facts"). Saturation must happen on-chip.

## Goal (user's request)

1. Add a **burn mode** to BOTH demos: a host-toggleable firmware mode that re-runs the
   fabric pipeline continuously so the VDDIO rail shows the **constant-workload** power
   draw (comparable to EC's benchmarking) instead of a duty-cycled average.
2. Change the **power reader display** (power_monitor.py standalone + both viewers'
   power panels) to show **life on 1x AA battery given a workload power draw of
   ___ mW run for a runtime of ___ s every ___ hours** — i.e. a fill-in-the-blank
   duty-cycle projection, with the blanks as CLI flags (and the workload mW defaulting
   to the live measured chip power, so with burn ON the measurement self-fills).

## Architecture facts you need (verified by reading the code)

- The two demos are structurally parallel. Firmware: portable C99 superloop over a
  6-function `hal.h`; `hal_posix.c` (simulator, "serial" = loopback TCP, `make sim`)
  and `hal_e1.c` (EVK UART3). Host: OpenCV viewer, `e1proto.py` protocol mirror,
  `power_monitor.py` power-CSV telemetry.
- `E1-Hum-Sentinel/host/power_monitor.py` and `E1-Meter-Reader/host/power_monitor.py`
  are **byte-identical**. Keep them identical after editing.
- Power CSV rows: `timestamp(us)` then `mA, mV, mW` per rail SYS/1V8/VDDIO/VDDVAR.
  mW is measured on the board (no host-side P=V*I). **VDDIO = the E1x chip alone** —
  the headline rail. The monitor keeps only the latest sample today.
- Protocol: `AA 55` magic, CRC16-CCITT, one message in flight. SET_PARAM(id, i32) is
  ACKed with the param id in the seq slot. Reply order per chunk/frame:
  SPECTRUM/SCORES → STATUS → **ACK last** (ACK = host's licence to transmit; the link
  is only clean half-duplex).
- **CRITICAL HW constraint:** the E1's UART rx FIFO is only **16 bytes** deep
  (`hal_e1.c` drains it hot; see the comment at the `continue` in `hal_serial_read`).
  At ~10.85 bytes/ms line rate, one 2.08 ms burn iteration ≈ 22 bytes of potential
  inbound traffic — a 1 KB AUDIO chunk arriving mid-burn would overflow the FIFO and
  corrupt the stream. A 10-byte SET_PARAM message fits the FIFO even if it lands
  entirely within one iteration. **Design consequence:** burn runs ONE pipeline
  iteration per serial poll, and the host viewer PAUSES audio/frame streaming while
  burn is on (only params flow). This also makes the measurement purer — zero link
  traffic, exactly like EC's constant-workload benchmark.
- `hal_serial_read(buf, len, timeout_ms=0)` is a clean non-blocking poll in BOTH
  backends (verified: hal_e1 checks the FIFO once then breaks; hal_posix uses
  `poll(..., 0)`).
- Param ID maps: Hum Sentinel uses 1..8 (8 = PARAM_RELEARN, a command) → new
  **PARAM_BURN = 9**. Meter Reader uses 1..9 (9 = PARAM_RESET, a command) → new
  **PARAM_BURN = 10**.
- Hum Sentinel's `detector_t` keeps `pcm/wtmp/re/im/spec/excess` buffers, so burn can
  re-run the fabric stages in place: it recomputes the SAME spec/excess from the same
  pcm, so the detector state is undisturbed. It must SKIP `au_baseline_update` and the
  event hysteresis — re-seeing the same chunk hundreds of times per second would slam
  the learned baseline.
- Meter Reader's `gauge_process(g, p, pixels)` gets `pixels` as a pointer into the
  parser buffer (valid only until the next byte is parsed) and blurs into a
  function-static `s_blur`. For burn, the raw frame must be RETAINED: add a
  file-static `s_frame[GK_IMG_N]` (4096 B), copy `pixels` into it at the top of
  `gauge_process` with a plain loop (no libc in firmware core), and run the kernels
  from `s_frame`.
- Tests that constrain this change: `E1-Hum-Sentinel/tests/test_dsp.c:test_params` and
  `E1-Meter-Reader/tests/test_gauge.c:test_params` assert unknown id 99 is rejected
  (ids 9/10 are currently unused → safe to claim). Both `tests/e2e_check.py` drive the
  real sim binary over TCP.
- Viewer structure: `sentinel_viewer.py` — keys in `App.handle_key`, chunk send gated
  in `App.run` (`start_chunk` when nothing in flight and no pending params), power
  panel in `_draw_bottom` (panel y0 = 556, height 154 → room for a third text line at
  y0+80; keys help line at y0+h-12). `meter_viewer.py` — keys in `handle_key`, frame
  send is the `else:` branch in the main loop, power block drawn in `render()` under
  the polar panel at `pyy = MARGIN + POLAR_S + 30 = 424` with only ~76 px of vertical
  room before the bottom strip at y=500 → keep the power block lines compact
  (title at pyy, rails at +24 @0.45 scale, chip line at +44 @0.5, projection at
  +64 @0.45, and keep the projection string short).
- Current battery figure: `battery_hours() = 2500 mAh / VDDIO mA` (continuous). Note
  it mixes voltage domains (AA delivers ~1.2 V avg through a boost converter; VDDIO is
  1.8 V), which is why the NEW projection must be **energy-based** (mWh / mW).

## Implementation

### A. Hum Sentinel firmware (`E1-Hum-Sentinel/firmware/`)

**detector.h** — add to `params_t`:
```c
    int32_t burn;         /* 9: 1 = fabric power soak between chunks (0) */
```
add to the enum:
```c
    PARAM_BURN         = 9,
```
declare after `detector_process`:
```c
/* Re-run the fabric pipeline on the last chunk's PCM without touching
 * the baseline, score, or event state.  Constant-workload power soak
 * (PARAM_BURN): the demo's ~1% compute duty cycle makes the VDDIO rail
 * read as nearly all idle power; looping this saturates the fabric so
 * the rail shows the constant-workload draw instead. */
void detector_burn(detector_t *d, const params_t *p);
```

**detector.c** — `params_defaults`: `p->burn = 0;`. `params_set`: new case accepting
0/1 (match the file's compact `case` style). New function (place before
`detector_viz`):
```c
void detector_burn(detector_t *d, const params_t *p)
{
    /* Exactly the fabric stages of detector_process, on the buffers
     * the last chunk left behind (recomputing the same spec/excess, so
     * the next real chunk sees nothing changed).  The u-law decode and
     * the control-core reductions/hysteresis are skipped — this is the
     * fabric workload, isolated. */
    au_window(d->pcm, AU_HANN_Q15, d->wtmp, AU_FFT_N);
    au_bitrev_gather(d->wtmp, AU_BITREV, d->re, d->im, AU_FFT_N);
    au_fft(d->re, d->im, AU_TW_RE, AU_TW_IM, AU_FFT_N);
    au_logmag(d->re, d->im, d->spec, AU_NBINS, AU_MAG2_FLOOR);
    au_excess(d->spec, d->mu, d->dev, d->excess, AU_NBINS,
              (int)p->k_q4, (int)(p->margin << 8));
}
```

**main.c** — in the superloop, between the parser-stall reset and the serial read:
```c
        /* Burn mode: saturate the fabric so VDDIO shows the constant-
         * workload draw, not a ~1% duty-cycle average.  ONE iteration
         * (~2 ms) per serial poll: the EVK rx FIFO is only 16 bytes
         * deep, so anything bigger than a SET_PARAM (10 bytes) sent
         * mid-iteration would overflow it — the host stops streaming
         * audio while burn is on and only params arrive. */
        if (s_params.burn)
            detector_burn(&s_det, &s_params);

        n = hal_serial_read(s_rx, (int)sizeof s_rx, s_params.burn ? 0 : 20);
```
(replacing the existing `n = hal_serial_read(s_rx, (int)sizeof s_rx, 20);`).

### B. Meter Reader firmware (`E1-Meter-Reader/firmware/`)

**gauge.h** — add to `params_t`: `int32_t burn;` (comment `/* 10: fabric power soak */`).
Add `PARAM_BURN = 10,` to the enum. Declare:
```c
/* Re-run the fabric kernels on the last frame without touching the
 * reading, smoother, or counters — constant-workload power soak
 * (PARAM_BURN).  See gauge_burn in gauge.c. */
void gauge_burn(gauge_t *g, const params_t *p);
```

**gauge.c** —
- `params_defaults`: `p->burn = 0;`; `params_set`: case PARAM_BURN accepts 0/1.
- Hoist the blur buffer to file scope and add the frame copy:
```c
/* Kernel input/output buffers (single instance; no reentrancy).  The
 * raw frame is retained so gauge_burn can re-run the kernels after the
 * parser buffer holding the FRAME payload has been reused. */
static uint8_t s_frame[GK_IMG_N];
static uint8_t s_blur[GK_IMG_N];
```
- In `gauge_process`, remove the local `static uint8_t s_blur[...]`, copy the frame
  first, and feed the kernels from the copy:
```c
    for (int i = 0; i < GK_IMG_N; i++)   /* keep for gauge_burn */
        s_frame[i] = pixels[i];

    gk_blur3x3(s_frame, s_blur, GK_IMG_W, GK_IMG_H);
```
  (rest of the function unchanged).
- New function:
```c
void gauge_burn(gauge_t *g, const params_t *p)
{
    int32_t sum = 0;

    /* Exactly the fabric kernels of gauge_process on the last frame,
     * recomputing the same scores; argmax/EMA/calibration (control
     * core) are skipped and no state or counter changes. */
    gk_blur3x3(s_frame, s_blur, GK_IMG_W, GK_IMG_H);
    gk_pixel_sum(s_blur, GK_IMG_N, &sum);
    gk_ray_scores(s_blur, gk_ray_idx, GK_ANGLES, GK_RADII, sum >> 12,
                  p->polarity ? 1 : -1, g->scores);
}
```

**main.c** — same superloop change as Hum Sentinel, calling
`gauge_burn(&s_gauge, &s_params)`; the frame message is 4 KB so the FIFO comment
applies identically.

### C. Protocol mirrors

- `E1-Hum-Sentinel/host/e1proto.py`: `PARAM_BURN = 9` (after `PARAM_RELEARN`).
- `E1-Meter-Reader/host/e1proto.py`: `PARAM_BURN = 10` (after `PARAM_RESET`).

### D. power_monitor.py (edit BOTH copies identically)

1. **Rolling average** (single samples wobble; burn measurements need a stable read):
   - `import collections`; module constant `AVG_WINDOW_S = 10.0`.
   - In `_Monitor.__init__`: `self._hist = collections.deque()` holding `(t, mw_dict)`.
   - `_publish` appends `(time.monotonic(), mw)` and prunes entries older than
     `AVG_WINDOW_S` (under the lock).
   - New method `avg_mw(self) -> dict[str, float]`: per-rail mean over the deque
     (empty dict if no samples).
2. **Energy-based AA model** next to `AA_CAPACITY_MAH`:
```python
# usable energy of one AA: 2500 mAh at ~1.2 V average under light load.
# Energy-based (mWh / mW) so the projection doesn't mix voltage domains
# the way mAh / rail-mA does.
AA_ENERGY_MWH = 3000.0
```
3. **Duty-cycle projection helpers**:
```python
def duty_avg_mw(workload_mw: float, runtime_s: float, period_h: float,
                sleep_mw: float = 0.0) -> float:
    """Average power of a duty-cycled workload: workload_mw for
    runtime_s out of every period_h hours, sleep_mw in between."""
    period_s = max(period_h * 3600.0, 1e-9)
    duty = min(runtime_s / period_s, 1.0)
    return workload_mw * duty + sleep_mw * (1.0 - duty)


def duty_battery_hours(workload_mw: float, runtime_s: float, period_h: float,
                       sleep_mw: float = 0.0) -> float:
    """Projected runtime on 1x AA (AA_ENERGY_MWH) for the duty-cycled workload."""
    return AA_ENERGY_MWH / max(duty_avg_mw(workload_mw, runtime_s,
                                           period_h, sleep_mw), 1e-9)


def fmt_mw(mw: float) -> str:
    return f"{mw * 1000:.0f} uW" if mw < 1.0 else f"{mw:.1f} mW"
```
4. **fmt_runtime** — add a shelf-life cap as the first branch (self-discharge wins
   long before the arithmetic does):
```python
    if hours >= 10 * 24 * 365:
        return ">10 years (shelf life)"
```
5. **CLI** (standalone `main()`): add
   - `--workload-mw` (float, default None → use the live 10 s chip average, so with
     burn ON the measured constant-workload draw fills the blank automatically),
   - `--workload-runtime` (float seconds, default 2.0),
   - `--workload-period` (float hours, default 1.0),
   - `--sleep-mw` (float, default 0.0).
   Print the projection as a second line, e.g.:
```
SYS  83.60mW  1V8   4.10mW  VDDIO   3.40mW  VDDVAR   1.00mW | chip 3.40mW (10s avg 3.38)
  workload 3.4 mW x 2s every 1h (+0 uW sleep) -> avg 1.9 uW -> 1x AA >10 years (shelf life)
```

### E. Viewers

Both viewers get:
- CLI flags identical to power_monitor's four (`--workload-mw --workload-runtime
  --workload-period --sleep-mw`, same defaults/semantics).
- `self.burn = False` in `App.__init__`; key **`b`** in `handle_key` toggles it and
  sends `send_param(e1proto.PARAM_BURN, 1 if self.burn else 0)`.
- **Streaming pause while burn is on** (the FIFO constraint above):
  - `sentinel_viewer.py` `run()`: add `and not self.burn` to the `start_chunk` gate.
  - `meter_viewer.py` main loop: change the frame-send `else:` to
    `elif not self.burn:`.
- Power panel:
  - chip headline uses `self.power.avg_mw()` (label it `(10s avg)`), rails stay
    instantaneous;
  - a `[BURN — constant workload, streaming paused]` tag (red) when burn is on;
  - a projection line:
    `workload {fmt_mw(w)} x {runtime:g}s every {period:g}h -> avg {fmt_mw(avg)} -> 1x AA {fmt_runtime(h)}`
    where `w = args.workload_mw if set else chip 10s avg`. In `meter_viewer.py` use
    the compact form (`{fmt_mw(w)} x {rt:g}s/{per:g}h -> {fmt_runtime(h)}`) — the
    block is only ~384 px wide.
  - Hum: rails line y0+24, chip/continuous-AA line y0+52, projection y0+80.
  - Meter: title pyy, rails pyy+24 (0.45), chip pyy+44 (0.5), projection pyy+64 (0.45).
- Update the on-screen keys help line and the module docstring key list to include
  `b: burn (constant-workload power soak)`. In `meter_viewer.py` also `flash()` a
  message on toggle, matching the other keys.
- Import the new helpers from power_monitor
  (`duty_avg_mw, duty_battery_hours, fmt_mw`).

### F. Tests

- `E1-Hum-Sentinel/tests/test_dsp.c` `test_params()`: add
  `CHECK_EQ(params_set(&PR, PARAM_BURN, 1), 0);` and
  `CHECK_EQ(params_set(&PR, PARAM_BURN, 2), -1);`
- `E1-Meter-Reader/tests/test_gauge.c` `test_params()`: same pattern
  (`PARAM_BURN`, values 1 ok / 2 rejected), matching that file's `CHECK` macro style.
- Both `tests/e2e_check.py`: add a burn check — set `PARAM_BURN=1` (expect ACK),
  issue a `GET_STATUS` and confirm a STATUS still comes back while the firmware is
  burning (proves the poll stays responsive), then `PARAM_BURN=0` (expect ACK).
  Follow each file's existing `check(...)` helper style.

### G. READMEs (both demos)

Add a short "Burn mode (constant-workload power measurement)" section:
- why (EC feedback: duty-cycled average vs constant workload; quote the ~4.8 mW fft4k
  / ~9 mW conv3x3 reference numbers),
- how (`b` key or `SET_PARAM burn=1`; streaming pauses; firmware loops the fabric
  pipeline at ~99% duty),
- the measurement procedure below,
- the new projection flags, with the framing EC recommended: quote the demo's ~2.6/2.7
  mW as "average power at this demo's duty cycle" and the burn number as "constant
  workload".

Measurement procedure (30+ s per reading, use the 10 s average display):
1. **Idle floor** — firmware flashed, viewer connected but streaming paused (or burn
   off, no audio): VDDIO = P_idle.
2. **Normal demo** — the existing duty-cycled average (~2.6 / 2.7 mW).
3. **Burn** — constant workload; compare against EC's 4.8 mW (fft-type) / 9 mW
   (conv-type) benchmarks.
Derived: energy per chunk ≈ (P_burn − P_idle) × 2.08 ms (Hum) / 0.76 ms (Meter);
the duty-cycle projection then gives honest battery life for any deployment cadence.

## Verification

1. `python -m py_compile` every touched host file in both demos.
2. Hum Sentinel: `make test` (runs 369 C unit checks + protocol/detector e2e against
   the real sim binary). Meter Reader: `make test`. These need gcc/make (POSIX sim
   uses sockets/poll) — on this Windows machine run them under WSL or Git Bash with
   gcc available; if no toolchain exists, say so explicitly in the summary rather
   than skipping silently.
3. Manual sim smoke test (per demo): build the sim, run the viewer with `--source
   synth` (Hum) / `--source synth` (Meter), press `b`, confirm: streaming pauses,
   BURN tag appears, `b` again resumes streaming and the score/reading picks back up,
   and the detector baseline was NOT corrupted (no event storm after resume).
4. Confirm the two power_monitor.py files are still byte-identical
   (`git diff --no-index` or `fc`).
5. Do not commit unless asked.

## Gotchas recap (the things that will bite)

- ONE burn iteration per poll; never batch iterations — 16-byte rx FIFO.
- Host must not stream chunks/frames while burn is on (viewer gates this; the
  firmware cannot protect itself).
- Burn must skip all state updates (baseline EMA, hysteresis, EMA smoother, frame
  counters); it may freely overwrite spec/excess/scores because it recomputes
  identical values from the retained input.
- Meter Reader: the FRAME payload pointer is transient — burn must use the retained
  `s_frame` copy, not the parser buffer.
- Hum PARAM_BURN = 9, Meter PARAM_BURN = 10 (Meter's 9 is already PARAM_RESET).
- Keep both power_monitor.py copies identical.
- `params_set` returning 0 must actually store the value; the ACK carries the param
  id in the seq slot (existing convention — don't change it).
