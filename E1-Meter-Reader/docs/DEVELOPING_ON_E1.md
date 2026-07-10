# Developing applications for the E1 EVK — field notes

Everything in this file was learned the hard way while building two
working demos (a webcam people counter and this acoustic sentinel) on
the Electron E1 EVK, SDK 25.4.  It is the checklist to start from
next time.

## Toolchain and board

- The Efficient SDK lives at `~/effcc`: the `effcc` compiler,
  `eff-flash`, and — most importantly — the **full SDK source** under
  `~/effcc/sdk`.  When an API's behavior is unclear, read the driver
  source (`sdk/drivers/`, `sdk/stdlib/`, `sdk/include/eff/`) instead
  of guessing; it has settled every question so far (UART FIFO
  depths, blocking semantics, timer backends, baud dividers).
- Board switches that matter: `SW1` = USB power, `SW2` = ON, `SW9` ON
  (routes the E1's UART3 to the USB bridge), `SW11` position 1 ON
  (user button SW12).  BOOT switches: `101` = boot from SRAM, `010` =
  boot from MRAM.
- Flash: `~/effcc/bin/eff-flash app.hex sram` (finds the kit on
  ttyACM0 by itself).  **SRAM is volatile** — reflash after every
  power cycle; use `mram` + BOOT `010` for anything that must survive
  one.  Flashing reboots the chip (all firmware state resets).
- USB serial ports (also `/dev/eff-*` if effcc's udev rule is
  installed):

  | Port | Role |
  |---|---|
  | ttyACM0 | programmer (eff-flash) |
  | ttyACM1 | power telemetry CSV |
  | ttyACM2 | E1x stdio = your application's UART3 link |

- Power telemetry: ttyACM1 streams CSV rows of *measured* mA/mV/mW —
  `timestamp(us)` then triples for SYS, 1V8, VDDIO, VDDVAR.  It
  looks dead to `cat`: **the stream starts only when the port is
  opened with DTR asserted** (pyserial's default does this).  Rail
  meanings: SYS = whole board, VDDIO = the E1x chip alone, VDDVAR =
  scalar core + fabric.  Host code should poll a few times per
  second and parse only the newest line (see below).

## The fabric: rules that actually matter

The single most important fact: **effcc only puts a function on the
dataflow fabric if it is marked `__efficient__`, and forgetting the
keyword produces no error** — the SDK compiles `-D__efficient__=`
(empty) for scalar/native targets, so an unannotated "fabric" build
runs 100 % on the RISC-V control core and silently benchmarks
identical to the scalar build.  Always verify placement with a timer
(`uptime_us()` around the hot call, reported over the link).

Recipe for fabric-friendly functions (mirrors
`e1x_examples/app_examples/{jpeg,ldpc,biquad_filter}`):

- Annotate the **definition** with `__efficient__`.
- Plain `for` loops over contiguous arrays; nested loops with
  variable bounds are fine (this project's whole radix-2 FFT is one
  `__efficient__` function).
- `restrict` on every pointer parameter; callers pass
  non-overlapping buffers.
- **No libc calls** — write `for (i…) dst[i] = 0;` instead of
  `memset`.
- Pass const tables (windows, twiddles, index tables) in as pointer
  arguments rather than referencing globals.
- Prefer fixed control structure over data-dependent loops (e.g. a
  5-step binary-search log2 instead of `while (v >>= 1)`).
- Table-driven gathers (`dst[i] = src[table[i]]`) work.
- Un-annotated helper functions *called from* fabric functions are
  fine (ldpc does this).
- Keep branchy/stateful/sequential logic (event machines, union-find,
  argmax over small arrays) on the control core — it is the wrong
  shape for spatial hardware and small enough not to matter.

Guard the keyword so the same sources build with gcc for simulation
and unit tests:

```c
#if !defined(EFF_BLD_FABRIC) && !defined(__efficient__)
#define __efficient__
#endif
```

CMake skeleton (mirrors the SDK examples):

```cmake
set(EFF_STDIO_PORT 3)
file(REAL_PATH "$ENV{HOME}/effcc/sdk" EFF_SDK_ROOT_DIR)
include(${EFF_SDK_ROOT_DIR}/setup_sdk.cmake)
add_subdirectory(${EFF_SDK_ROOT_DIR} sdk)
add_eff_app(myapp TYPE exe ARCHS e1x TARGETS fabric scalar SOURCE ...)
eff_subtarget_compile_options(myapp_fabric PRIVATE -DE1_TARGET)
```

Build both `fabric` and `scalar` targets and *measure* them against
each other — the ratio proves the annotation took (this project:
2.08 ms vs 8.26 ms per chunk).  Expect the fabric .hex to grow by
tens of KB (the fabric bitstream).

## The serial link: physics you cannot negotiate with

Measured, not documented anywhere:

- The EVK's USB bridge runs its UART side at a **fixed ~115200**.
  Host-side CDC baud requests have no physical effect (proved:
  byte-identical garbage at two different requested rates).  The
  E1-side divider is `uart_clock / (baud × 18)` with a 7,813,120 Hz
  clock, so a 115200 request actually yields 108,516 baud — which is
  what the bridge speaks.  **Request 115200 on both sides and treat
  ~10.8 KB/s each way as the budget.**
- The link is **only clean half-duplex**: while one direction is
  busy, the bridge silently drops contiguous runs of bytes from the
  other (values never corrupt; whole spans vanish).  Protocol design
  rules that follow:
  1. Strict request/response, one message in flight, ever.
  2. The device sends its ACK **last**, after all reply data — the
     ACK is the host's licence to transmit again.
  3. Frame everything (magic + CRC16) and resync by scanning for the
     magic; NAK on bad CRC so the host can retry.
  4. Add a firmware-side stall reset (drop a half-received message
     after ~500 ms of line silence).
- The E1 UART FIFOs are 16 bytes; `eff_uart_getc()` is non-blocking
  (check `eff_uart_rx_empty()` first), `eff_uart_putc()` blocks when
  the TX FIFO is full.  Drain RX hot in the read loop.
- The SDK claims UART3 for stdio before `main()`; re-init it in your
  own init and **never printf in the firmware core** — a stray
  runtime print (e.g. `sleep_us()` can emit a WARN) lands in your
  binary stream.  Framing + resync makes this survivable.
- First hardware step every time: flash a **gap-echo** app (buffer
  RX, echo it back after ~50 ms of line silence — a live echo is
  full-duplex and unmeasurable here) and prove the link byte-exact
  before any application traffic.

## A development workflow that worked twice

1. **Portable core + tiny HAL.**  Application logic in plain C99
   against a ~6-function HAL (`serial_read/write`, `millis/micros`,
   `button`).  Two backends: POSIX (the "serial port" is a loopback
   TCP socket) and E1.  The firmware then runs as a Linux process
   and the entire system is testable with zero hardware.
2. **Unit tests with reference implementations.**  gcc + the guard
   macro lets the exact fabric sources run on the host.  Test DSP
   against slow-but-obviously-correct references (this project: the
   int16 FFT against a double-precision DFT, bin by bin).
3. **End-to-end against the sim binary.**  A Python harness spawns
   the firmware process and drives the real wire protocol over TCP —
   protocol edge cases (CRC corruption, resync, param validation)
   and behavior scenarios, CI-friendly.
4. **`hw_smoke.py` acceptance test.**  The same scenarios over the
   real serial link, ~1 minute, printed PASS/FAIL plus timing.  Run
   it after every flash.  Make assertions robust to a warm device
   (counters persist until reflash/power-cycle).
5. **Only then the GUI.**

Report on-chip compute time in the status message from day one
(`uptime_us()` truncated to u32) — it is how you catch a
mis-annotated fabric build immediately.

## Host-side (Python/pyserial) lessons

- **Never write to the serial port from the GUI thread.**  A write
  can block for seconds when the half-duplex bridge holds the line;
  the window manager then declares the app "not responding".  Put
  writes on a dedicated thread fed by a queue, and resolve ACKs by
  polling a queue, never by blocking waits in the render loop.
- **One in-flight message state machine** with retry-then-drop keeps
  a demo alive through any link hiccup.  Timeouts must reflect the
  link: replies legitimately take 150–200 ms per KB round trip.
- **The GIL is part of your link budget.**  A sibling thread parsing
  ~200 telemetry lines/s cut chunk throughput in half; polling 5×/s
  and parsing only the newest line fixed it.  Similarly, a render
  loop without a sleeping `waitKey` starves the reader thread.
- For live audio/video sources, keep a drop-oldest ring and always
  send the newest data — a slower-than-realtime link then costs
  coverage, not latency.

## Known open items / quirks

- The user-button mapping (SW12 → GPIO_11 pin 4) is inferred from
  the quickstart's LED example, not verified against the schematic.
  Misreads are harmless (reads as released); the protocol mode
  toggle is the reliable path.
- `sleep_us()` in `sdk/stdlib/time.c` can print
  `WARN: unexpected wait_for_wake()` onto stdio (= your link).  The
  parser's resync handles it; don't let it surprise you in byte
  traces.
- Baud override plumbing (`-DE1_LINK_BAUD`) exists in this project's
  CMake but is moot through the USB bridge; it would only matter for
  direct wiring to the UART header.
