/*
 * echo_main.c — frame-link echo test, step 1 of hardware bring-up.
 *
 * Echoes every received byte straight back.  Flash this before the
 * people counter to prove the UART path (port, baud, byte integrity,
 * throughput) with host/echo_test.py — it reports effective KB/s and
 * the frame rate that implies.  Also builds against the simulator
 * (make sim-echo) so the host tool itself can be sanity-checked
 * without hardware.
 *
 * The E1 build is a GAP-ECHO, not a byte-by-byte echo.  A live echo
 * is full-duplex, and the EVK link path is not full-duplex clean
 * (measured: ~255 bytes survive, then contiguous runs are dropped —
 * the USB bridge discards traffic in one direction while busy with
 * the other, and the 16-byte on-chip FIFOs add their own overrun
 * window while hal_serial_write() blocks).  The real firmware never
 * hits any of that: the protocol is half-duplex by design — the host
 * sends nothing until the E1 finishes replying (ACK-after-processing,
 * one frame in flight), and the E1 sends nothing until a frame has
 * fully arrived.  So the echo mirrors that exact temporal pattern:
 * buffer everything that arrives, and only when the line has been
 * quiet for ECHO_GAP_MS echo the whole burst back.  Byte count and
 * byte values then measure each direction separately.
 */
#include <stdint.h>

#include "hal.h"

#ifdef E1_TARGET

#include <eff.h>

#define ECHO_UART UART_3 /* = E1_LINK_UART in hal_e1.c */
#define ECHO_GAP_MS 50u  /* silence that marks the end of a burst */
#define BUF_SIZE 32768u  /* holds a full 30,000-byte test frame */

static uint8_t buf[BUF_SIZE];

int main(void)
{
    uint32_t n = 0, last_rx = 0;

    hal_init(); /* sets the link baud (E1_LINK_BAUD) */
    for (;;) {
        while (!eff_uart_rx_empty(ECHO_UART)) {
            char c;

            eff_uart_getc(ECHO_UART, &c);
            if (n < BUF_SIZE)
                buf[n++] = (uint8_t)c;
            last_rx = hal_millis();
        }
        if (n > 0 && (uint32_t)(hal_millis() - last_rx) >= ECHO_GAP_MS) {
            /* burst over, line quiet: safe to transmit (half-duplex) */
            hal_serial_write(buf, (int)n);
            n = 0;
        }
    }
}

#else /* simulator: no shallow FIFOs, the plain HAL loop is fine */

int main(void)
{
    static uint8_t buf[512];

    hal_init();
    for (;;) {
        int n = hal_serial_read(buf, (int)sizeof buf, 100);

        if (n > 0)
            hal_serial_write(buf, n);
    }
}

#endif
