/*
 * hal_e1.c — Electron E1 (EVK) backend.
 *
 * Frame link: UART_3, the UART the SDK normally uses for stdio on the
 * EVK — it reaches the laptop through the USB programmer bridge (EVK
 * switch bank SW9), so no extra wiring.  The SDK's stdio constructor
 * claims UART_3 at 115200 before main(); hal_init() re-inits it with
 * our settings.  There is deliberately no printf anywhere in the
 * firmware core, so nothing corrupts the binary stream (and if the
 * runtime ever prints, the host parser just resyncs on AA 55).
 *
 * Baud: 115200 is the only rate that works, measured on the EVK
 * (2026-07-09).  The USB bridge runs its UART side at a fixed
 * ~115200 regardless of the host's CDC line-coding request, and the
 * E1 divider (uart_clock / (baud * 18), 7,813,120 Hz clock) turns a
 * 115200 request into an actual 108,516 baud — which matches what
 * the bridge produces, since 30KB bursts round-trip byte-exact.
 * Requesting any other rate on either side corrupts bytes.  The link
 * is also only clean HALF-duplex: the bridge drops one direction
 * while the other is busy, which is why main.c sends the ACK last
 * and the host never transmits while a reply is in flight.
 *
 * Board setup (see README "Going to hardware"): J16 USB, SW1=USB
 * power, SW2=ON, SW9 ON, SW11 position 1 ON (user button SW12 ->
 * DIGIO114, active-low).
 *
 * Every SDK call here was checked against the installed SDK sources
 * (~/effcc/sdk, version 25.4): eff_uart_getc() is NON-blocking (check
 * eff_uart_rx_empty() first), eff_uart_putc() blocks on a full FIFO,
 * eff_gpio_get() returns the masked input levels, uptime_ms() is the
 * mtimer-backed time base.
 */
#ifndef E1_TARGET
#error "hal_e1.c is the E1 hardware backend; build it with -DE1_TARGET. The Linux simulator uses hal_posix.c (make sim)."
#endif

#include <eff.h>

#include "hal.h"

/* Frame-link UART and rate.  Override with -DE1_LINK_BAUD=... */
#ifndef E1_LINK_BAUD
#define E1_LINK_BAUD 115200
#endif
#define E1_LINK_UART   UART_3
#define E1_LINK_PINMUX PINMUX_3 /* = STDIO_PINMUX for E1X (uart.h) */

/* User button SW12 = DIGIO114, active-low.  The quickstart example
 * drives the user LEDs as GPIO_11 pins 2/3 (DIGIO112/113), which puts
 * DIGIO114 at GPIO_11 pin 4.  UNVERIFIED against the EVK schematic —
 * if the button does nothing, this mapping is the first suspect (the
 * 'd' key in the viewer toggles the mode over the protocol either
 * way).  A wrong-but-in-mask pin reads as a plain input; a pin outside
 * the bank mask makes eff_gpio_get() return -1, which reads as
 * "released" — both are harmless. */
#define E1_BUTTON_GPIO GPIO_11
#define E1_BUTTON_PIN  GPIO_PIN_4
#define E1_BUTTON_DEBOUNCE_MS 50u

static eff_uart_t *s_link;

void hal_init(void)
{
    eff_uart_cfg_t cfg = EFF_UART_DEFAULTS; /* 8N1 */

    /* frame link */
    eff_pinmux_set(E1_LINK_PINMUX, PINMUX_UART);
    cfg.baud = E1_LINK_BAUD;
    eff_uart_init(E1_LINK_UART, cfg);
    s_link = E1_LINK_UART;

    /* mode-toggle button */
    eff_pinmux_set(PINMUX_11, PINMUX_GPIO);
    eff_gpio_dir_set(E1_BUTTON_GPIO, E1_BUTTON_PIN, EFF_GPIO_IN);
    eff_gpio_pull_set(E1_BUTTON_GPIO, E1_BUTTON_PIN, EFF_GPIO_PULL_UP);
}

int hal_serial_read(uint8_t *buf, int len, int timeout_ms)
{
    uint32_t t0 = hal_millis();
    int n = 0;

    while (n < len) {
        if (!eff_uart_rx_empty(s_link)) {
            char c;

            eff_uart_getc(s_link, &c);
            buf[n++] = (uint8_t)c;
            continue; /* drain hot: the rx FIFO is only 16 bytes deep */
        }
        if (n > 0)
            break; /* got a chunk; let the superloop parse it */
        if ((uint32_t)(hal_millis() - t0) >= (uint32_t)timeout_ms)
            break;
        sleep_us(200); /* idle between frames; don't spin flat out */
    }
    return n;
}

int hal_serial_write(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        eff_uart_putc(s_link, (char)buf[i]); /* blocks on a full FIFO */
    return len;
}

uint32_t hal_millis(void)
{
    return (uint32_t)uptime_ms(); /* wrap at 2^32 is fine: only diffed */
}

uint32_t hal_micros(void)
{
    return (uint32_t)uptime_us(); /* mtimer-backed; wraps at ~71 min */
}

bool hal_button_pressed(void)
{
    static uint8_t prev_level = 1; /* pulled up = released */
    static uint32_t last_edge_ms;
    uint32_t now = hal_millis();
    uint8_t level = eff_gpio_get(E1_BUTTON_GPIO, E1_BUTTON_PIN) ? 1 : 0;

    if (level != prev_level &&
        (uint32_t)(now - last_edge_ms) >= E1_BUTTON_DEBOUNCE_MS) {
        prev_level = level;
        last_edge_ms = now;
        return level == 0; /* 1 -> 0 edge = press (active-low) */
    }
    return false;
}
