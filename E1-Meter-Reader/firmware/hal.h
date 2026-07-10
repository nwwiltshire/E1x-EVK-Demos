/*
 * hal.h — platform abstraction for the E1 meter-reader firmware.
 *
 * Exactly two backends implement this interface:
 *   hal_posix.c — Linux simulator: "serial" is a loopback TCP socket,
 *                 button is SIGUSR1.  (make sim)
 *   hal_e1.c    — Electron E1 EVK: UART frame link, user button SW12
 *                 (DIGIO114).  (make e1)
 *
 * Everything above this line of the stack (main/gauge/protocol) is
 * portable C99 with no OS assumptions.
 */
#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

void     hal_init(void);

/* Read up to len bytes from the host serial link.  Blocks at most
 * timeout_ms; returns the number of bytes read (0 on timeout). */
int      hal_serial_read(uint8_t *buf, int len, int timeout_ms);

/* Write len bytes to the host serial link (blocking).  Returns bytes
 * written. */
int      hal_serial_write(const uint8_t *buf, int len);

/* Monotonic milliseconds since boot; wraps at 2^32. */
uint32_t hal_millis(void);

/* Monotonic microseconds since boot; wraps at 2^32 (~71 min) — only
 * ever diffed across one frame, so the wrap is harmless. */
uint32_t hal_micros(void);

/* Mode-toggle button, edge-triggered: returns true once per press.
 * EVK: user button SW12 -> DIGIO114 (active-low).  Sim: SIGUSR1. */
bool     hal_button_pressed(void);

#endif /* HAL_H */
