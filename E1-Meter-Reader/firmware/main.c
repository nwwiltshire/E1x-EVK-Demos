/*
 * main.c — firmware superloop: rx camera frame -> gauge reader -> tx
 * reading.
 *
 * Portable C99 on top of hal.h only.  No threads, no signals, no
 * stdio, no allocation after init.  Identical source for the Linux
 * simulator (make sim) and the E1 target (make e1).
 */
#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "protocol.h"
#include "gauge.h"

/* gauge geometry and wire format must agree */
typedef char assert_frame_size[(GK_IMG_N == PROTO_FRAME_BYTES &&
                                GK_ANGLES == PROTO_N_ANGLES) ? 1 : -1];

#define RX_STALL_RESET_MS 500 /* drop a half-received message after this */

static proto_parser_t s_parser;
static uint8_t        s_tx[PROTO_MAX_MSG];
static uint8_t        s_rx[512];
static params_t       s_params;
static gauge_t        s_gauge; /* frame + score buffers; keep static */

static uint32_t s_last_us;

static void send_status(void)
{
    uint8_t flags = 0;
    int n;

    if (s_gauge.confidence >= (uint32_t)s_params.conf_min)
        flags |= PROTO_FLAG_NEEDLE;
    n = proto_build_status(s_tx, s_gauge.angle_cdeg, s_gauge.value_milli,
                           s_gauge.confidence, flags,
                           (uint8_t)s_params.mode, s_last_us,
                           s_gauge.frames, s_gauge.mean);
    hal_serial_write(s_tx, n);
}

static void send_ack(uint8_t seq, uint8_t ok)
{
    int n = proto_build_ack(s_tx, seq, ok);
    hal_serial_write(s_tx, n);
}

static void handle_frame(const proto_msg_t *msg)
{
    uint32_t t0;

    if (msg->len != PROTO_FRAME_BYTES) {
        send_ack(msg->seq, 0);
        return;
    }

    t0 = hal_micros();
    gauge_process(&s_gauge, &s_params, msg->payload);
    s_last_us = hal_micros() - t0;

    if (s_params.mode == MODE_DEV) {
        int n = proto_build_scores(s_tx, msg->seq, s_gauge.scores);
        hal_serial_write(s_tx, n);
    }

    send_status();

    /* ACK last: it is the host's licence to send the next frame, so it
     * must not overtake the SCORES — the link is only clean
     * half-duplex (the EVK USB bridge drops one direction while the
     * other is busy). */
    send_ack(msg->seq, 1);
}

static void handle_set_param(const proto_msg_t *msg)
{
    uint8_t ok;

    if (msg->param_id == PARAM_RESET) {
        gauge_reset(&s_gauge);
        ok = 1;
    } else {
        ok = params_set(&s_params, msg->param_id, msg->value) == 0;
    }
    /* the ACK carries the param id in the seq slot */
    send_ack(msg->param_id, ok);
}

int main(void)
{
    proto_msg_t msg;
    uint32_t last_rx_ms = 0;

    hal_init();
    proto_init();
    proto_parser_reset(&s_parser);
    params_defaults(&s_params);
    gauge_init(&s_gauge);

    for (;;) {
        uint32_t now = hal_millis();
        int n;

        if (hal_button_pressed())
            s_params.mode = (s_params.mode == MODE_DEV) ? MODE_DEPLOY : MODE_DEV;

        /* the link went quiet mid-message: resync */
        if (proto_parser_in_msg(&s_parser) && (now - last_rx_ms) > RX_STALL_RESET_MS)
            proto_parser_reset(&s_parser);

        /* Burn mode: saturate the fabric so VDDIO shows the constant-
         * workload draw, not a tiny duty-cycle average.  ONE iteration
         * per serial poll: the EVK rx FIFO is only 16 bytes deep, so
         * anything bigger than a SET_PARAM (10 bytes) sent
         * mid-iteration would overflow it — the host stops streaming
         * frames while burn is on and only params arrive. */
        if (s_params.burn)
            gauge_burn(&s_gauge, &s_params);

        n = hal_serial_read(s_rx, (int)sizeof s_rx, s_params.burn ? 0 : 20);
        if (n <= 0)
            continue;
        last_rx_ms = hal_millis();

        for (int i = 0; i < n; i++) {
            int r = proto_parser_push(&s_parser, s_rx[i], &msg);

            if (r == 0)
                continue;
            if (r < 0) { /* CRC mismatch: NAK so the host resends */
                send_ack(msg.seq, 0);
                continue;
            }
            switch (msg.type) {
            case PROTO_FRAME:
                handle_frame(&msg);
                break;
            case PROTO_SET_PARAM:
                handle_set_param(&msg);
                break;
            case PROTO_GET_STATUS:
                send_status();
                break;
            default: /* fw->host types echoed at us: ignore */
                break;
            }
        }
    }
}
