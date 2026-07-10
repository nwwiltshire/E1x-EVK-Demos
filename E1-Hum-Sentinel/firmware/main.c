/*
 * main.c — firmware superloop: rx audio chunk -> detector -> tx result.
 *
 * Portable C99 on top of hal.h only.  No threads, no signals, no
 * stdio, no allocation after init.  Identical source for the Linux
 * simulator (make sim) and the E1 target (make e1).
 */
#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "protocol.h"
#include "detector.h"

/* detector and wire format must agree on the chunk geometry */
typedef char assert_chunk_size[(AU_CHUNK == PROTO_AUDIO_BYTES &&
                                3 * AU_VIZ_BINS == PROTO_SPEC_BYTES) ? 1 : -1];

#define RX_STALL_RESET_MS 500 /* drop a half-received message after this */

static proto_parser_t s_parser;
static uint8_t        s_tx[PROTO_MAX_MSG];
static uint8_t        s_rx[512];
static uint8_t        s_viz[PROTO_SPEC_BYTES];
static params_t       s_params;
static detector_t     s_det; /* ~15KB of model + FFT buffers; keep static */

static uint32_t s_last_us;

static void send_status(void)
{
    uint8_t flags = 0;
    int n;

    if (s_det.ev_active)
        flags |= PROTO_FLAG_EVENT;
    if (s_det.learn_left > 0)
        flags |= PROTO_FLAG_LEARNING;
    n = proto_build_status(s_tx, s_det.score, flags, s_det.ev_class,
                           (uint8_t)s_params.mode, detector_learn_pct(&s_det),
                           s_det.top_bin, s_last_us, s_det.events);
    hal_serial_write(s_tx, n);
}

static void send_ack(uint8_t seq, uint8_t ok)
{
    int n = proto_build_ack(s_tx, seq, ok);
    hal_serial_write(s_tx, n);
}

static void handle_audio(const proto_msg_t *msg)
{
    uint32_t t0;

    if (msg->len != AU_CHUNK) {
        send_ack(msg->seq, 0);
        return;
    }

    t0 = hal_micros();
    detector_process(&s_det, &s_params, msg->payload);
    s_last_us = hal_micros() - t0;

    if (s_params.mode == MODE_DEV) {
        detector_viz(&s_det, &s_params, s_viz);
        int n = proto_build_spectrum(s_tx, msg->seq, s_viz);
        hal_serial_write(s_tx, n);
    }

    send_status();

    /* ACK last: it is the host's licence to send the next chunk, so it
     * must not overtake the SPECTRUM — the link is only clean
     * half-duplex (the EVK USB bridge drops one direction while the
     * other is busy). */
    send_ack(msg->seq, 1);
}

static void handle_set_param(const proto_msg_t *msg)
{
    uint8_t ok;

    if (msg->param_id == PARAM_RELEARN) {
        detector_relearn(&s_det, &s_params);
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
    detector_init(&s_det, &s_params);

    for (;;) {
        uint32_t now = hal_millis();
        int n;

        if (hal_button_pressed())
            s_params.mode = (s_params.mode == MODE_DEV) ? MODE_DEPLOY : MODE_DEV;

        /* the link went quiet mid-message: resync */
        if (proto_parser_in_msg(&s_parser) && (now - last_rx_ms) > RX_STALL_RESET_MS)
            proto_parser_reset(&s_parser);

        n = hal_serial_read(s_rx, (int)sizeof s_rx, 20);
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
            case PROTO_AUDIO:
                handle_audio(&msg);
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
