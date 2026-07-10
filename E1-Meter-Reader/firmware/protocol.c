#include "protocol.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection)               */
/* ------------------------------------------------------------------ */

static uint16_t s_crc_table[256];
static int s_crc_ready;

void proto_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t c = (uint16_t)(i << 8);
        for (int b = 0; b < 8; b++)
            c = (uint16_t)((c & 0x8000u) ? (uint16_t)(c << 1) ^ 0x1021u
                                         : (uint16_t)(c << 1));
        s_crc_table[i] = c;
    }
    s_crc_ready = 1;
}

uint16_t proto_crc16(uint16_t crc, const uint8_t *data, uint32_t len)
{
    if (!s_crc_ready)
        proto_init();
    for (uint32_t i = 0; i < len; i++)
        crc = (uint16_t)((uint16_t)(crc << 8) ^
                         s_crc_table[((crc >> 8) ^ data[i]) & 0xFFu]);
    return crc;
}

/* ------------------------------------------------------------------ */
/* Incremental parser                                                  */
/* ------------------------------------------------------------------ */

enum { PS_MAGIC0, PS_MAGIC1, PS_TYPE, PS_BODY, PS_CRC0, PS_CRC1 };

void proto_parser_reset(proto_parser_t *p)
{
    p->state = PS_MAGIC0;
    p->stage = 0;
    p->have = 0;
    p->need = 0;
    p->crc = 0xFFFF;
    p->rx_crc = 0;
    p->type = 0;
}

int proto_parser_in_msg(const proto_parser_t *p)
{
    return p->state != PS_MAGIC0;
}

/* Body bytes expected immediately after the type byte; -1 = unknown type. */
static int initial_need(uint8_t type)
{
    switch (type) {
    case PROTO_FRAME:      return 3;  /* seq + len16, then len more */
    case PROTO_SET_PARAM:  return 5;  /* param_id + i32 */
    case PROTO_GET_STATUS: return 0;
    case PROTO_ACK:        return 2;  /* seq + ok */
    case PROTO_STATUS:     return 16; /* angle16 value32 conf flags mode us32 frames16 mean */
    case PROTO_SCORES:     return 3;  /* seq + len16, then len more */
    default:               return -1;
    }
}

/* Called whenever have == need: extends need for variable-length parts.
 * Returns 1 = body complete, 0 = need extended, -1 = invalid (resync). */
static int advance(proto_parser_t *p)
{
    uint16_t len, cap;

    switch (p->type) {
    case PROTO_FRAME:
    case PROTO_SCORES:
        if (p->stage == 0) {
            cap = (p->type == PROTO_FRAME) ? PROTO_FRAME_BYTES
                                           : PROTO_SCORE_BYTES;
            len = (uint16_t)(p->body[1] | ((uint16_t)p->body[2] << 8));
            if (len == 0 || len > cap)
                return -1;
            p->stage = 1;
            p->need += len;
            return 0;
        }
        return 1;
    default:
        return 1;
    }
}

static int settle(proto_parser_t *p)
{
    while (p->have == p->need) {
        int a = advance(p);
        if (a < 0) {
            proto_parser_reset(p);
            return 0;
        }
        if (a == 1) {
            p->state = PS_CRC0;
            return 0;
        }
    }
    return 0;
}

static void fill_out(const proto_parser_t *p, proto_msg_t *out)
{
    memset(out, 0, sizeof *out);
    out->type = p->type;

    switch (p->type) {
    case PROTO_FRAME:
    case PROTO_SCORES:
        out->seq = p->body[0];
        out->len = (uint16_t)(p->body[1] | ((uint16_t)p->body[2] << 8));
        out->payload = p->body + 3;
        break;
    case PROTO_SET_PARAM:
        out->param_id = p->body[0];
        out->value = (int32_t)((uint32_t)p->body[1] |
                               ((uint32_t)p->body[2] << 8) |
                               ((uint32_t)p->body[3] << 16) |
                               ((uint32_t)p->body[4] << 24));
        break;
    case PROTO_ACK:
        out->seq = p->body[0];
        out->ok = p->body[1];
        break;
    case PROTO_STATUS:
        out->angle_cdeg = (int16_t)(uint16_t)(p->body[0] |
                                              ((uint16_t)p->body[1] << 8));
        out->value_milli = (int32_t)((uint32_t)p->body[2] |
                                     ((uint32_t)p->body[3] << 8) |
                                     ((uint32_t)p->body[4] << 16) |
                                     ((uint32_t)p->body[5] << 24));
        out->confidence = p->body[6];
        out->flags = p->body[7];
        out->mode = p->body[8];
        out->frame_us = (uint32_t)p->body[9] |
                        ((uint32_t)p->body[10] << 8) |
                        ((uint32_t)p->body[11] << 16) |
                        ((uint32_t)p->body[12] << 24);
        out->frames = (uint16_t)(p->body[13] | ((uint16_t)p->body[14] << 8));
        out->mean = p->body[15];
        break;
    default:
        break;
    }
}

int proto_parser_push(proto_parser_t *p, uint8_t byte, proto_msg_t *out)
{
    switch (p->state) {
    case PS_MAGIC0:
        if (byte == PROTO_MAGIC0)
            p->state = PS_MAGIC1;
        return 0;

    case PS_MAGIC1:
        if (byte == PROTO_MAGIC1)
            p->state = PS_TYPE;
        else if (byte != PROTO_MAGIC0) /* AA AA 55 still syncs */
            p->state = PS_MAGIC0;
        return 0;

    case PS_TYPE: {
        int need = initial_need(byte);
        if (need < 0) { /* unknown type: resync (byte may itself be 0xAA) */
            p->state = (byte == PROTO_MAGIC0) ? PS_MAGIC1 : PS_MAGIC0;
            return 0;
        }
        p->type = byte;
        p->crc = proto_crc16(0xFFFF, &byte, 1);
        p->have = 0;
        p->need = (uint32_t)need;
        p->stage = 0;
        p->state = PS_BODY;
        return settle(p); /* zero-length body goes straight to CRC */
    }

    case PS_BODY:
        p->body[p->have++] = byte;
        p->crc = proto_crc16(p->crc, &byte, 1);
        if (p->have == p->need)
            return settle(p);
        return 0;

    case PS_CRC0:
        p->rx_crc = byte;
        p->state = PS_CRC1;
        return 0;

    case PS_CRC1: {
        int ok;
        p->rx_crc |= (uint16_t)((uint16_t)byte << 8);
        ok = (p->rx_crc == p->crc);
        fill_out(p, out); /* body buffer stays valid until the next push */
        proto_parser_reset(p);
        return ok ? 1 : -1;
    }

    default:
        proto_parser_reset(p);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Builders                                                            */
/* ------------------------------------------------------------------ */

/* dst[2..2+body_len) already holds the body (starting with the type
 * byte); adds magic + CRC and returns the total message length. */
static int wrap(uint8_t *dst, int body_len)
{
    uint16_t crc;

    dst[0] = PROTO_MAGIC0;
    dst[1] = PROTO_MAGIC1;
    crc = proto_crc16(0xFFFF, dst + 2, (uint32_t)body_len);
    dst[2 + body_len] = (uint8_t)(crc & 0xFF);
    dst[3 + body_len] = (uint8_t)(crc >> 8);
    return body_len + 4;
}

int proto_build_frame(uint8_t *dst, uint8_t seq, const uint8_t *pixels, uint16_t len)
{
    uint8_t *b = dst + 2;

    b[0] = PROTO_FRAME;
    b[1] = seq;
    b[2] = (uint8_t)(len & 0xFF);
    b[3] = (uint8_t)(len >> 8);
    memcpy(b + 4, pixels, len);
    return wrap(dst, 4 + (int)len);
}

int proto_build_set_param(uint8_t *dst, uint8_t param_id, int32_t value)
{
    uint8_t *b = dst + 2;
    uint32_t v = (uint32_t)value;

    b[0] = PROTO_SET_PARAM;
    b[1] = param_id;
    b[2] = (uint8_t)(v & 0xFF);
    b[3] = (uint8_t)((v >> 8) & 0xFF);
    b[4] = (uint8_t)((v >> 16) & 0xFF);
    b[5] = (uint8_t)((v >> 24) & 0xFF);
    return wrap(dst, 6);
}

int proto_build_get_status(uint8_t *dst)
{
    dst[2] = PROTO_GET_STATUS;
    return wrap(dst, 1);
}

int proto_build_ack(uint8_t *dst, uint8_t seq, uint8_t ok)
{
    uint8_t *b = dst + 2;

    b[0] = PROTO_ACK;
    b[1] = seq;
    b[2] = ok;
    return wrap(dst, 3);
}

int proto_build_status(uint8_t *dst, int16_t angle_cdeg, int32_t value_milli,
                       uint8_t confidence, uint8_t flags, uint8_t mode,
                       uint32_t frame_us, uint16_t frames, uint8_t mean)
{
    uint8_t *b = dst + 2;
    uint16_t a = (uint16_t)angle_cdeg;
    uint32_t v = (uint32_t)value_milli;

    b[0] = PROTO_STATUS;
    b[1] = (uint8_t)(a & 0xFF);
    b[2] = (uint8_t)(a >> 8);
    b[3] = (uint8_t)(v & 0xFF);
    b[4] = (uint8_t)((v >> 8) & 0xFF);
    b[5] = (uint8_t)((v >> 16) & 0xFF);
    b[6] = (uint8_t)((v >> 24) & 0xFF);
    b[7] = confidence;
    b[8] = flags;
    b[9] = mode;
    b[10] = (uint8_t)(frame_us & 0xFF);
    b[11] = (uint8_t)((frame_us >> 8) & 0xFF);
    b[12] = (uint8_t)((frame_us >> 16) & 0xFF);
    b[13] = (uint8_t)((frame_us >> 24) & 0xFF);
    b[14] = (uint8_t)(frames & 0xFF);
    b[15] = (uint8_t)(frames >> 8);
    b[16] = mean;
    return wrap(dst, 17);
}

int proto_build_scores(uint8_t *dst, uint8_t seq, const uint16_t *scores)
{
    uint8_t *b = dst + 2;

    b[0] = PROTO_SCORES;
    b[1] = seq;
    b[2] = (uint8_t)(PROTO_SCORE_BYTES & 0xFF);
    b[3] = (uint8_t)(PROTO_SCORE_BYTES >> 8);
    for (int i = 0; i < PROTO_N_ANGLES; i++) {
        b[4 + 2 * i] = (uint8_t)(scores[i] & 0xFF);
        b[5 + 2 * i] = (uint8_t)(scores[i] >> 8);
    }
    return wrap(dst, 4 + PROTO_SCORE_BYTES);
}
