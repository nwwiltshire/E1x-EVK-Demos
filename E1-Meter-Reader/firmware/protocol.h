/*
 * protocol.h — host<->firmware serial protocol for the analog meter
 * reader.
 *
 * Wire format (little-endian), CRC16-CCITT (poly 0x1021, init 0xFFFF)
 * computed over everything after the AA 55 magic:
 *
 *   FRAME      host->fw  AA 55 01 seq:u8 len:u16(=4096) payload(64x64 gray) crc:u16
 *   SET_PARAM  host->fw  AA 55 02 param_id:u8 value:i32 crc:u16
 *   GET_STATUS host->fw  AA 55 03 crc:u16
 *   ACK/NAK    fw->host  AA 55 10 seq:u8 ok:u8 crc:u16
 *   STATUS     fw->host  AA 55 11 angle_cdeg:i16 value_milli:i32 conf:u8
 *                        flags:u8 mode:u8 frame_us:u32 frames:u16 mean:u8 crc:u16
 *   SCORES     fw->host  AA 55 12 seq:u8 len:u16(=480) payload(u16 x 240) crc:u16
 *                        (DEV mode only)
 *
 * Reply order per FRAME: SCORES (DEV) -> STATUS -> ACK.  The ACK comes
 * LAST: it is the host's licence to transmit, and the EVK USB bridge
 * is only clean half-duplex.
 *
 * Angle convention: 0 = 12 o'clock, clockwise positive, centidegrees
 * in [-18000, +18000).  SCORES index ai <-> -18000 + ai*GK_STEP_CDEG.
 *
 * Resync on garbage: scan for AA 55.
 *
 * Pure data-in/data-out: no HAL, no stdio, no allocation — the same
 * code runs in the firmware, the simulator, and the unit tests.
 * host/e1proto.py is the Python mirror of this file.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_MAGIC0 0xAA
#define PROTO_MAGIC1 0x55

#define PROTO_FRAME      0x01
#define PROTO_SET_PARAM  0x02
#define PROTO_GET_STATUS 0x03
#define PROTO_ACK        0x10
#define PROTO_STATUS     0x11
#define PROTO_SCORES     0x12

/* Frame geometry is part of the wire contract.  Must match the ray
 * table geometry in ray_tables.h (compile-time checked in main.c). */
#define PROTO_FRAME_W     64
#define PROTO_FRAME_H     64
#define PROTO_FRAME_BYTES (PROTO_FRAME_W * PROTO_FRAME_H)
#define PROTO_N_ANGLES    240
#define PROTO_SCORE_BYTES (2 * PROTO_N_ANGLES) /* u16 per angle */

/* STATUS flags */
#define PROTO_FLAG_NEEDLE 0x01 /* confidence >= conf_min: reading is live */

/* Largest possible message on the wire (a FRAME). */
#define PROTO_MAX_MSG (2 + 1 + 3 + PROTO_FRAME_BYTES + 2)

typedef struct {
    uint8_t  type;
    /* FRAME / SCORES / ACK */
    uint8_t  seq;
    uint16_t len;           /* payload length */
    const uint8_t *payload; /* points into the parser buffer; valid until next push */
    /* ACK */
    uint8_t  ok;            /* 1 = ACK, 0 = NAK */
    /* SET_PARAM */
    uint8_t  param_id;
    int32_t  value;
    /* STATUS */
    int16_t  angle_cdeg;    /* smoothed needle angle, centidegrees */
    int32_t  value_milli;   /* calibrated reading x1000 */
    uint8_t  confidence;    /* 0..255, peak-vs-mean of the score array */
    uint8_t  flags, mode;
    uint32_t frame_us;      /* on-chip compute time for the last frame */
    uint16_t frames;        /* frames processed since boot */
    uint8_t  mean;          /* mean brightness of the last frame */
} proto_msg_t;

typedef struct {
    int      state;
    int      stage;
    uint32_t have, need;
    uint16_t crc, rx_crc;
    uint8_t  type;
    uint8_t  body[3 + PROTO_FRAME_BYTES];
} proto_parser_t;

/* Build the CRC table.  Call once at boot (parser/builders also lazily
 * self-initialize, so unit tests need no setup). */
void     proto_init(void);

/* CRC16-CCITT; seed with 0xFFFF.  Chainable for incremental use. */
uint16_t proto_crc16(uint16_t crc, const uint8_t *data, uint32_t len);

void proto_parser_reset(proto_parser_t *p);
int  proto_parser_in_msg(const proto_parser_t *p); /* mid-message? (for stall timeouts) */

/* Feed one byte.  Returns 1 = complete valid message in *out,
 * -1 = CRC mismatch (*out holds best-effort type/seq for the NAK),
 * 0 = need more bytes.  Unknown types and absurd lengths resync. */
int  proto_parser_push(proto_parser_t *p, uint8_t byte, proto_msg_t *out);

/* Builders write a complete wire message into dst (>= PROTO_MAX_MSG
 * for a frame, small otherwise) and return its length in bytes. */
int proto_build_frame(uint8_t *dst, uint8_t seq, const uint8_t *pixels, uint16_t len);
int proto_build_set_param(uint8_t *dst, uint8_t param_id, int32_t value);
int proto_build_get_status(uint8_t *dst);
int proto_build_ack(uint8_t *dst, uint8_t seq, uint8_t ok);
int proto_build_status(uint8_t *dst, int16_t angle_cdeg, int32_t value_milli,
                       uint8_t confidence, uint8_t flags, uint8_t mode,
                       uint32_t frame_us, uint16_t frames, uint8_t mean);
/* scores is the PROTO_N_ANGLES-entry u16 array from the fabric kernel. */
int proto_build_scores(uint8_t *dst, uint8_t seq, const uint16_t *scores);

#endif /* PROTOCOL_H */
