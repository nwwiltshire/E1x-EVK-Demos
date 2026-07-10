/*
 * protocol.h — host<->firmware serial protocol for the acoustic
 * anomaly detector.
 *
 * Wire format (little-endian), CRC16-CCITT (poly 0x1021, init 0xFFFF)
 * computed over everything after the AA 55 magic:
 *
 *   AUDIO      host->fw  AA 55 01 seq:u8 len:u16(=1024) payload(u-law) crc:u16
 *   SET_PARAM  host->fw  AA 55 02 param_id:u8 value:i32 crc:u16
 *   GET_STATUS host->fw  AA 55 03 crc:u16
 *   ACK/NAK    fw->host  AA 55 10 seq:u8 ok:u8 crc:u16
 *   STATUS     fw->host  AA 55 11 score:u16 flags:u8 ev_class:u8 mode:u8
 *                        learn_pct:u8 top_bin:u16 chunk_us:u32 events:u16 crc:u16
 *   SPECTRUM   fw->host  AA 55 12 seq:u8 len:u16(=384)
 *                        payload(spec[128] base[128] trig[128]) crc:u16
 *                        (DEV mode only)
 *
 * Reply order per AUDIO chunk: SPECTRUM (DEV) -> STATUS -> ACK.  The
 * ACK comes LAST: it is the host's licence to transmit, and the EVK
 * USB bridge is only clean half-duplex.
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

#define PROTO_AUDIO      0x01
#define PROTO_SET_PARAM  0x02
#define PROTO_GET_STATUS 0x03
#define PROTO_ACK        0x10
#define PROTO_STATUS     0x11
#define PROTO_SPECTRUM   0x12

/* Chunk geometry is part of the wire contract.  Must match AU_CHUNK /
 * AU_VIZ_BINS in detector.h (compile-time checked in main.c). */
#define PROTO_AUDIO_BYTES 1024 /* 8-bit u-law samples per chunk */
#define PROTO_VIZ_BINS    128
#define PROTO_SPEC_BYTES  (3 * PROTO_VIZ_BINS) /* spec + base + trig */

/* STATUS flags */
#define PROTO_FLAG_EVENT    0x01
#define PROTO_FLAG_LEARNING 0x02

/* Largest possible message on the wire (an AUDIO chunk). */
#define PROTO_MAX_MSG (2 + 1 + 3 + PROTO_AUDIO_BYTES + 2)

typedef struct {
    uint8_t  type;
    /* AUDIO / SPECTRUM / ACK */
    uint8_t  seq;
    uint16_t len;           /* payload length */
    const uint8_t *payload; /* points into the parser buffer; valid until next push */
    /* ACK */
    uint8_t  ok;            /* 1 = ACK, 0 = NAK */
    /* SET_PARAM */
    uint8_t  param_id;
    int32_t  value;
    /* STATUS */
    uint16_t score;
    uint8_t  flags, ev_class, mode, learn_pct;
    uint16_t top_bin;
    uint32_t chunk_us;
    uint16_t events;
} proto_msg_t;

typedef struct {
    int      state;
    int      stage;
    uint32_t have, need;
    uint16_t crc, rx_crc;
    uint8_t  type;
    uint8_t  body[3 + PROTO_AUDIO_BYTES];
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
 * for audio, small otherwise) and return its length in bytes. */
int proto_build_audio(uint8_t *dst, uint8_t seq, const uint8_t *ulaw, uint16_t len);
int proto_build_set_param(uint8_t *dst, uint8_t param_id, int32_t value);
int proto_build_get_status(uint8_t *dst);
int proto_build_ack(uint8_t *dst, uint8_t seq, uint8_t ok);
int proto_build_status(uint8_t *dst, uint16_t score, uint8_t flags,
                       uint8_t ev_class, uint8_t mode, uint8_t learn_pct,
                       uint16_t top_bin, uint32_t chunk_us, uint16_t events);
/* viz is the PROTO_SPEC_BYTES output of detector_viz(). */
int proto_build_spectrum(uint8_t *dst, uint8_t seq, const uint8_t *viz);

#endif /* PROTOCOL_H */
