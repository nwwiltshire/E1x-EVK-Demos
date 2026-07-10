/*
 * test_dsp.c — unit tests for the firmware core: CRC + protocol
 * roundtrips, u-law codec, log2 approximation, fixed-point FFT vs a
 * double-precision DFT reference, baseline model, event hysteresis.
 * Plain C, no framework: exit code = number of failed checks.
 *
 * Build & run:  make -C tests
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI /* strict c99 doesn't define it */
#define M_PI 3.14159265358979323846
#endif

#include "detector.h"
#include "dsp.h"
#include "fft_tables.h"
#include "protocol.h"

static int g_fail, g_checks;

#define CHECK(cond)                                                     \
    do {                                                                \
        g_checks++;                                                     \
        if (!(cond)) {                                                  \
            g_fail++;                                                   \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                               \
    } while (0)

#define CHECK_EQ(a, b)                                                  \
    do {                                                                \
        long _va = (long)(a), _vb = (long)(b);                          \
        g_checks++;                                                     \
        if (_va != _vb) {                                               \
            g_fail++;                                                   \
            printf("FAIL %s:%d: %s == %s  (%ld != %ld)\n", __FILE__,    \
                   __LINE__, #a, #b, _va, _vb);                         \
        }                                                               \
    } while (0)

/* static buffers: the detector alone is ~15KB */
static proto_parser_t P;
static uint8_t WIRE[PROTO_MAX_MSG];
static uint8_t ULAW[AU_CHUNK];
static detector_t D;
static params_t PR;

static int16_t PCM[AU_FFT_N];
static int16_t WT[AU_FFT_N], RE[AU_FFT_N], IM[AU_FFT_N];
static uint8_t SPEC[AU_NBINS], RSPEC[AU_NBINS];

/* Feed a buffer through the parser; returns the code of the last
 * non-zero push result (1 complete, -1 crc error, 0 nothing). */
static int feed(const uint8_t *buf, int n, proto_msg_t *out)
{
    int last = 0;

    for (int i = 0; i < n; i++) {
        int r = proto_parser_push(&P, buf[i], out);

        if (r != 0)
            last = r;
    }
    return last;
}

/* ------------------------------------------------------------------ */

static void test_crc(void)
{
    /* standard CRC-16/CCITT-FALSE check value */
    CHECK_EQ(proto_crc16(0xFFFF, (const uint8_t *)"123456789", 9), 0x29B1);
    CHECK_EQ(proto_crc16(0xFFFF, (const uint8_t *)"", 0), 0xFFFF);

    uint16_t c = proto_crc16(0xFFFF, (const uint8_t *)"1234", 4);
    c = proto_crc16(c, (const uint8_t *)"56789", 5);
    CHECK_EQ(c, 0x29B1);
}

static void test_protocol_roundtrip(void)
{
    proto_msg_t m;
    int n;

    proto_parser_reset(&P);

    /* AUDIO */
    for (int i = 0; i < AU_CHUNK; i++)
        ULAW[i] = (uint8_t)(i * 7 + 3);
    n = proto_build_audio(WIRE, 42, ULAW, AU_CHUNK);
    CHECK_EQ(n, PROTO_MAX_MSG);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_AUDIO);
    CHECK_EQ(m.seq, 42);
    CHECK_EQ(m.len, AU_CHUNK);
    CHECK(memcmp(m.payload, ULAW, AU_CHUNK) == 0);

    /* SET_PARAM (negative value survives) */
    n = proto_build_set_param(WIRE, PARAM_THRESHOLD, -123456);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_SET_PARAM);
    CHECK_EQ(m.param_id, PARAM_THRESHOLD);
    CHECK_EQ(m.value, -123456);

    /* GET_STATUS */
    n = proto_build_get_status(WIRE);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_GET_STATUS);

    /* ACK */
    n = proto_build_ack(WIRE, 7, 1);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_ACK);
    CHECK_EQ(m.seq, 7);
    CHECK_EQ(m.ok, 1);

    /* STATUS */
    n = proto_build_status(WIRE, 51234, PROTO_FLAG_EVENT, DET_CLASS_HIGH,
                           MODE_DEPLOY, 100, 422, 123456789u, 42);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_STATUS);
    CHECK_EQ(m.score, 51234);
    CHECK_EQ(m.flags, PROTO_FLAG_EVENT);
    CHECK_EQ(m.ev_class, DET_CLASS_HIGH);
    CHECK_EQ(m.mode, MODE_DEPLOY);
    CHECK_EQ(m.learn_pct, 100);
    CHECK_EQ(m.top_bin, 422);
    CHECK(m.chunk_us == 123456789u);
    CHECK_EQ(m.events, 42);

    /* SPECTRUM */
    uint8_t viz[PROTO_SPEC_BYTES];
    for (int i = 0; i < PROTO_SPEC_BYTES; i++)
        viz[i] = (uint8_t)(255 - (i & 0xFF));
    n = proto_build_spectrum(WIRE, 9, viz);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_SPECTRUM);
    CHECK_EQ(m.seq, 9);
    CHECK_EQ(m.len, PROTO_SPEC_BYTES);
    CHECK(memcmp(m.payload, viz, PROTO_SPEC_BYTES) == 0);
}

static void test_protocol_errors(void)
{
    proto_msg_t m;
    int n;

    proto_parser_reset(&P);

    /* corrupt payload -> -1 (CRC), best-effort seq for the NAK */
    n = proto_build_audio(WIRE, 5, ULAW, AU_CHUNK);
    WIRE[100] ^= 0xFF;
    CHECK_EQ(feed(WIRE, n, &m), -1);
    CHECK_EQ(m.type, PROTO_AUDIO);
    CHECK_EQ(m.seq, 5);

    /* garbage + a clean message: parser resyncs */
    const uint8_t junk[] = { 0x00, 0xAA, 0x00, 0xAA, 0xAA, 0x55, 0x99, 0xFF };
    CHECK_EQ(feed(junk, (int)sizeof junk, &m), 0);
    n = proto_build_ack(WIRE, 3, 1);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.seq, 3);

    /* absurd AUDIO length resyncs instead of swallowing the stream */
    uint8_t bad[8] = { 0xAA, 0x55, PROTO_AUDIO, 0, 0xFF, 0xFF, 0, 0 };
    CHECK_EQ(feed(bad, (int)sizeof bad, &m), 0);
    n = proto_build_get_status(WIRE);
    CHECK_EQ(feed(WIRE, n, &m), 1);
    CHECK_EQ(m.type, PROTO_GET_STATUS);
}

/* ------------------------------------------------------------------ */

/* Reference u-law encoder (Sun/G.711); mirrors host/audio_source.py. */
static uint8_t ulaw_encode_ref(int16_t pcm)
{
    int sign = pcm < 0 ? 0x80 : 0;
    int32_t mag = pcm < 0 ? -(int32_t)pcm : pcm;
    int seg = 0;

    if (mag > 32635)
        mag = 32635;
    mag += 0x84;
    while (seg < 7 && (mag >> (seg + 8)) != 0)
        seg++;
    return (uint8_t)(~(sign | (seg << 4) | ((mag >> (seg + 3)) & 0x0F)) & 0xFF);
}

static void test_ulaw(void)
{
    /* known G.711 values */
    CHECK_EQ(detector_ulaw_decode(0xFF), 0);      /* +0 */
    CHECK_EQ(detector_ulaw_decode(0x7F), 0);      /* -0 */
    CHECK_EQ(detector_ulaw_decode(0x80), 32124);  /* max positive */
    CHECK_EQ(detector_ulaw_decode(0x00), -32124); /* max negative */

    /* code -> pcm -> code is the identity (except -0, which encodes
     * back as +0 = 0xFF) */
    for (int b = 0; b < 256; b++) {
        if (b == 0x7F)
            continue;
        CHECK_EQ(ulaw_encode_ref(detector_ulaw_decode((uint8_t)b)), b);
    }

    /* pcm -> code -> pcm stays within the segment's quantization step */
    for (int32_t v = -32700; v <= 32700; v += 37) {
        int16_t d = detector_ulaw_decode(ulaw_encode_ref((int16_t)v));
        int32_t err = (int32_t)d - v;

        if (err < 0)
            err = -err;
        CHECK(err <= (v < 0 ? -v : v) / 8 + 8);
        g_checks--; /* count the sweep as one check */
    }
    g_checks++;
}

static void test_log2u8(void)
{
    CHECK_EQ(dsp_log2u8(0), 0);
    CHECK_EQ(dsp_log2u8(1), 0);
    CHECK_EQ(dsp_log2u8(2), 8);
    CHECK_EQ(dsp_log2u8(4), 16);
    CHECK_EQ(dsp_log2u8(1u << 20), 160);
    CHECK_EQ(dsp_log2u8(1u << 31), 248);
    CHECK_EQ(dsp_log2u8(0xFFFFFFFFu), 255);

    /* approx <= 8*log2(v) < approx + 1.7, and monotonic */
    uint8_t prev = 0;
    int ok_bound = 1, ok_mono = 1;
    for (uint32_t v = 1; v < (1u << 24); v = v + 1 + v / 7) {
        uint8_t r = dsp_log2u8(v);
        double t = 8.0 * log2((double)v);

        if (!(r <= t + 1e-9 && t - r < 1.7))
            ok_bound = 0;
        if (r < prev)
            ok_mono = 0;
        prev = r;
    }
    CHECK(ok_bound);
    CHECK(ok_mono);
}

/* ------------------------------------------------------------------ */

static void test_tables(void)
{
    /* periodic Hann: w[0] = 0, w[N/2] = max, w[i] = w[N-i] */
    CHECK_EQ(AU_HANN_Q15[0], 0);
    CHECK_EQ(AU_HANN_Q15[AU_FFT_N / 2], 32767);
    int sym = 1;
    for (int i = 1; i < AU_FFT_N; i++)
        if (AU_HANN_Q15[i] != AU_HANN_Q15[AU_FFT_N - i])
            sym = 0;
    CHECK(sym);

    /* bit reversal is an involution */
    int inv = 1;
    for (int i = 0; i < AU_FFT_N; i++)
        if (AU_BITREV[AU_BITREV[i]] != i)
            inv = 0;
    CHECK(inv);

    /* twiddles: w[0] = 1, |w| ~ 1 */
    CHECK_EQ(AU_TW_RE[0], 32767);
    CHECK_EQ(AU_TW_IM[0], 0);
    CHECK_EQ(AU_TW_IM[AU_FFT_N / 4], -32767); /* -sin(pi/2) */
}

/* Run the fixed-point pipeline pcm -> spec. */
static void run_fft(void)
{
    au_window(PCM, AU_HANN_Q15, WT, AU_FFT_N);
    au_bitrev_gather(WT, AU_BITREV, RE, IM, AU_FFT_N);
    au_fft(RE, IM, AU_TW_RE, AU_TW_IM, AU_FFT_N);
    au_logmag(RE, IM, SPEC, AU_NBINS, AU_MAG2_FLOOR);
}

/* Double-precision reference: same window, DFT, same 1/N scaling. */
static void ref_spectrum(void)
{
    static double xw[AU_FFT_N];

    for (int i = 0; i < AU_FFT_N; i++)
        xw[i] = (double)PCM[i] * AU_HANN_Q15[i] / 32768.0;
    for (int k = 0; k < AU_NBINS; k++) {
        double xr = 0, xi = 0;

        for (int n = 0; n < AU_FFT_N; n++) {
            double a = -2.0 * M_PI * k * n / AU_FFT_N;

            xr += xw[n] * cos(a);
            xi += xw[n] * sin(a);
        }
        xr /= AU_FFT_N;
        xi /= AU_FFT_N;
        double m2 = xr * xr + xi * xi + AU_MAG2_FLOOR;

        RSPEC[k] = (uint8_t)(8.0 * log2(m2));
    }
}

static int argmax_spec(void)
{
    int best = 0;

    for (int i = 1; i < AU_NBINS; i++)
        if (SPEC[i] > SPEC[best])
            best = i;
    return best;
}

/* every reference bin above `floor` must match within `tol` units */
static void check_against_ref(int floor_u8, int tol, const char *what)
{
    int worst = 0;

    for (int k = 0; k < AU_NBINS; k++) {
        if (RSPEC[k] < floor_u8)
            continue;
        int d = (int)SPEC[k] - (int)RSPEC[k];

        if (d < 0)
            d = -d;
        if (d > worst)
            worst = d;
    }
    g_checks++;
    if (worst > tol) {
        g_fail++;
        printf("FAIL fft vs ref (%s): worst diff %d > %d\n", what, worst, tol);
    }
}

static void test_fft(void)
{
    /* DC */
    for (int i = 0; i < AU_FFT_N; i++)
        PCM[i] = 16000;
    run_fft();
    ref_spectrum();
    CHECK_EQ(argmax_spec(), 0);
    check_against_ref(80, 4, "dc");

    /* centered impulse: flat spectrum near the fixed-point floor */
    memset(PCM, 0, sizeof PCM);
    PCM[AU_FFT_N / 2] = 32000;
    run_fft();
    ref_spectrum();
    check_against_ref(40, 8, "impulse");

    /* strong bin-centered tone */
    for (int i = 0; i < AU_FFT_N; i++)
        PCM[i] = (int16_t)(20000.0 * sin(2.0 * M_PI * 16.0 * i / AU_FFT_N));
    run_fft();
    ref_spectrum();
    CHECK_EQ(argmax_spec(), 16);
    check_against_ref(100, 4, "tone bin 16");
    CHECK(SPEC[300] < SPEC[16] - 100); /* leakage + fp noise stay far down */

    /* high-frequency tone (the "jingle" band) */
    for (int i = 0; i < AU_FFT_N; i++)
        PCM[i] = (int16_t)(20000.0 * sin(2.0 * M_PI * 422.0 * i / AU_FFT_N));
    run_fft();
    ref_spectrum();
    CHECK_EQ(argmax_spec(), 422);
    check_against_ref(100, 4, "tone bin 422");

    /* -40 dB tone still resolves cleanly */
    for (int i = 0; i < AU_FFT_N; i++)
        PCM[i] = (int16_t)(200.0 * sin(2.0 * M_PI * 422.0 * i / AU_FFT_N));
    run_fft();
    ref_spectrum();
    CHECK_EQ(argmax_spec(), 422);
    check_against_ref(60, 6, "tone -40dB");
}

/* ------------------------------------------------------------------ */

static void test_excess_update(void)
{
    uint8_t spec[4] = { 10, 100, 50, 0 };
    uint16_t mu[4] = { 10 << 8, 90 << 8, 60 << 8, 0 };
    uint16_t dev[4] = { 512, 512, 512, 512 };
    uint16_t ex[4];

    /* trigger = mu + 2.0*dev + 4 units */
    au_excess(spec, mu, dev, ex, 4, 32, 4 << 8);
    CHECK_EQ(ex[0], 0);
    CHECK_EQ(ex[1], 2); /* 100 - (90 + 4 + 4) */
    CHECK_EQ(ex[2], 0);
    CHECK_EQ(ex[3], 0);

    au_baseline_update(spec, mu, dev, 4, 2, 384);
    CHECK_EQ(mu[1], (90 << 8) + 640);  /* += (10<<8)>>2 */
    CHECK_EQ(dev[1], 1024);            /* += (2560-512)>>2 */
    CHECK_EQ(mu[0], 10 << 8);          /* on-baseline bin unchanged */

    /* dev decays toward the floor but never below it */
    uint8_t s2[1] = { 40 };
    uint16_t m2[1] = { 40 << 8 };
    uint16_t d2[1] = { 400 };
    for (int i = 0; i < 50; i++)
        au_baseline_update(s2, m2, d2, 1, 2, 384);
    CHECK_EQ(d2[0], 384);
}

/* ------------------------------------------------------------------ */

static uint32_t g_lcg = 12345;
static uint32_t g_n; /* absolute sample counter: phase-continuous chunks */

static double noise(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return ((double)((g_lcg >> 8) & 0xFFFF) / 65536.0 - 0.5) * 2.0;
}

/* room hum (matches host SynthSource) + optional anomaly tone */
static void synth_chunk(double tone_amp, double tone_hz)
{
    for (int i = 0; i < AU_CHUNK; i++) {
        double t = (double)g_n++ / AU_RATE;
        double v = 1200.0 * sin(2.0 * M_PI * 120.0 * t) +
                   500.0 * sin(2.0 * M_PI * 240.0 * t) +
                   250.0 * sin(2.0 * M_PI * 360.0 * t) +
                   120.0 * noise() +
                   tone_amp * sin(2.0 * M_PI * tone_hz * t);

        if (v > 32767.0)
            v = 32767.0;
        if (v < -32768.0)
            v = -32768.0;
        ULAW[i] = ulaw_encode_ref((int16_t)v);
    }
}

static void test_params(void)
{
    params_defaults(&PR);
    CHECK_EQ(params_set(&PR, PARAM_THRESHOLD, 100), 0);
    CHECK_EQ(PR.threshold, 100);
    CHECK_EQ(params_set(&PR, PARAM_THRESHOLD, 0), -1);
    CHECK_EQ(params_set(&PR, PARAM_MODE, MODE_DEPLOY), 0);
    CHECK_EQ(params_set(&PR, PARAM_MODE, 7), -1);
    CHECK_EQ(params_set(&PR, PARAM_ADAPT_SHIFT, 13), -1);
    CHECK_EQ(params_set(&PR, 99, 1), -1);
    CHECK_EQ(params_set(&PR, PARAM_RELEARN, 0), -1); /* command, not a param */
}

static void test_detector(void)
{
    params_defaults(&PR);
    PR.learn_chunks = 8;
    detector_init(&D, &PR);
    CHECK_EQ(D.learn_left, 8);
    CHECK(detector_learn_pct(&D) < 100);

    /* learning phase: quiet room */
    for (int i = 0; i < 8; i++) {
        synth_chunk(0, 0);
        detector_process(&D, &PR, ULAW);
        CHECK_EQ(D.score, 0);
        g_checks--;
    }
    g_checks++;
    CHECK_EQ(D.learn_left, 0);
    CHECK_EQ(detector_learn_pct(&D), 100);

    /* watching a quiet room: no events, score stays low */
    int max_quiet = 0;
    for (int i = 0; i < 12; i++) {
        synth_chunk(0, 0);
        detector_process(&D, &PR, ULAW);
        if (D.score > max_quiet)
            max_quiet = D.score;
    }
    CHECK(max_quiet < PR.threshold);
    CHECK_EQ(D.ev_active, 0);
    CHECK_EQ(D.events, 0);

    /* jingle: high tone burst -> HIGH event, baseline frozen */
    uint16_t mu_snapshot = D.mu[422];
    for (int i = 0; i < 4; i++) {
        synth_chunk(6000, 3300);
        detector_process(&D, &PR, ULAW);
    }
    CHECK(D.score >= PR.threshold);
    CHECK_EQ(D.ev_active, 1);
    CHECK_EQ(D.ev_class, DET_CLASS_HIGH);
    CHECK(D.top_bin >= 415 && D.top_bin <= 430);
    CHECK_EQ(D.mu[422], mu_snapshot); /* the anomaly was not learned */
    CHECK_EQ(D.events, 1);

    /* silence again: event clears after event_hold quiet chunks */
    for (int i = 0; i < 2 + PR.event_hold; i++) {
        synth_chunk(0, 0);
        detector_process(&D, &PR, ULAW);
    }
    CHECK_EQ(D.ev_active, 0);

    /* thump: low knock -> LOW event */
    for (int i = 0; i < 3; i++) {
        synth_chunk(9000, 70);
        detector_process(&D, &PR, ULAW);
    }
    CHECK_EQ(D.ev_active, 1);
    CHECK_EQ(D.ev_class, DET_CLASS_LOW);
    CHECK(D.top_bin < 32);
    CHECK_EQ(D.events, 2);

    /* viz: the spike lands in the pooled bin, trig plane >= base plane */
    uint8_t viz[PROTO_SPEC_BYTES];
    detector_viz(&D, &PR, viz);
    int ordered = 1;
    for (int j = 0; j < AU_VIZ_BINS; j++)
        if (viz[2 * AU_VIZ_BINS + j] < viz[AU_VIZ_BINS + j])
            ordered = 0;
    CHECK(ordered);
    /* spec > base in the pooled bin holding the knock */
    CHECK(viz[D.top_bin / 4] > viz[AU_VIZ_BINS + D.top_bin / 4]);

    /* relearn resets the state machine */
    detector_relearn(&D, &PR);
    CHECK_EQ(D.learn_left, 8);
    CHECK_EQ(D.score, 0);
    CHECK_EQ(D.ev_active, 0);
}

static void test_persistent_sound_absorbed(void)
{
    /* a sound that never stops must become the new normal: the
     * adaptation freeze is bounded, so the event fires, the baseline
     * absorbs the tone, and the event closes — no endless flapping */
    params_defaults(&PR);
    PR.learn_chunks = 8;
    detector_init(&D, &PR);
    for (int i = 0; i < 8; i++) {
        synth_chunk(0, 0);
        detector_process(&D, &PR, ULAW);
    }

    uint16_t mu_before = D.mu[422];
    int fired = 0, flaps = 0, prev_active = 0;

    for (int i = 0; i < 100; i++) {
        synth_chunk(6000, 3300);
        detector_process(&D, &PR, ULAW);
        if (D.ev_active && !prev_active) {
            fired = 1;
            flaps++;
        }
        prev_active = D.ev_active;
    }
    CHECK(fired);
    CHECK_EQ(D.ev_active, 0);              /* healed */
    CHECK(D.score < PR.threshold);         /* tone is the new baseline */
    CHECK(D.mu[422] > mu_before);          /* absorbed, not ignored */
    CHECK(flaps <= 2);                     /* no event storm */
    CHECK_EQ(D.events, (uint16_t)flaps);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    proto_init();

    test_crc();
    test_protocol_roundtrip();
    test_protocol_errors();
    test_ulaw();
    test_log2u8();
    test_tables();
    test_fft();
    test_excess_update();
    test_params();
    test_detector();
    test_persistent_sound_absorbed();

    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail;
}
