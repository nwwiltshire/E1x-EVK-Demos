/*
 * test_gauge.c — unit tests for the meter-reader firmware core.
 *
 * Compiled with gcc against the actual firmware sources: the guard
 * macro in kernels.h neutralizes __efficient__, so the exact fabric
 * kernels run on the host.  Every DSP result is checked against a
 * slow-but-obviously-correct double-precision reference.
 *
 * No framework: CHECK() counts failures, the exit code reports them.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI /* strict c99 doesn't provide it */
#define M_PI 3.14159265358979323846
#endif

#include "gauge.h"
#include "kernels.h"
#include "protocol.h"

static int g_failures;

#define CHECK(cond, ...) do {                                   \
    if (!(cond)) {                                              \
        g_failures++;                                           \
        printf("FAIL %s:%d: ", __func__, __LINE__);             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    }                                                           \
} while (0)

/* ------------------------------------------------------------------ */
/* Synthetic gauge renderer (double precision, obviously correct)      */
/* ------------------------------------------------------------------ */

/* Distance from pixel (px,py) to the segment (x0,y0)-(x1,y1). */
static double seg_dist(double px, double py,
                       double x0, double y0, double x1, double y1)
{
    double dx = x1 - x0, dy = y1 - y0;
    double len2 = dx * dx + dy * dy;
    double t = len2 > 0 ? ((px - x0) * dx + (py - y0) * dy) / len2 : 0.0;

    if (t < 0)
        t = 0;
    if (t > 1)
        t = 1;
    dx = px - (x0 + t * dx);
    dy = py - (y0 + t * dy);
    return sqrt(dx * dx + dy * dy);
}

/* Needle at angle_deg (0 = up, CW+) on a plain face, plus a hub and a
 * rim ring outside the sampled radii. */
static void render_needle(uint8_t *img, double angle_deg, int light_needle)
{
    const double cx = GK_IMG_W / 2.0 - 0.5, cy = GK_IMG_H / 2.0 - 0.5;
    const double a = angle_deg * M_PI / 180.0;
    const double tipx = cx + 26.0 * sin(a), tipy = cy - 26.0 * cos(a);
    const int face = light_needle ? 40 : 230;
    const int ink = light_needle ? 235 : 25;

    for (int y = 0; y < GK_IMG_H; y++) {
        for (int x = 0; x < GK_IMG_W; x++) {
            double r = sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
            int v = face;

            if (seg_dist(x, y, cx, cy, tipx, tipy) < 1.6)
                v = ink;
            else if (r < 3.5)
                v = ink;                    /* hub */
            else if (r > 30.0 && r < 31.5)
                v = ink;                    /* rim, outside sampled radii */
            img[y * GK_IMG_W + x] = (uint8_t)v;
        }
    }
}

static double err_deg(double got, double want)
{
    double e = fmod(got - want + 540.0, 360.0) - 180.0;
    return fabs(e);
}

/* ------------------------------------------------------------------ */
/* CRC + protocol                                                      */
/* ------------------------------------------------------------------ */

static void test_crc(void)
{
    /* CCITT-FALSE check value */
    CHECK(proto_crc16(0xFFFF, (const uint8_t *)"123456789", 9) == 0x29B1,
          "CRC check value");
    /* chainable */
    uint16_t a = proto_crc16(0xFFFF, (const uint8_t *)"1234", 4);
    a = proto_crc16(a, (const uint8_t *)"56789", 5);
    CHECK(a == 0x29B1, "incremental CRC");
}

static int parse_all(proto_parser_t *p, const uint8_t *buf, int n,
                     proto_msg_t *msgs, int *results, int max)
{
    int count = 0;

    for (int i = 0; i < n; i++) {
        proto_msg_t m;
        int r = proto_parser_push(p, buf[i], &m);

        if (r != 0 && count < max) {
            msgs[count] = m;
            results[count] = r;
            count++;
        }
    }
    return count;
}

static void test_protocol_roundtrip(void)
{
    static uint8_t wire[PROTO_MAX_MSG];
    static uint8_t pixels[PROTO_FRAME_BYTES];
    static uint16_t scores[PROTO_N_ANGLES];
    proto_parser_t p;
    proto_msg_t m[4];
    int r[4], n, count;

    proto_parser_reset(&p);

    for (int i = 0; i < PROTO_FRAME_BYTES; i++)
        pixels[i] = (uint8_t)(i * 7);
    n = proto_build_frame(wire, 42, pixels, PROTO_FRAME_BYTES);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1, "frame parses");
    CHECK(m[0].type == PROTO_FRAME && m[0].seq == 42 &&
          m[0].len == PROTO_FRAME_BYTES, "frame fields");
    CHECK(memcmp(m[0].payload, pixels, PROTO_FRAME_BYTES) == 0, "frame payload");

    n = proto_build_set_param(wire, PARAM_CAL_ANGLE_MIN, -13500);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1 && m[0].type == PROTO_SET_PARAM &&
          m[0].param_id == PARAM_CAL_ANGLE_MIN && m[0].value == -13500,
          "set_param roundtrip");

    n = proto_build_status(wire, -12345, 987654, 200, PROTO_FLAG_NEEDLE,
                           MODE_DEV, 123456u, 999, 128);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1 && m[0].type == PROTO_STATUS, "status parses");
    CHECK(m[0].angle_cdeg == -12345 && m[0].value_milli == 987654 &&
          m[0].confidence == 200 && m[0].flags == PROTO_FLAG_NEEDLE &&
          m[0].mode == MODE_DEV && m[0].frame_us == 123456u &&
          m[0].frames == 999 && m[0].mean == 128, "status fields");

    for (int i = 0; i < PROTO_N_ANGLES; i++)
        scores[i] = (uint16_t)(i * 25);
    n = proto_build_scores(wire, 7, scores);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1 && m[0].type == PROTO_SCORES &&
          m[0].len == PROTO_SCORE_BYTES, "scores parses");
    CHECK(m[0].payload[3] == (uint8_t)((25 >> 8) & 0xFF) &&
          m[0].payload[2] == (uint8_t)(25 & 0xFF), "scores little-endian");

    n = proto_build_ack(wire, 9, 1);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1 && m[0].type == PROTO_ACK &&
          m[0].seq == 9 && m[0].ok == 1, "ack roundtrip");

    n = proto_build_get_status(wire);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1 && m[0].type == PROTO_GET_STATUS,
          "get_status roundtrip");
}

static void test_protocol_errors(void)
{
    static uint8_t wire[PROTO_MAX_MSG];
    static uint8_t junk[64];
    proto_parser_t p;
    proto_msg_t m[4];
    int r[4], n, count;

    proto_parser_reset(&p);

    /* corrupt payload -> CRC mismatch (-1) with best-effort seq */
    n = proto_build_ack(wire, 3, 1);
    wire[4] ^= 0x40;
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == -1 && m[0].seq == 3, "corrupt -> crc fail");

    /* garbage then a clean message: resync */
    for (int i = 0; i < 64; i++)
        junk[i] = (uint8_t)(0xAA); /* runs of AA must not break sync */
    count = parse_all(&p, junk, 64, m, r, 4);
    CHECK(count == 0, "junk produces nothing");
    n = proto_build_ack(wire, 5, 0);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1 && m[0].seq == 5, "resync after junk");

    /* absurd length resyncs without swallowing the stream */
    uint8_t bogus[] = {0xAA, 0x55, PROTO_FRAME, 1, 0xFF, 0xFF};
    count = parse_all(&p, bogus, sizeof bogus, m, r, 4);
    CHECK(count == 0, "absurd length rejected");
    n = proto_build_get_status(wire);
    count = parse_all(&p, wire, n, m, r, 4);
    CHECK(count == 1 && r[0] == 1, "clean after absurd length");
}

/* ------------------------------------------------------------------ */
/* Ray table + kernels vs references                                   */
/* ------------------------------------------------------------------ */

static void test_ray_table(void)
{
    for (int i = 0; i < GK_ANGLES * GK_RADII; i++)
        CHECK(gk_ray_idx[i] < GK_IMG_N, "index %d out of range", i);

    /* each ray's samples march outward from the centre */
    const double cx = GK_IMG_W / 2.0 - 0.5, cy = GK_IMG_H / 2.0 - 0.5;
    for (int a = 0; a < GK_ANGLES; a++) {
        double prev = -1.0;
        for (int ri = 0; ri < GK_RADII; ri++) {
            int idx = gk_ray_idx[a * GK_RADII + ri];
            double x = idx % GK_IMG_W, y = idx / GK_IMG_W;
            double r = sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
            CHECK(r >= prev - 1.0, "ray %d not outward at %d", a, ri);
            prev = r;
        }
    }

    /* angle 0 = up: the first ray's samples sit above the centre */
    int idx = gk_ray_idx[(GK_ANGLES / 2) * GK_RADII + GK_RADII - 1];
    CHECK(idx / GK_IMG_W < GK_IMG_H / 2,
          "index GK_ANGLES/2 (= 0 cdeg) points up");
}

static void test_blur_vs_ref(void)
{
    static uint8_t src[GK_IMG_N], dst[GK_IMG_N];
    srand(123);

    for (int i = 0; i < GK_IMG_N; i++)
        src[i] = (uint8_t)(rand() & 0xFF);
    gk_blur3x3(src, dst, GK_IMG_W, GK_IMG_H);

    for (int y = 0; y < GK_IMG_H; y++) {
        for (int x = 0; x < GK_IMG_W; x++) {
            int v = dst[y * GK_IMG_W + x];
            int want;

            if (x == 0 || y == 0 || x == GK_IMG_W - 1 || y == GK_IMG_H - 1) {
                want = src[y * GK_IMG_W + x];
            } else {
                const int w[3][3] = {{1, 2, 1}, {2, 4, 2}, {1, 2, 1}};
                int acc = 0;
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                        acc += w[dy + 1][dx + 1] *
                               src[(y + dy) * GK_IMG_W + (x + dx)];
                want = acc >> 4;
            }
            CHECK(v == want, "blur(%d,%d) = %d want %d", x, y, v, want);
            if (v != want)
                return; /* don't spam thousands of failures */
        }
    }
}

static void test_sum_and_scores_vs_ref(void)
{
    static uint8_t img[GK_IMG_N];
    static uint16_t scores[GK_ANGLES];
    int32_t sum = -1;
    long ref_sum = 0;
    srand(321);

    for (int i = 0; i < GK_IMG_N; i++) {
        img[i] = (uint8_t)(rand() & 0xFF);
        ref_sum += img[i];
    }
    gk_pixel_sum(img, GK_IMG_N, &sum);
    CHECK(sum == ref_sum, "pixel sum %d want %ld", sum, ref_sum);

    int mean = (int)(sum >> 12);
    for (int sign = -1; sign <= 1; sign += 2) {
        gk_ray_scores(img, gk_ray_idx, GK_ANGLES, GK_RADII, mean, sign, scores);
        for (int a = 0; a < GK_ANGLES; a++) {
            int ref = 0;
            for (int ri = 0; ri < GK_RADII; ri++) {
                int d = sign * ((int)img[gk_ray_idx[a * GK_RADII + ri]] - mean);
                if (d > 0)
                    ref += d;
            }
            CHECK(scores[a] == ref, "score[%d] sign %d = %d want %d",
                  a, sign, scores[a], ref);
            if (scores[a] != ref)
                return;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Needle detection end to end (against the double-precision renderer) */
/* ------------------------------------------------------------------ */

static void test_needle_sweep(void)
{
    static uint8_t img[GK_IMG_N];
    gauge_t g;
    params_t p;

    params_defaults(&p);
    p.smooth_shift = 0; /* raw angles: each frame stands alone */

    for (int polarity = 0; polarity <= 1; polarity++) {
        p.polarity = polarity;
        gauge_init(&g);
        for (double want = -175.0; want < 180.0; want += 13.7) {
            render_needle(img, want, polarity);
            gauge_process(&g, &p, img);
            CHECK(err_deg(g.angle_cdeg / 100.0, want) < 2.0,
                  "polarity %d angle %.1f -> %.2f (err %.2f)",
                  polarity, want, g.angle_cdeg / 100.0,
                  err_deg(g.angle_cdeg / 100.0, want));
            CHECK(g.confidence >= p.conf_min,
                  "polarity %d angle %.1f confidence %d < %d",
                  polarity, want, g.confidence, (int)p.conf_min);
        }
    }
}

static void test_no_needle_low_confidence(void)
{
    static uint8_t img[GK_IMG_N];
    gauge_t g;
    params_t p;

    params_defaults(&p);
    gauge_init(&g);
    /* a blank face: nothing but the hub and rim */
    render_needle(img, 0.0, 0);
    for (int i = 0; i < GK_IMG_N; i++) {
        int x = i % GK_IMG_W, y = i / GK_IMG_W;
        double r = sqrt(pow(x - 31.5, 2) + pow(y - 31.5, 2));
        if (r > 4.5 && r < 29.5)
            img[i] = 230; /* erase the needle, keep hub + rim */
    }
    gauge_process(&g, &p, img);
    CHECK(g.confidence < 40, "blank face confidence %d", g.confidence);
}

/* ------------------------------------------------------------------ */
/* Control-core math                                                   */
/* ------------------------------------------------------------------ */

static void test_wrap(void)
{
    CHECK(gauge_wrap_cdeg(18000) == -18000, "wrap +180");
    CHECK(gauge_wrap_cdeg(-18000) == -18000, "wrap -180");
    CHECK(gauge_wrap_cdeg(36000) == 0, "wrap 360");
    CHECK(gauge_wrap_cdeg(-54000) == -18000, "wrap -540");
    CHECK(gauge_wrap_cdeg(100) == 100, "wrap identity");
}

static void test_value_mapping(void)
{
    params_t p;

    params_defaults(&p);
    /* default: -135..+135 deg -> 0..100.000 */
    CHECK(gauge_value_milli(&p, -13500) == 0, "min end");
    CHECK(gauge_value_milli(&p, 13500) == 100000, "max end");
    CHECK(gauge_value_milli(&p, 0) == 50000, "middle");
    /* outside the sweep clamps to the nearer end */
    CHECK(gauge_value_milli(&p, -17000) == 0, "clamp below");
    CHECK(gauge_value_milli(&p, 17000) == 100000, "clamp above");

    /* a sweep that crosses the +/-180 wrap: 90 -> -90 through down */
    p.cal_angle_min = 9000;
    p.cal_angle_max = -9000;
    p.cal_value_min = -50000;
    p.cal_value_max = 50000;
    CHECK(gauge_value_milli(&p, 9000) == -50000, "wrap sweep min");
    CHECK(gauge_value_milli(&p, -9000) == 50000, "wrap sweep max");
    CHECK(gauge_value_milli(&p, 18000 - 18000) == 0 ||
          gauge_value_milli(&p, -18000) == 0, "wrap sweep middle (down)");

    /* degenerate calibration */
    p.cal_angle_min = p.cal_angle_max = 0;
    CHECK(gauge_value_milli(&p, 5000) == p.cal_value_min, "zero sweep");
}

static void test_params(void)
{
    params_t p;

    params_defaults(&p);
    CHECK(params_set(&p, PARAM_MODE, MODE_DEPLOY) == 0 && p.mode == MODE_DEPLOY,
          "set mode");
    CHECK(params_set(&p, PARAM_MODE, 7) < 0, "bad mode rejected");
    CHECK(params_set(&p, PARAM_POLARITY, 1) == 0, "set polarity");
    CHECK(params_set(&p, PARAM_SMOOTH_SHIFT, 9) < 0, "bad smooth rejected");
    CHECK(params_set(&p, PARAM_CONF_MIN, 300) < 0, "bad conf rejected");
    CHECK(params_set(&p, PARAM_CAL_ANGLE_MIN, 25000) < 0, "bad angle rejected");
    CHECK(params_set(&p, PARAM_CAL_VALUE_MAX, -2000000) == 0, "any value ok");
    CHECK(params_set(&p, 99, 0) < 0, "unknown id rejected");
}

static void test_smoothing(void)
{
    static uint8_t img[GK_IMG_N];
    gauge_t g;
    params_t p;
    int16_t first;

    params_defaults(&p);
    p.smooth_shift = 2;
    gauge_init(&g);

    render_needle(img, 40.0, 0);
    gauge_process(&g, &p, img);
    first = g.angle_cdeg;
    CHECK(err_deg(first / 100.0, 40.0) < 2.0, "first frame direct");

    /* a jump: the EMA must move toward the new angle, not reach it */
    render_needle(img, 100.0, 0);
    gauge_process(&g, &p, img);
    CHECK(g.angle_cdeg > first + 1000 && g.angle_cdeg < 9000,
          "EMA moves partway (%d)", g.angle_cdeg);

    /* converges after enough frames */
    for (int i = 0; i < 30; i++)
        gauge_process(&g, &p, img);
    CHECK(err_deg(g.angle_cdeg / 100.0, 100.0) < 2.5,
          "EMA converges (%.2f)", g.angle_cdeg / 100.0);

    /* reset makes the next frame direct again */
    gauge_reset(&g);
    render_needle(img, -60.0, 0);
    gauge_process(&g, &p, img);
    CHECK(err_deg(g.angle_cdeg / 100.0, -60.0) < 2.0, "direct after reset");
}

int main(void)
{
    proto_init();

    test_crc();
    test_protocol_roundtrip();
    test_protocol_errors();
    test_ray_table();
    test_blur_vs_ref();
    test_sum_and_scores_vs_ref();
    test_needle_sweep();
    test_no_needle_low_confidence();
    test_wrap();
    test_value_mapping();
    test_params();
    test_smoothing();

    if (g_failures == 0)
        printf("all gauge tests passed\n");
    else
        printf("%d FAILURES\n", g_failures);
    return g_failures;
}
