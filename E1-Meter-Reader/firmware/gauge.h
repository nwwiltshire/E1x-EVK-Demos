/*
 * gauge.h — needle detection + calibrated reading (control core).
 *
 * The fabric (kernels.h) turns a frame into a per-angle evidence
 * array; this module turns that array into a reading: argmax with
 * sub-step parabolic interpolation, a confidence figure, an EMA
 * smoother, and the linear angle->value calibration.  Branchy,
 * sequential, tiny — the wrong shape for spatial hardware and small
 * enough not to matter.
 *
 * Angle convention (everywhere): 0 = 12 o'clock, clockwise positive,
 * centidegrees in [-18000, +18000).
 */
#ifndef GAUGE_H
#define GAUGE_H

#include <stdint.h>

#include "ray_tables.h"

enum { MODE_DEV = 0, MODE_DEPLOY = 1 };

enum {
    PARAM_MODE          = 1, /* 0 = DEV (stream scores), 1 = DEPLOY */
    PARAM_POLARITY      = 2, /* 0 = dark needle / light face, 1 = inverse */
    PARAM_SMOOTH_SHIFT  = 3, /* EMA strength 0..6 (0 = raw) */
    PARAM_CONF_MIN      = 4, /* 0..255: below this the reading is "no needle" */
    PARAM_CAL_ANGLE_MIN = 5, /* cdeg of the scale minimum */
    PARAM_CAL_ANGLE_MAX = 6, /* cdeg of the scale maximum */
    PARAM_CAL_VALUE_MIN = 7, /* reading x1000 at the scale minimum */
    PARAM_CAL_VALUE_MAX = 8, /* reading x1000 at the scale maximum */
    PARAM_RESET         = 9, /* command: clear the EMA state (value ignored) */
    PARAM_BURN          = 10,
};

typedef struct {
    int32_t mode;
    int32_t polarity;
    int32_t smooth_shift;
    int32_t conf_min;
    int32_t cal_angle_min, cal_angle_max; /* cdeg */
    int32_t cal_value_min, cal_value_max; /* milli-units */
    int32_t burn;                         /* 10: fabric power soak */
} params_t;

typedef struct {
    /* outputs of the last frame */
    int16_t  angle_cdeg;   /* smoothed */
    int16_t  raw_cdeg;     /* this frame's peak, uninterpolated smoothing input */
    int32_t  value_milli;
    uint8_t  confidence;
    uint8_t  mean;         /* mean brightness */
    uint16_t frames;
    uint16_t scores[GK_ANGLES];
    /* smoother state */
    int32_t  ema_cdeg;     /* Q0 centidegrees; valid iff ema_valid */
    int      ema_valid;
} gauge_t;

void params_defaults(params_t *p);
/* 0 = accepted, -1 = unknown id or out-of-range value. */
int  params_set(params_t *p, uint8_t id, int32_t value);

void gauge_init(gauge_t *g);
void gauge_reset(gauge_t *g); /* clear the EMA (PARAM_RESET) */

/* One frame: runs the fabric kernels on pixels (PROTO_FRAME_BYTES,
 * blurred in place into an internal buffer) and updates every output
 * field of g. */
void gauge_process(gauge_t *g, const params_t *p, const uint8_t *pixels);

/* Re-run the fabric kernels on the last frame without touching the
 * reading, smoother, or counters — constant-workload power soak
 * (PARAM_BURN).  See gauge_burn in gauge.c. */
void gauge_burn(gauge_t *g, const params_t *p);

/* Map a smoothed angle to a calibrated reading (exposed for tests). */
int32_t gauge_value_milli(const params_t *p, int32_t angle_cdeg);

/* Wrap any centidegree count into [-18000, +18000). */
int32_t gauge_wrap_cdeg(int32_t cdeg);

#endif /* GAUGE_H */
