#include "gauge.h"

#include "kernels.h"

void params_defaults(params_t *p)
{
    p->mode = MODE_DEV;
    p->polarity = 0;      /* dark needle on a light face */
    p->smooth_shift = 2;  /* EMA alpha 1/4: settles in ~1 s at 2 fps */
    p->conf_min = 40;
    /* out-of-the-box calibration: a classic 270-degree sweep reading
     * 0..100, so the demo shows a plausible percentage uncalibrated */
    p->cal_angle_min = -13500;
    p->cal_angle_max = 13500;
    p->cal_value_min = 0;
    p->cal_value_max = 100000;
}

int params_set(params_t *p, uint8_t id, int32_t value)
{
    switch (id) {
    case PARAM_MODE:
        if (value != MODE_DEV && value != MODE_DEPLOY)
            return -1;
        p->mode = value;
        return 0;
    case PARAM_POLARITY:
        if (value != 0 && value != 1)
            return -1;
        p->polarity = value;
        return 0;
    case PARAM_SMOOTH_SHIFT:
        if (value < 0 || value > 6)
            return -1;
        p->smooth_shift = value;
        return 0;
    case PARAM_CONF_MIN:
        if (value < 0 || value > 255)
            return -1;
        p->conf_min = value;
        return 0;
    case PARAM_CAL_ANGLE_MIN:
    case PARAM_CAL_ANGLE_MAX:
        if (value < -18000 || value >= 18000)
            return -1;
        if (id == PARAM_CAL_ANGLE_MIN)
            p->cal_angle_min = value;
        else
            p->cal_angle_max = value;
        return 0;
    case PARAM_CAL_VALUE_MIN:
        p->cal_value_min = value;
        return 0;
    case PARAM_CAL_VALUE_MAX:
        p->cal_value_max = value;
        return 0;
    default:
        return -1;
    }
}

void gauge_init(gauge_t *g)
{
    g->angle_cdeg = 0;
    g->raw_cdeg = 0;
    g->value_milli = 0;
    g->confidence = 0;
    g->mean = 0;
    g->frames = 0;
    g->ema_cdeg = 0;
    g->ema_valid = 0;
    for (int i = 0; i < GK_ANGLES; i++)
        g->scores[i] = 0;
}

void gauge_reset(gauge_t *g)
{
    g->ema_valid = 0;
}

int32_t gauge_wrap_cdeg(int32_t cdeg)
{
    while (cdeg >= 18000)
        cdeg -= 36000;
    while (cdeg < -18000)
        cdeg += 36000;
    return cdeg;
}

/* Sweep position in [0, 36000): degrees travelled clockwise from the
 * scale minimum.  Handles sweeps that cross the +/-180 wrap (needles
 * pointing straight down). */
static int32_t sweep_cdeg(int32_t from, int32_t to)
{
    int32_t d = (to - from) % 36000;

    if (d < 0)
        d += 36000;
    return d;
}

int32_t gauge_value_milli(const params_t *p, int32_t angle_cdeg)
{
    int32_t sweep = sweep_cdeg(p->cal_angle_min, p->cal_angle_max);
    int32_t pos = sweep_cdeg(p->cal_angle_min, angle_cdeg);
    int64_t span = (int64_t)p->cal_value_max - p->cal_value_min;

    if (sweep == 0)
        return p->cal_value_min;
    if (pos > sweep) {
        /* outside the scale: clamp to the nearer end */
        if (pos - sweep < 36000 - pos)
            pos = sweep;
        else
            pos = 0;
    }
    return p->cal_value_min + (int32_t)(span * pos / sweep);
}

/* Peak angle with sub-step parabolic interpolation, in centidegrees. */
static int32_t peak_angle_cdeg(const uint16_t *scores, int *out_peak_idx)
{
    int best = 0;

    for (int i = 1; i < GK_ANGLES; i++)
        if (scores[i] > scores[best])
            best = i;
    *out_peak_idx = best;

    {
        int32_t s0 = scores[(best + GK_ANGLES - 1) % GK_ANGLES];
        int32_t s1 = scores[best];
        int32_t s2 = scores[(best + 1) % GK_ANGLES];
        int32_t denom = s0 - 2 * s1 + s2;
        int32_t frac_q8 = 0; /* peak offset in [-1/2, +1/2] steps, Q8 */

        if (denom != 0) {
            frac_q8 = (128 * (s0 - s2)) / denom;
            if (frac_q8 > 128)
                frac_q8 = 128;
            if (frac_q8 < -128)
                frac_q8 = -128;
        }
        return gauge_wrap_cdeg(-18000 + best * GK_STEP_CDEG +
                               ((GK_STEP_CDEG * frac_q8) >> 8));
    }
}

/* EMA over a wrapping angle: step by the shortest signed arc. */
static int32_t smooth_angle(gauge_t *g, int32_t new_cdeg, int shift)
{
    int32_t delta;

    if (!g->ema_valid || shift == 0) {
        g->ema_cdeg = new_cdeg;
        g->ema_valid = 1;
        return new_cdeg;
    }
    delta = gauge_wrap_cdeg(new_cdeg - g->ema_cdeg);
    /* symmetric shift: >> on a negative would bias toward -inf */
    if (delta >= 0)
        g->ema_cdeg += delta >> shift;
    else
        g->ema_cdeg -= (-delta) >> shift;
    g->ema_cdeg = gauge_wrap_cdeg(g->ema_cdeg);
    return g->ema_cdeg;
}

void gauge_process(gauge_t *g, const params_t *p, const uint8_t *pixels)
{
    static uint8_t s_blur[GK_IMG_N]; /* single instance; no reentrancy */
    int32_t sum = 0;
    int mean, peak_idx;
    int32_t raw_cdeg, smoothed;
    uint32_t total = 0, peak, avg;

    gk_blur3x3(pixels, s_blur, GK_IMG_W, GK_IMG_H);
    gk_pixel_sum(s_blur, GK_IMG_N, &sum);
    mean = sum >> 12; /* / GK_IMG_N (= 4096), exact */
    gk_ray_scores(s_blur, gk_ray_idx, GK_ANGLES, GK_RADII, mean,
                  p->polarity ? 1 : -1, g->scores);

    raw_cdeg = peak_angle_cdeg(g->scores, &peak_idx);

    /* confidence: how far the peak stands above the average angle,
     * derated when the peak itself is small — a blank face has tiny
     * scores whose relative spread is meaningless noise.  600 is ~24
     * radii x 25 grey levels, well below any visible needle. */
    for (int i = 0; i < GK_ANGLES; i++)
        total += g->scores[i];
    peak = g->scores[peak_idx];
    avg = total / GK_ANGLES;
    {
        enum { CONF_PEAK_FLOOR = 600 };
        uint32_t rel = (peak > 0) ? (255u * (peak - avg)) / peak : 0;

        if (peak < CONF_PEAK_FLOOR)
            rel = rel * peak / CONF_PEAK_FLOOR;
        g->confidence = (uint8_t)rel;
    }

    if (g->confidence >= (uint32_t)p->conf_min)
        smoothed = smooth_angle(g, raw_cdeg, (int)p->smooth_shift);
    else /* no needle: hold the last smoothed angle */
        smoothed = g->ema_valid ? g->ema_cdeg : raw_cdeg;

    g->raw_cdeg = (int16_t)raw_cdeg;
    g->angle_cdeg = (int16_t)smoothed;
    g->value_milli = gauge_value_milli(p, smoothed);
    g->mean = (uint8_t)mean;
    g->frames++;
}
