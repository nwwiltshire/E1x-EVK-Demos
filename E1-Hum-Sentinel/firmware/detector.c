#include "detector.h"

#include "dsp.h"
#include "fft_tables.h"

/* geometry of dsp tables and wire constants must agree */
typedef char assert_fft_n[(AU_FFT_N == AU_TABLES_FFT_N &&
                           AU_CHUNK == AU_FFT_N &&
                           AU_NBINS == AU_FFT_N / 2 &&
                           AU_NBINS % AU_VIZ_BINS == 0) ? 1 : -1];

#define LEARN_SHIFT  2   /* fast EMA during the learning phase */
#define DEV_FLOOR_Q8 384 /* 1.5 log units: never trigger on quantization
                          * noise in a dead-silent room */

/* ------------------------------------------------------------------ */
/* parameters                                                          */
/* ------------------------------------------------------------------ */

void params_defaults(params_t *p)
{
    p->threshold = 60;
    p->k_q4 = 40; /* 2.5 * dev */
    p->mode = MODE_DEV;
    p->adapt_shift = 6;
    p->margin = 6; /* ~4.5 dB above the k*dev band */
    p->event_hold = 4;
    p->learn_chunks = 40; /* ~5 s at 128 ms/chunk */
    p->burn = 0;
}

int params_set(params_t *p, uint8_t id, int32_t value)
{
    switch (id) {
    case PARAM_THRESHOLD:
        if (value < 1 || value > 65535) return -1;
        p->threshold = value; return 0;
    case PARAM_K_Q4:
        if (value < 1 || value > 255) return -1;
        p->k_q4 = value; return 0;
    case PARAM_MODE:
        if (value != MODE_DEV && value != MODE_DEPLOY) return -1;
        p->mode = value; return 0;
    case PARAM_ADAPT_SHIFT:
        if (value < 1 || value > 12) return -1;
        p->adapt_shift = value; return 0;
    case PARAM_MARGIN:
        if (value < 0 || value > 64) return -1;
        p->margin = value; return 0;
    case PARAM_EVENT_HOLD:
        if (value < 1 || value > 255) return -1;
        p->event_hold = value; return 0;
    case PARAM_LEARN_CHUNKS:
        if (value < 4 || value > 1000) return -1;
        p->learn_chunks = value; return 0;
    case PARAM_BURN:
        if (value != 0 && value != 1) return -1;
        p->burn = value; return 0;
    default: /* PARAM_RELEARN is a command, handled in main.c */
        return -1;
    }
}

/* ------------------------------------------------------------------ */
/* u-law decode (G.711)                                                */
/* ------------------------------------------------------------------ */

static int16_t s_ulaw[256];
static int s_ulaw_ready;

int16_t detector_ulaw_decode(uint8_t b)
{
    uint8_t u = (uint8_t)~b;
    int32_t t = (int32_t)(((u & 0x0F) << 3) + 0x84) << ((u & 0x70) >> 4);

    return (int16_t)((u & 0x80) ? (0x84 - t) : (t - 0x84));
}

static void ulaw_table_init(void)
{
    for (int i = 0; i < 256; i++)
        s_ulaw[i] = detector_ulaw_decode((uint8_t)i);
    s_ulaw_ready = 1;
}

/* ------------------------------------------------------------------ */
/* detector                                                            */
/* ------------------------------------------------------------------ */

void detector_relearn(detector_t *d, const params_t *p)
{
    d->learn_total = (uint16_t)p->learn_chunks;
    d->learn_left = d->learn_total;
    d->seeded = 0;
    d->score = 0;
    d->top_bin = 0;
    d->ev_active = 0;
    d->ev_peak = 0;
    d->frozen = 0;
    d->over = 0;
    d->under = 0;
    for (int i = 0; i < AU_NBINS; i++)
        d->excess[i] = 0;
}

void detector_init(detector_t *d, const params_t *p)
{
    if (!s_ulaw_ready)
        ulaw_table_init();
    for (int i = 0; i < AU_NBINS; i++) {
        d->mu[i] = 0;
        d->dev[i] = DEV_FLOOR_Q8;
        d->spec[i] = 0;
    }
    d->chunks = 0;
    d->events = 0;
    d->ev_class = DET_CLASS_NONE;
    detector_relearn(d, p);
}

uint8_t detector_learn_pct(const detector_t *d)
{
    if (d->learn_left == 0 || d->learn_total == 0)
        return 100;
    return (uint8_t)(100u - (100u * d->learn_left) / d->learn_total);
}

static uint8_t classify(uint16_t bin)
{
    /* 7.8125 Hz per bin: <250 Hz | 250 Hz - 2 kHz | >2 kHz */
    if (bin < 32)
        return DET_CLASS_LOW;
    if (bin < 256)
        return DET_CLASS_MID;
    return DET_CLASS_HIGH;
}

void detector_process(detector_t *d, const params_t *p, const uint8_t *ulaw)
{
    if (!s_ulaw_ready)
        ulaw_table_init();

    for (int i = 0; i < AU_CHUNK; i++)
        d->pcm[i] = s_ulaw[ulaw[i]];

    /* fabric stages */
    au_window(d->pcm, AU_HANN_Q15, d->wtmp, AU_FFT_N);
    au_bitrev_gather(d->wtmp, AU_BITREV, d->re, d->im, AU_FFT_N);
    au_fft(d->re, d->im, AU_TW_RE, AU_TW_IM, AU_FFT_N);
    au_logmag(d->re, d->im, d->spec, AU_NBINS, AU_MAG2_FLOOR);

    if (d->learn_left > 0) {
        /* learning: adapt fast, report no anomalies */
        if (!d->seeded) {
            for (int i = 0; i < AU_NBINS; i++) {
                d->mu[i] = (uint16_t)((uint32_t)d->spec[i] << 8);
                d->dev[i] = DEV_FLOOR_Q8 * 2;
            }
            d->seeded = 1;
        } else {
            au_baseline_update(d->spec, d->mu, d->dev, AU_NBINS,
                               LEARN_SHIFT, DEV_FLOOR_Q8);
        }
        d->learn_left--;
        d->score = 0;
        d->top_bin = 0;
        d->chunks++;
        return;
    }

    /* watching: score against the frozen-in baseline first... */
    au_excess(d->spec, d->mu, d->dev, d->excess, AU_NBINS,
              (int)p->k_q4, (int)(p->margin << 8));

    uint32_t sum = 0;
    uint32_t best = 0;
    uint16_t best_i = 0;
    for (int i = 0; i < AU_NBINS; i++) {
        uint16_t e = d->excess[i];
        sum += e;
        if (e > best) {
            best = e;
            best_i = (uint16_t)i;
        }
    }
    d->score = (uint16_t)(sum > 0xFFFF ? 0xFFFF : sum);
    d->top_bin = best_i;

    /* ...then event hysteresis (2 chunks to fire, event_hold to clear) */
    if (d->score >= p->threshold) {
        d->under = 0;
        if (d->over < 255)
            d->over++;
        if (!d->ev_active && d->over >= 2) {
            d->ev_active = 1;
            d->ev_peak = 0;
            if (d->events < 0xFFFF)
                d->events++;
        }
        if (d->ev_active && d->score >= d->ev_peak) {
            d->ev_peak = d->score;
            d->ev_class = classify(d->top_bin);
        }
    } else {
        d->over = 0;
        if (d->under < 255)
            d->under++;
        if (d->ev_active && d->under >= p->event_hold)
            d->ev_active = 0;
    }

    /* ...and adapt slowly, but never learn a brief anomaly into the
     * baseline: freeze while an event is active or the score is
     * anywhere near the threshold.  The freeze is bounded
     * (DET_FREEZE_LIMIT): a sound that persists past it is absorbed
     * as the new normal instead of alarming forever, so a baseline
     * that stops matching the room always heals itself. */
    int hot = d->ev_active || d->score >= (uint16_t)(p->threshold / 2);

    if (hot && d->frozen < DET_FREEZE_LIMIT) {
        d->frozen++;
    } else {
        if (!hot)
            d->frozen = 0;
        au_baseline_update(d->spec, d->mu, d->dev, AU_NBINS,
                           (int)p->adapt_shift, DEV_FLOOR_Q8);
    }

    d->chunks++;
}

void detector_burn(detector_t *d, const params_t *p)
{
    /* Exactly the fabric stages of detector_process, on the buffers
     * the last chunk left behind (recomputing the same spec/excess, so
     * the next real chunk sees nothing changed).  The u-law decode and
     * the control-core reductions/hysteresis are skipped — this is the
     * fabric workload, isolated. */
    au_window(d->pcm, AU_HANN_Q15, d->wtmp, AU_FFT_N);
    au_bitrev_gather(d->wtmp, AU_BITREV, d->re, d->im, AU_FFT_N);
    au_fft(d->re, d->im, AU_TW_RE, AU_TW_IM, AU_FFT_N);
    au_logmag(d->re, d->im, d->spec, AU_NBINS, AU_MAG2_FLOOR);
    au_excess(d->spec, d->mu, d->dev, d->excess, AU_NBINS,
              (int)p->k_q4, (int)(p->margin << 8));
}

void detector_viz(const detector_t *d, const params_t *p, uint8_t *out)
{
    const int pool = AU_NBINS / AU_VIZ_BINS;

    for (int j = 0; j < AU_VIZ_BINS; j++) {
        uint32_t smax = 0, bmax = 0, tmax = 0;

        for (int k = 0; k < pool; k++) {
            int i = j * pool + k;
            uint32_t trig = (uint32_t)d->mu[i] +
                            (((uint32_t)p->k_q4 * d->dev[i]) >> 4) +
                            ((uint32_t)p->margin << 8);

            if (d->spec[i] > smax)
                smax = d->spec[i];
            if (d->mu[i] > bmax)
                bmax = d->mu[i];
            if (trig > tmax)
                tmax = trig;
        }
        bmax >>= 8;
        tmax >>= 8;
        out[j] = (uint8_t)smax;
        out[AU_VIZ_BINS + j] = (uint8_t)(bmax > 255 ? 255 : bmax);
        out[2 * AU_VIZ_BINS + j] = (uint8_t)(tmax > 255 ? 255 : tmax);
    }
}
