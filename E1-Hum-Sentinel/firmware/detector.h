/*
 * detector.h — acoustic anomaly detector: learn the room's baseline
 * spectrum, flag deviations (a tapped bearing, jingled keys, a voice).
 *
 * Per 1024-sample chunk (128 ms at 8 kHz):
 *   u-law decode -> Hann window -> 1024-pt FFT -> 8*log2 magnitude
 *   -> per-bin excess over (mu + k*dev + margin) -> score = sum(excess)
 * with a per-bin EMA baseline (mu, dev) that learns fast for the first
 * learn_chunks chunks, then adapts slowly — and freezes while an
 * anomaly is in progress so the model never learns the anomaly.
 *
 * The per-sample/per-bin loops live in dsp.[ch] and run on the E1
 * dataflow fabric; everything here (decode LUT, reductions, event
 * hysteresis) is small sequential control-core code.
 */
#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdint.h>

#define AU_RATE     8000 /* samples/s reaching the chip */
#define AU_CHUNK    1024 /* samples per AUDIO message = 128 ms */
#define AU_FFT_N    1024
#define AU_NBINS    512  /* useful bins, 7.8125 Hz each, 0..4 kHz */
#define AU_VIZ_BINS 128  /* 4:1 max-pooled wire/viz resolution */

#define MODE_DEV    0 /* stream spectrum + baseline + trigger back */
#define MODE_DEPLOY 1 /* score/events only; no audio-derived data leaves */

#define DET_CLASS_NONE 0
#define DET_CLASS_LOW  1 /* < 250 Hz: thump, rumble, bearing knock */
#define DET_CLASS_MID  2 /* 250 Hz - 2 kHz: voice */
#define DET_CLASS_HIGH 3 /* > 2 kHz: keys, hiss, clink */

/* Baseline adaptation freezes while an anomaly is in progress so the
 * model doesn't learn it — but only for this many consecutive chunks
 * (~3 s).  After that a persistent sound is absorbed as the new
 * normal: without the bound, any lasting change to the room (or a
 * baseline carried over from an earlier session) parks the score
 * above the freeze gate forever and events flap indefinitely. */
#define DET_FREEZE_LIMIT 24

/* Runtime-tunable parameters (SET_PARAM ids in the comments). */
typedef struct {
    int32_t threshold;    /* 1: event when score >= threshold    (60)  */
    int32_t k_q4;         /* 2: trigger = mu + k*dev/16 + margin (40)  */
    int32_t mode;         /* 3: MODE_DEV / MODE_DEPLOY           (DEV) */
    int32_t adapt_shift;  /* 4: slow EMA shift while watching    (6)   */
    int32_t margin;       /* 5: trigger margin, log units        (6)   */
    int32_t event_hold;   /* 6: quiet chunks until event clears  (4)   */
    int32_t learn_chunks; /* 7: fast-learn phase length          (40)  */
} params_t;

enum {
    PARAM_THRESHOLD    = 1,
    PARAM_K_Q4         = 2,
    PARAM_MODE         = 3,
    PARAM_ADAPT_SHIFT  = 4,
    PARAM_MARGIN       = 5,
    PARAM_EVENT_HOLD   = 6,
    PARAM_LEARN_CHUNKS = 7,
    PARAM_RELEARN      = 8, /* command, not a value: reset the baseline */
};

void params_defaults(params_t *p);
int  params_set(params_t *p, uint8_t id, int32_t value); /* 0 = ok, -1 = bad id/range */

typedef struct {
    /* baseline model, Q8 log units per bin */
    uint16_t mu[AU_NBINS];
    uint16_t dev[AU_NBINS];
    /* per-chunk products */
    uint16_t excess[AU_NBINS];
    uint8_t  spec[AU_NBINS];
    /* FFT working buffers (int16 PCM in, re/im in place) */
    int16_t  pcm[AU_CHUNK];
    int16_t  wtmp[AU_FFT_N];
    int16_t  re[AU_FFT_N];
    int16_t  im[AU_FFT_N];
    /* state */
    uint32_t chunks;      /* chunks processed since boot */
    uint16_t learn_left;  /* fast-learn chunks remaining (0 = watching) */
    uint16_t learn_total;
    uint16_t score;       /* sum of per-bin excess, saturated */
    uint16_t top_bin;     /* argmax of excess (0 when score == 0) */
    uint16_t events;      /* events since boot */
    uint16_t ev_peak;     /* peak score of the active event */
    uint16_t frozen;      /* consecutive chunks with adaptation frozen */
    uint8_t  ev_active;
    uint8_t  ev_class;    /* DET_CLASS_* of the active/last event */
    uint8_t  over, under; /* hysteresis counters */
    uint8_t  seeded;      /* first learning chunk seeds mu directly */
} detector_t;

void detector_init(detector_t *d, const params_t *p);
void detector_relearn(detector_t *d, const params_t *p);

/* Process one u-law chunk (AU_CHUNK bytes): decode, transform, score,
 * update the baseline and event state. */
void detector_process(detector_t *d, const params_t *p, const uint8_t *ulaw);

/* Fill out[3*AU_VIZ_BINS] for the SPECTRUM message: max-pooled
 * spectrum, baseline mean, and trigger envelope (u8 log units). */
void detector_viz(const detector_t *d, const params_t *p, uint8_t *out);

/* 0..100 for the STATUS learn_pct field. */
uint8_t detector_learn_pct(const detector_t *d);

/* u-law byte -> int16 PCM (exposed for tests). */
int16_t detector_ulaw_decode(uint8_t b);

#endif /* DETECTOR_H */
