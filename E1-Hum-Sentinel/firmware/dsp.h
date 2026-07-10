/*
 * dsp.h — per-sample / per-bin audio DSP stages: window, bit-reversal,
 * fixed-point radix-2 FFT, log-magnitude, and the per-bin baseline
 * excess/update loops.
 *
 * Portable C99, integer-only, no allocation, no libc calls.  These are
 * the hot flat loops over contiguous arrays — the shape that maps onto
 * the E1 dataflow fabric — so every function here is annotated
 * __efficient__ and takes restrict pointers (callers must pass
 * non-overlapping buffers).  Const tables are passed in as arguments;
 * see fft_tables.h for the generated window/bitrev/twiddle tables.
 *
 * Fixed-point conventions:
 *   PCM and FFT re/im are int16.  Window and twiddles are Q15.  Every
 *   FFT stage halves its outputs, so a length-N transform carries an
 *   overall 1/N scaling and can never overflow.  Spectra are 8*log2
 *   units per u8 step (~0.75 dB); the baseline model holds them in
 *   Q8 (u16) for sub-unit EMA resolution.
 */
#ifndef DSP_H
#define DSP_H

#include <stdint.h>

/* Fabric annotation: effcc's fabric subtarget treats __efficient__ as
 * a keyword marking a function for dataflow-fabric compilation.  Every
 * other build neutralizes it: the SDK itself passes -D__efficient__=
 * for its scalar and native subtargets, and this covers the gcc
 * sim/test builds (EFF_BLD_FABRIC comes from setup_sdk.cmake). */
#if !defined(EFF_BLD_FABRIC) && !defined(__efficient__)
#define __efficient__
#endif

/* out[i] = (pcm[i] * win[i]) >> 15   (Q15 window) */
void au_window(const int16_t *restrict pcm, const int16_t *restrict win,
               int16_t *restrict out, int n);

/* re[i] = src[br[i]], im[i] = 0 — bit-reversal permutation feeding the
 * decimation-in-time FFT. */
void au_bitrev_gather(const int16_t *restrict src, const uint16_t *restrict br,
                      int16_t *restrict re, int16_t *restrict im, int n);

/* In-place radix-2 DIT FFT (input bit-reversed, output natural order).
 * twr/twi are the n/2 Q15 twiddles e^{-2*pi*i*j/n}.  Outputs are scaled
 * by 1/n (one >>1 per stage). */
void au_fft(int16_t *restrict re, int16_t *restrict im,
            const int16_t *restrict twr, const int16_t *restrict twi, int n);

/* spec[i] = dsp_log2u8(re[i]^2 + im[i]^2 + m2_floor) for the first
 * nbins bins.  The floor keeps near-empty bins from flapping in the
 * log domain: without it a 2-LSB arithmetic-noise wiggle swings an
 * empty bin's log by ~25 units and the baseline can never settle.
 * AU_MAG2_FLOOR compresses that to ~1 unit while a real signal
 * (mag^2 >> floor) is unaffected. */
#define AU_MAG2_FLOOR 256
void au_logmag(const int16_t *restrict re, const int16_t *restrict im,
               uint8_t *restrict spec, int nbins, int m2_floor);

/* Per-bin anomaly excess in whole log units:
 *   trigger = mu + k_q4*dev/16 + margin_q8      (all Q8)
 *   excess[i] = max(0, spec[i]*256 - trigger) / 256
 * The caller reduces excess[] to a score/argmax. */
void au_excess(const uint8_t *restrict spec, const uint16_t *restrict mu,
               const uint16_t *restrict dev, uint16_t *restrict excess,
               int nbins, int k_q4, int margin_q8);

/* Per-bin EMA baseline update (Q8):
 *   mu  += (spec*256 - mu)  >> shift
 *   dev += (|spec*256 - mu_old| - dev) >> shift,  floored at dev_floor_q8
 * Skipped entirely by the caller to freeze learning during an event. */
void au_baseline_update(const uint8_t *restrict spec, uint16_t *restrict mu,
                        uint16_t *restrict dev, int nbins, int shift,
                        int dev_floor_q8);

/* 8*log2(v) rounded down to u8 (exact 255 cap at v >= 2^31.875).
 * Plain helper (also used by unit tests); called from the fabric
 * functions, which is fine for un-annotated leaf helpers. */
uint8_t dsp_log2u8(uint32_t v);

#endif /* DSP_H */
