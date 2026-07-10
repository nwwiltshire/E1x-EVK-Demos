#include "dsp.h"

/* No libc anywhere in this file: every function body must be
 * compilable for the dataflow fabric. */

uint8_t dsp_log2u8(uint32_t v)
{
    /* floor(log2 v) by fixed binary search (no data-dependent loop
     * bound — friendlier to the fabric than a while loop), then 3
     * mantissa bits: result = 8*log2(v) with < 1 unit of error. */
    uint32_t t = v;
    int m = 0;
    uint32_t f;

    if (v == 0)
        return 0;
    if (t >> 16) { m += 16; t >>= 16; }
    if (t >> 8)  { m += 8;  t >>= 8; }
    if (t >> 4)  { m += 4;  t >>= 4; }
    if (t >> 2)  { m += 2;  t >>= 2; }
    if (t >> 1)  { m += 1; }
    f = (m >= 3) ? (v >> (m - 3)) & 7u : (v << (3 - m)) & 7u;
    return (uint8_t)(((uint32_t)m << 3) | f);
}

__efficient__
void au_window(const int16_t *restrict pcm, const int16_t *restrict win,
               int16_t *restrict out, int n)
{
    for (int i = 0; i < n; i++)
        out[i] = (int16_t)(((int32_t)pcm[i] * win[i] + 16384) >> 15);
}

__efficient__
void au_bitrev_gather(const int16_t *restrict src, const uint16_t *restrict br,
                      int16_t *restrict re, int16_t *restrict im, int n)
{
    for (int i = 0; i < n; i++) {
        re[i] = src[br[i]];
        im[i] = 0;
    }
}

__efficient__
void au_fft(int16_t *restrict re, int16_t *restrict im,
            const int16_t *restrict twr, const int16_t *restrict twi, int n)
{
    for (int half = 1; half < n; half <<= 1) {
        int step = n / (2 * half); /* twiddle stride at this stage */

        for (int g = 0; g < n; g += 2 * half) {
            for (int k = 0; k < half; k++) {
                int j = k * step;
                int32_t wr = twr[j], wi = twi[j];
                int32_t xr = re[g + k + half], xi = im[g + k + half];
                /* |xr*wr| + |xi*wi| <= 2*32767^2 + rounding < 2^31 */
                int32_t tr = (xr * wr - xi * wi + 16384) >> 15;
                int32_t ti = (xr * wi + xi * wr + 16384) >> 15;
                int32_t ar = re[g + k], ai = im[g + k];

                re[g + k]        = (int16_t)((ar + tr) >> 1);
                im[g + k]        = (int16_t)((ai + ti) >> 1);
                re[g + k + half] = (int16_t)((ar - tr) >> 1);
                im[g + k + half] = (int16_t)((ai - ti) >> 1);
            }
        }
    }
}

__efficient__
void au_logmag(const int16_t *restrict re, const int16_t *restrict im,
               uint8_t *restrict spec, int nbins, int m2_floor)
{
    for (int i = 0; i < nbins; i++) {
        int32_t xr = re[i], xi = im[i];
        uint32_t m2 = (uint32_t)(xr * xr) + (uint32_t)(xi * xi)
                    + (uint32_t)m2_floor;

        spec[i] = dsp_log2u8(m2);
    }
}

__efficient__
void au_excess(const uint8_t *restrict spec, const uint16_t *restrict mu,
               const uint16_t *restrict dev, uint16_t *restrict excess,
               int nbins, int k_q4, int margin_q8)
{
    for (int i = 0; i < nbins; i++) {
        int32_t x = (int32_t)spec[i] << 8;
        int32_t trig = (int32_t)mu[i] + (((int32_t)k_q4 * dev[i]) >> 4)
                     + margin_q8;
        int32_t e = x - trig;

        excess[i] = (uint16_t)(e > 0 ? (e >> 8) : 0);
    }
}

__efficient__
void au_baseline_update(const uint8_t *restrict spec, uint16_t *restrict mu,
                        uint16_t *restrict dev, int nbins, int shift,
                        int dev_floor_q8)
{
    for (int i = 0; i < nbins; i++) {
        int32_t x = (int32_t)spec[i] << 8;
        int32_t m = mu[i];
        int32_t d = x - m;
        int32_t ad = d < 0 ? -d : d;
        int32_t dv = dev[i];

        m += d >> shift;
        dv += (ad - dv) >> shift;
        if (m < 0)
            m = 0;
        if (m > 0xFFFF)
            m = 0xFFFF;
        if (dv < dev_floor_q8)
            dv = dev_floor_q8;
        if (dv > 0xFFFF)
            dv = 0xFFFF;
        mu[i] = (uint16_t)m;
        dev[i] = (uint16_t)dv;
    }
}
