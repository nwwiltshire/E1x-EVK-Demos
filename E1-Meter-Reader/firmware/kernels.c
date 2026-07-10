/*
 * kernels.c — fabric image kernels for the meter reader.
 *
 * No libc anywhere in this file: every function body must be
 * compilable for the dataflow fabric.  Plain loops over contiguous
 * arrays, restrict pointers, const tables passed in as arguments.
 */
#include "kernels.h"

__efficient__ void gk_blur3x3(const uint8_t *restrict src,
                              uint8_t *restrict dst, int w, int h)
{
    /* border: pass through (no memset/memcpy on the fabric) */
    for (int x = 0; x < w; x++) {
        dst[x] = src[x];
        dst[(h - 1) * w + x] = src[(h - 1) * w + x];
    }
    for (int y = 1; y < h - 1; y++) {
        dst[y * w] = src[y * w];
        dst[y * w + w - 1] = src[y * w + w - 1];
    }

    for (int y = 1; y < h - 1; y++) {
        const uint8_t *r0 = src + (y - 1) * w;
        const uint8_t *r1 = src + y * w;
        const uint8_t *r2 = src + (y + 1) * w;
        uint8_t *o = dst + y * w;

        for (int x = 1; x < w - 1; x++) {
            int acc = (int)r0[x - 1] + 2 * (int)r0[x] + (int)r0[x + 1]
                    + 2 * (int)r1[x - 1] + 4 * (int)r1[x] + 2 * (int)r1[x + 1]
                    + (int)r2[x - 1] + 2 * (int)r2[x] + (int)r2[x + 1];
            o[x] = (uint8_t)(acc >> 4);
        }
    }
}

__efficient__ void gk_pixel_sum(const uint8_t *restrict img, int n,
                                int32_t *restrict out_sum)
{
    int32_t acc = 0;

    for (int i = 0; i < n; i++)
        acc += img[i];
    out_sum[0] = acc;
}

__efficient__ void gk_ray_scores(const uint8_t *restrict img,
                                 const uint16_t *restrict ray_idx,
                                 int n_angles, int n_radii,
                                 int mean, int sign,
                                 uint16_t *restrict scores)
{
    for (int a = 0; a < n_angles; a++) {
        const uint16_t *ray = ray_idx + a * n_radii;
        int acc = 0;

        for (int r = 0; r < n_radii; r++) {
            int d = sign * ((int)img[ray[r]] - mean);

            if (d > 0)
                acc += d;
        }
        scores[a] = (uint16_t)acc; /* bounded by n_radii*255 = 6120 */
    }
}
