/*
 * kernels.h — the dataflow-fabric image kernels.
 *
 * Everything here is data-parallel work over contiguous arrays: the
 * shape the E1's spatial fabric wants.  The branchy sequential logic
 * (argmax, interpolation, calibration) lives in gauge.c on the
 * control core.
 *
 * Per-frame fabric pipeline:
 *   gk_blur3x3     denoise the webcam frame (weighted 3x3, >>4)
 *   gk_pixel_sum   brightness reduction (-> mean, the darkness ref)
 *   gk_ray_scores  per-angle needle evidence via the ray gather table
 */
#ifndef KERNELS_H
#define KERNELS_H

#include <stdint.h>

/* Fabric annotation: effcc's fabric subtarget treats __efficient__ as
 * a keyword marking a function for dataflow-fabric compilation.  Every
 * other build neutralizes it: the SDK itself passes -D__efficient__=
 * for its scalar and native subtargets, and this covers the gcc
 * sim/test builds (EFF_BLD_FABRIC comes from setup_sdk.cmake). */
#if !defined(EFF_BLD_FABRIC) && !defined(__efficient__)
#define __efficient__
#endif

/* 3x3 weighted blur [1 2 1; 2 4 2; 1 2 1] >> 4.  Border rows/columns
 * are copied through unchanged. */
__efficient__ void gk_blur3x3(const uint8_t *restrict src,
                              uint8_t *restrict dst, int w, int h);

/* Sum of all n pixels (n <= 2^23 so the i32 cannot overflow). */
__efficient__ void gk_pixel_sum(const uint8_t *restrict img, int n,
                                int32_t *restrict out_sum);

/* Needle evidence per candidate angle.  For each of n_angles rays,
 * sums max(0, sign*(px - mean)) over the n_radii samples the gather
 * table points at.  sign = -1: dark needle on a light face; +1: light
 * needle on a dark face.  Max score n_radii*255 = 6120, fits u16. */
__efficient__ void gk_ray_scores(const uint8_t *restrict img,
                                 const uint16_t *restrict ray_idx,
                                 int n_angles, int n_radii,
                                 int mean, int sign,
                                 uint16_t *restrict scores);

#endif /* KERNELS_H */
