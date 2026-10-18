/*
 * In libmirisdr: the half-band of the real to I/Q converter from Airspy (Youssef Touil,
 * MIT licence below), reworked for lpc4370sdr by Bertold Van den Bergh: vectorised, NEON
 * on ARM.
 *
 * Copied from lpc4370sdr-host 0.3.1, and cut down to the filter for hb_kernel.h's
 * kernels: the DC removal and the fs/4 translation are done in baseband.c.
 */
/*
Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
Copyright (c) 2026 Bertold Van den Bergh <vandenbergh@bertold.org> (modifications for lpc4370sdr)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "iqconverter_float.h"
#include "hb_kernel.h"
#include <stdlib.h>
#include <string.h>

#include <stdio.h>

#if defined(__MINGW32__) && !defined(__MINGW64_VERSION_MAJOR)
  #include <malloc.h>
  #define _aligned_malloc __mingw_aligned_malloc
  #define _aligned_free  __mingw_aligned_free
  #define _inline inline
#elif defined(__APPLE__)
  #include <malloc/malloc.h>
  #define _aligned_malloc(size, alignment) malloc(size)
  #define _aligned_free(mem) free(mem)
  #define _inline inline
#elif defined(__FreeBSD__)
  #define _inline inline
  #define _aligned_free(mem) free(mem)
static void *_aligned_malloc(size_t size, size_t alignment)
{
    void *result;
    if (posix_memalign(&result, alignment, size) == 0)
        return result;
    return 0;
}
#elif defined(__GNUC__) && !defined(__MINGW64_VERSION_MAJOR)
  #include <malloc.h>
  #define _aligned_malloc(size, alignment) memalign(alignment, size)
  #define _aligned_free(mem) free(mem)
  #define _inline inline
#endif

/* The FIR and the delay line work on blocks of FIR_BLOCK I/Q pairs */
#define FIR_BLOCK 128

/* GCC and Clang vector extensions: SSE on x86, NEON on ARM, without intrinsics. On ARM, NEON
   intrinsics also split and merge the I/Q pairs around the FIR, four pairs at a time. */
#if defined(__GNUC__) || defined(__clang__)
#define HAVE_V4SF
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define HAVE_NEON
typedef float32x4_t v4sf;
#else
typedef float v4sf __attribute__((vector_size(16)));
#endif
static _inline v4sf load4(const float *p) { v4sf v; memcpy(&v, p, sizeof(v)); return v; }
static _inline void store4(float *p, v4sf v) { memcpy(p, &v, sizeof(v)); }
#endif
#define DEFAULT_ALIGNMENT 16

static void fir_last(iqconverter_float_t *cnv, float *samples, int len);
static void fir_mid(iqconverter_float_t *cnv, float *samples, int len);
static void fir_early(iqconverter_float_t *cnv, float *samples, int len);

static iqconverter_float_t *iqconverter_float_create(const float *hb_kernel, int len)
{
	iqconverter_float_t *cnv;
	void (*fir)(iqconverter_float_t *, float *, int);

	/* one of hb_kernel.h's, for which the filter is made */
	if (hb_kernel == mirisdr_hb_last && len == MIRISDR_HB_LAST_LEN) fir = fir_last;
	else if (hb_kernel == mirisdr_hb_mid && len == MIRISDR_HB_MID_LEN) fir = fir_mid;
	else if (hb_kernel == mirisdr_hb_early && len == MIRISDR_HB_EARLY_LEN) fir = fir_early;
	else return NULL;

	cnv = (iqconverter_float_t *) _aligned_malloc(sizeof(iqconverter_float_t), DEFAULT_ALIGNMENT);
	if (cnv == NULL)
	{
		return NULL;
	}

	cnv->len = len / 2 + 1;
	cnv->hbc = hb_kernel[len / 2];
	cnv->fir = fir;

	cnv->fir_queue = (float *) _aligned_malloc((cnv->len - 1 + FIR_BLOCK) * sizeof(float), DEFAULT_ALIGNMENT);
	cnv->delay_line = (float *) _aligned_malloc((cnv->len / 2 + FIR_BLOCK) * sizeof(float), DEFAULT_ALIGNMENT);
	if (cnv->fir_queue == NULL || cnv->delay_line == NULL)
	{
		iqconverter_float_free(cnv);
		return NULL;
	}

	iqconverter_float_reset(cnv);

	return cnv;
}

static void iqconverter_float_free(iqconverter_float_t *cnv)
{
	if (cnv == NULL)
	{
		return;
	}
	if (cnv->fir_queue != NULL)
		_aligned_free(cnv->fir_queue);
	if (cnv->delay_line != NULL)
		_aligned_free(cnv->delay_line);
	_aligned_free(cnv);
}

static void iqconverter_float_reset(iqconverter_float_t *cnv)
{
	cnv->phase = 0;
	memset(cnv->delay_line, 0, (cnv->len / 2 + FIR_BLOCK) * sizeof(float));
	memset(cnv->fir_queue, 0, (cnv->len - 1 + FIR_BLOCK) * sizeof(float));
}

/*
 * The half-band filter: FIR on the even (I) samples, delay line on the odd (Q) samples.
 * fir_queue holds the last len - 1 I inputs in time order followed by the block being
 * filtered, delay_line the last len / 2 Q inputs followed by the block. Outputs are computed
 * FIR_GROUP at a time with every tap applied to the whole group, so the accumulators stay in
 * (vector) registers and no output waits on a chain of dependent multiply-adds.
 *
 * One for each of hb_kernel.h's kernels, made by the compiler for it: its taps are
 * constants, and as the kernel is symmetric each pair of inputs it weighs alike is added
 * before the multiply, half the multiplies. The I taps are the kernel's even ones.
 */
#define FIR_GROUP 16

#ifdef HAVE_V4SF
#define FIR_V4_GROUP(KERN, T)                                                           \
		for (i = 0; i + FIR_GROUP <= m; i += FIR_GROUP)                                 \
		{                                                                               \
			v4sf a0 = { 0 }, a1 = { 0 }, a2 = { 0 }, a3 = { 0 };                        \
			for (k = 0; k < (T) / 2; k++)                                               \
			{                                                                           \
				const float h = KERN[2 * k];                                            \
				const float *xa = x + (T) - 1 - k + i, *xb = x + k + i;                 \
				a0 += h * (load4(xa) + load4(xb));                                      \
				a1 += h * (load4(xa + 4) + load4(xb + 4));                              \
				a2 += h * (load4(xa + 8) + load4(xb + 8));                              \
				a3 += h * (load4(xa + 12) + load4(xb + 12));                            \
			}                                                                           \
			FIR_V4_STORE                                                                \
		}
#ifdef HAVE_NEON
#define FIR_V4_STORE                                                                    \
			{                                                                           \
				float32x4x2_t iq;                                                       \
				iq.val[0] = a0; iq.val[1] = vld1q_f32(q + i);                           \
				vst2q_f32(out + 2 * i, iq);                                             \
				iq.val[0] = a1; iq.val[1] = vld1q_f32(q + i + 4);                       \
				vst2q_f32(out + 2 * i + 8, iq);                                         \
				iq.val[0] = a2; iq.val[1] = vld1q_f32(q + i + 8);                       \
				vst2q_f32(out + 2 * i + 16, iq);                                        \
				iq.val[0] = a3; iq.val[1] = vld1q_f32(q + i + 12);                      \
				vst2q_f32(out + 2 * i + 24, iq);                                        \
			}
#else
#define FIR_V4_STORE                                                                    \
			{                                                                           \
				float acc[FIR_GROUP];                                                   \
				int j;                                                                  \
				store4(acc, a0); store4(acc + 4, a1);                                   \
				store4(acc + 8, a2); store4(acc + 12, a3);                              \
				for (j = 0; j < FIR_GROUP; j++)                                         \
				{                                                                       \
					out[2 * (i + j)] = acc[j];                                          \
					out[2 * (i + j) + 1] = q[i + j];                                    \
				}                                                                       \
			}
#endif
#else
#define FIR_V4_GROUP(KERN, T)
#endif

#ifdef HAVE_NEON
#define FIR_SPLIT                                                                       \
		for (; i + 4 <= m; i += 4)                                                      \
		{                                                                               \
			const float32x4x2_t iq = vld2q_f32(out + 2 * i);                            \
			vst1q_f32(x + taps - 1 + i, iq.val[0]);                                     \
			vst1q_f32(q + half + i, iq.val[1]);                                         \
		}
#else
#define FIR_SPLIT
#endif

#define FIR_FOR(NAME, KERN, LEN)                                                        \
static void NAME(iqconverter_float_t *cnv, float *samples, int len)                     \
{                                                                                       \
	enum { taps = (LEN) / 2 + 1, half = ((LEN) / 2 + 1) / 2 };                          \
	float *x = cnv->fir_queue;                                                          \
	float *q = cnv->delay_line;                                                         \
	int n = len / 2;                                                                    \
	int done, i, k;                                                                     \
                                                                                        \
	for (done = 0; done < n; done += FIR_BLOCK)                                         \
	{                                                                                   \
		int m = (n - done < FIR_BLOCK) ? n - done : FIR_BLOCK;                          \
		float *out = samples + 2 * done;                                                \
                                                                                        \
		i = 0;                                                                          \
		FIR_SPLIT                                                                       \
		for (; i < m; i++)                                                              \
		{                                                                               \
			x[taps - 1 + i] = out[2 * i];                                               \
			q[half + i] = out[2 * i + 1];                                               \
		}                                                                               \
		i = 0;                                                                          \
		FIR_V4_GROUP(KERN, taps)                                                        \
		for (; i < m; i++)                                                              \
		{                                                                               \
			float acc = 0.0f;                                                           \
			for (k = 0; k < taps / 2; k++)                                              \
			{                                                                           \
				acc += KERN[2 * k] * (x[taps - 1 - k + i] + x[k + i]);                  \
			}                                                                           \
			out[2 * i] = acc;                                                           \
			out[2 * i + 1] = q[i];                                                      \
		}                                                                               \
		memmove(x, x + m, (taps - 1) * sizeof(float));                                  \
		memmove(q, q + m, half * sizeof(float));                                        \
	}                                                                                   \
}

FIR_FOR(fir_last, mirisdr_hb_last, MIRISDR_HB_LAST_LEN)
FIR_FOR(fir_mid, mirisdr_hb_mid, MIRISDR_HB_MID_LEN)
FIR_FOR(fir_early, mirisdr_hb_early, MIRISDR_HB_EARLY_LEN)
