/*
 * In libmirisdr: the half-band of the real to I/Q converter from Airspy (Youssef Touil,
 * MIT licence below), as reworked for lpc4370sdr by Bertold Van den Bergh: vectorised,
 * NEON on ARM. Those changes are his, not Airspy's, so report problems here. Copied from
 * lpc4370sdr-host 0.3.1, and cut down to the filter: baseband.c takes the DC off and does
 * the fs/4 translation while it converts the samples to float.
 */
/*
Copyright (C) 2014, Youssef Touil <youssef@airspy.com>

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

#ifndef IQCONVERTER_FLOAT_H
#define IQCONVERTER_FLOAT_H

#include <stdint.h>

typedef struct iqconverter_float {
	float hbc;  /* the kernel's centre tap: the translation's weight on Q */
	int phase;  /* position of the next sample in the fs/4 translation's cycle of four,
	               kept by whoever translates */
	int len;    /* the I taps: the kernel's even ones */
	float *fir_queue;
	float *delay_line;
	/* the FIR, made for one of hb_kernel.h's kernels: n translated real samples in, the
	   I/Q pairs (n / 2) out, in place */
	void (*fir)(struct iqconverter_float *cnv, float *samples, int len);
} iqconverter_float_t;

static iqconverter_float_t *iqconverter_float_create(const float *hb_kernel, int len);
static void iqconverter_float_free(iqconverter_float_t *cnv);
static void iqconverter_float_reset(iqconverter_float_t *cnv);

#endif // IQCONVERTER_FLOAT_H
