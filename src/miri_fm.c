/*
 * Copy of rtl-sdr for Mirics devices
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c
 *
 * lots of locks, but that is okay
 * (no many-to-many locks)
 *
 * todo:
 *       sanity checks
 *       scale squelch to other input parameters
 *       test all the demodulations
 *       pad output on hop
 *       frequency ranges could be stored better
 *       scaled AM demod amplification
 *       auto-hop after time limit
 *       peak detector to tune onto stronger signals
 *       fifo for active hop frequency
 *       clips
 *       noise squelch
 *       merge stereo patch
 *       merge soft agc patch
 *       merge udp patch
 *       testmode to detect overruns
 *       break out convenience functions
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined (_WIN32) || defined(__MINGW32__)
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <pthread.h>

#include "mirisdr.h"

#include "convenience.c"

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_ASYNC_BUF_NUMBER	32
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN			-100
#define HOP_SETTLE_MS			20      /* dropped after a hop */

#define FREQUENCIES_LIMIT		1000

static volatile int do_exit = 0;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};

static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

struct dongle_state
{
	int      exit_flag;
	pthread_t thread;
	mirisdr_dev_t *dev;
	int      dev_index;
	uint32_t freq;
	uint32_t rate;
	int      gain;
	uint16_t buf16[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int      ppm_error;
	float    scale;         /* from the library's float to the int16 the demods take */
	int      rotate;        /* zero IF: the station lies a quarter of the rate below */
	int      direct_sampling;
	int      format;
	int      transfer;
	int      bw;
	int      if_mode;
	volatile int mute;      /* samples still to drop */
	struct demod_state *demod_target;
};

struct demod_state
{
	int      exit_flag;
	pthread_t thread;
	int16_t  lowpassed[MAXIMUM_BUF_LENGTH];
	int      lp_len;
	int16_t  lp_i_hist[10][6];
	int16_t  lp_q_hist[10][6];
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int16_t  droop_i_hist[9];
	int16_t  droop_q_hist[9];
	int      result_len;
	int      rate_in;
	int      rate_out;
	int      rate_out2;
	int      now_r, now_j;
	int      pre_r, pre_j;
	int      prev_index;
	int      downsample;    /* min 1, max 256 */
	int      post_downsample;
	int      output_scale;
	int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int      downsample_passes;
	int      comp_fir_size;
	int      custom_atan;
	int      deemph, deemph_a;
	int      now_lpr;
	int      prev_lpr_index;
	int      dc_block, dc_avg;
	void     (*mode_demod)(struct demod_state*);
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	struct output_state *output_target;
};

struct output_state
{
	int      exit_flag;
	pthread_t thread;
	FILE     *file;
	char     *filename;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int      result_len;
	int      rate;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
};

struct controller_state
{
	int      exit_flag;
	pthread_t thread;
	uint32_t freqs[FREQUENCIES_LIMIT];
	int      freq_len;
	int      freq_now;
	int      edge;
	int      wb_mode;
	pthread_cond_t hop;
	pthread_mutex_t hop_m;
};

// multiple of these, eventually
struct dongle_state dongle;
struct demod_state demod;
struct output_state output;
struct controller_state controller;

void usage(void)
{
	fprintf(stderr,
		"miri_fm, a simple narrow band FM demodulator for Mirics based receivers\n\n"
		"Use:\tmiri_fm -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t    use multiple -f for scanning (requires squelch)\n"
		"\t    ranges supported, -f 118M:137M:25k\n"
		"\t[-M modulation (default: fm)]\n"
		"\t    fm, wbfm, raw, am, usb, lsb\n"
		"\t    wbfm == -M fm -s 170k -o 4 -A fast -r 32k -l 0 -E deemp\n"
		"\t    raw mode outputs 2x16 bit IQ pairs, at the rate it says\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-T board (default: from the USB IDs)]\n"
		"\t    0:       a plain MSi2500/MSi001 board\n"
		"\t    1:       SDRplay RSP1\n"
		"\t    2:       SDRplay RSP1B\n"
		"\t    3:       SDRplay RSPdx-R2\n"
		"\t    4:       SDRplay RSPduo\n"
		"\t[-g total tuner gain in 10 dB units: 4.5 is 45 dB, taken to the nearest the band has\n"
		"\t    (default: the library's, 43 dB)]\n"
		"\t[-m sample format (default: auto]\n"
		"\t    504:    S8 (fastest)\n"
		"\t    384:    S10 +2bits \n"
		"\t    336:    S12\n"
		"\t    252:    S14\n"
#if !defined (_WIN32) || defined(__MINGW32__)
		"\t[-e USB transfer mode (default: 1)]\n"
#else
		"\t[-e USB transfer mode (default: 2)]\n"
#endif
		"\t    1:      Isochronous (maximum 196.608 Mbit/s) \n"
		"\t    2:      Bulk (maximum 333Mbit/s, depends on controller type)\n"
		"\t[-i IF mode (default: auto)]\n"
		"\t    auto:    450 kHz, or 1620 kHz for a band wider than 480 kHz\n"
		"\t    0:       ZERO, at 1.3 MS/s or more, with a box filter\n"
		"\t    450000:  450 kHz\n"
		"\t    1620000: 1620kHz\n"
		"\t    2048000: 2048kHz\n"
		"\t    a low IF takes any rate: the demod runs at a whole multiple\n"
		"\t    of the audio rate, which it averages down to\n"
		"\t[-w BW mode (default: the widest the IF allows)]\n"
		"\t    200000:  200kHz, 450 kHz IF\n"
		"\t    300000:  300kHz, 450 kHz IF\n"
		"\t    600000:  600kHz, any low IF\n"
		"\t    1536000: 1536kHz, zero, 1620 or 2048 kHz IF\n"
		"\t    5000000: 5MHz, zero IF, 6, 7 and 8 MHz too\n"
		"\t    8000000: 8MHz, zero IF\n"
		"\t    a combination the tuner cannot do is refused\n"
		"\t[-l squelch_level (default: 0/off)]\n"
		"\t    rms of the channel, full scale is 16384\n"
		//"\t    for fm squelch is inverted\n"
		"\t[-o oversampling, 1 to 16 (default: 1, 4 recommended)]\n"
		"\t[-p ppm_error of the reference, corrects the tuning (default: 0, or from a serial\n"
		"\t    ending in \" <ppm>p\")]\n"
		"\t[-E enable_option (default: none)]\n"
		"\t    use multiple -E to enable multiple options\n"
		"\t    edge:   enable lower edge tuning\n"
		"\t    dc:     enable dc blocking filter\n"
		"\t    deemp:  enable de-emphasis filter\n"
		"\t    direct: enable direct sampling\n"
		"\t    offset: not supported, has no effect\n"
		"\tfilename ('-' means stdout)\n"
		"\t    omitting the filename also uses stdout\n\n"
		"Experimental options:\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"
		"\t[-t squelch_delay in buffers of about 20 ms (default: 10)]\n"
		"\t    +values will mute/scan, -values will exit\n"
		"\t[-F fir_size (default: off)]\n"
		"\t    enables low-leakage downsample filter\n"
		"\t    size can be 0 or 9.  0 has bad roll off\n"
		"\t    zero IF only, a low IF has its own filters\n"
		"\t[-A std/fast/lut choose atan math (default: std)]\n"
		//"\t[-C clip_path (default: off)\n"
		//"\t (create time stamped raw clips, requires squelch)\n"
		//"\t (path must have '\%s' and will expand to date_time_freq)\n"
		//"\t[-H hop_fifo (default: off)\n"
		//"\t (fifo will contain the active frequency)\n"
		"\n"
		"Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
		"\tmiri_fm ... | play -t raw -r 24k -es -b 16 -c 1 -V1 -\n"
		"\t           | aplay -r 24k -f S16_LE -t raw -c 1\n"
		"\t  -M wbfm  | play -r 32k ... \n"
		"\t  -s 22050 | multimon -t raw /dev/stdin\n\n");
	exit(1);
}

#if defined (_WIN32) && !defined(__MINGW32__)
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		mirisdr_cancel_async(dongle.dev);
		return TRUE;
	}
	return FALSE;
}
#else
/* No output here: with SIGPIPE from stderr it would raise the signal again for ever */
static void sighandler(int signum)
{
(void) signum;
	do_exit = 1;
	mirisdr_cancel_async(dongle.dev);
}
#endif

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
/* not once the end is set: its signal may have come before the wait */
#define safe_cond_wait(n, m) pthread_mutex_lock(m); if (!do_exit) pthread_cond_wait(n, m); pthread_mutex_unlock(m)

/* {length, coef, coef, coef}  and scaled by 2^15
   for now, only length 9, optimal way to get +85% bandwidth */
#define CIC_TABLE_MAX 10
int cic_9_tables[][10] = {
	{0,},
	{9, -156,  -97, 2798, -15489, 61019, -15489, 2798,  -97, -156},
	{9, -128, -568, 5593, -24125, 74126, -24125, 5593, -568, -128},
	{9, -129, -639, 6187, -26281, 77511, -26281, 6187, -639, -129},
	{9, -122, -612, 6082, -26353, 77818, -26353, 6082, -612, -122},
	{9, -120, -602, 6015, -26269, 77757, -26269, 6015, -602, -120},
	{9, -120, -582, 5951, -26128, 77542, -26128, 5951, -582, -120},
	{9, -119, -580, 5931, -26094, 77505, -26094, 5931, -580, -119},
	{9, -119, -578, 5921, -26077, 77484, -26077, 5921, -578, -119},
	{9, -119, -577, 5917, -26067, 77473, -26067, 5917, -577, -119},
	{9, -199, -362, 5303, -25505, 77489, -25505, 5303, -362, -199},
};


/* to int16 without wrapping round: full scale with gain is more than it holds */
static int16_t clip16(int v)
{
	return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v;
}

void rotate_90_s16(short *buf, uint32_t len)
/* 90 rotation is 1+0j, 0+1j, -1+0j, 0-1j
   or [0, 1, -3, 2, -4, -5, 7, -6] */
{
	uint32_t i;
	short tmp;
	for (i=0; i<len; i+=8) {
		/* uint8_t negation = 255 - x */
		tmp = -buf[i+3];
		buf[i+3] = buf[i+2];
		buf[i+2] = tmp;

		buf[i+4] = -buf[i+4];
		buf[i+5] = -buf[i+5];

		tmp = -buf[i+6];
		buf[i+6] = buf[i+7];
		buf[i+7] = tmp;
	}
}

void low_pass(struct demod_state *d)
/* simple square window FIR */
{
	int i=0, i2=0;
	while (i < d->lp_len) {
		d->now_r += d->lowpassed[i];
		d->now_j += d->lowpassed[i+1];
		i += 2;
		d->prev_index++;
		if (d->prev_index < d->downsample) {
			continue;
		}
		d->lowpassed[i2]   = d->now_r; // * d->output_scale;
		d->lowpassed[i2+1] = d->now_j; // * d->output_scale;
		d->prev_index = 0;
		d->now_r = 0;
		d->now_j = 0;
		i2 += 2;
	}
	d->lp_len = i2;
}

int low_pass_simple(int16_t *signal2, int len, int step)
// no wrap around, length must be multiple of step
{
	int i, i2, sum;
	for(i=0; i < len; i+=step) {
		sum = 0;
		for(i2=0; i2<step; i2++) {
			sum += (int)signal2[i + i2];
		}
		//signal2[i/step] = (int16_t)(sum / step);
		signal2[i/step] = clip16(sum);
	}
	signal2[i/step + 1] = signal2[i/step];
	return len / step;
}

void low_pass_real(struct demod_state *s)
/* down to rate_out2, a whole fraction of rate_out: the average of each m, carried
   over from buffer to buffer. In place */
{
	int i, i2 = 0;
	int m = s->rate_out / s->rate_out2;
	for (i = 0; i < s->result_len; i++) {
		s->now_lpr += s->result[i];
		if (++s->prev_lpr_index < m) {
			continue;}
		s->result[i2++] = (int16_t)((s->now_lpr + (s->now_lpr >= 0 ? m / 2 : -(m / 2))) / m);
		s->now_lpr = 0;
		s->prev_lpr_index = 0;
	}
	s->result_len = i2;
}

void fifth_order(int16_t *data, int length, int16_t *hist)
/* for half of interleaved data */
{
	int i;
	int16_t a, b, c, d, e, f;
	a = hist[1];
	b = hist[2];
	c = hist[3];
	d = hist[4];
	e = hist[5];
	f = data[0];
	/* a downsample should improve resolution, so don't fully shift */
	data[0] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	for (i=4; i<length; i+=4) {
		a = c;
		b = d;
		c = e;
		d = f;
		e = data[i-2];
		f = data[i];
		data[i/2] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	}
	/* archive */
	hist[0] = a;
	hist[1] = b;
	hist[2] = c;
	hist[3] = d;
	hist[4] = e;
	hist[5] = f;
}

void generic_fir(int16_t *data, int length, int *fir, int16_t *hist)
/* Okay, not at all generic.  Assumes length 9, fix that eventually. */
{
	int d, temp, sum;
	for (d=0; d<length; d+=2) {
		temp = data[d];
		sum = 0;
		sum += (hist[0] + hist[8]) * fir[1];
		sum += (hist[1] + hist[7]) * fir[2];
		sum += (hist[2] + hist[6]) * fir[3];
		sum += (hist[3] + hist[5]) * fir[4];
		sum +=            hist[4]  * fir[5];
		data[d] = sum >> 15 ;
		hist[0] = hist[1];
		hist[1] = hist[2];
		hist[2] = hist[3];
		hist[3] = hist[4];
		hist[4] = hist[5];
		hist[5] = hist[6];
		hist[6] = hist[7];
		hist[7] = hist[8];
		hist[8] = temp;
	}
}

/* define our own complex math ops
   because ARMv5 has no hardware float */

void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

int polar_discriminant(int ar, int aj, int br, int bj)
{
	int cr, cj;
	double angle;
	multiply(ar, aj, br, -bj, &cr, &cj);
	angle = atan2((double)cj, (double)cr);
	return (int)(angle / 3.14159 * (1<<14));
}

int fast_atan2(int y, int x)
/* pre scaled for int16 */
{
	int yabs, angle;
	int pi4=(1<<12), pi34=3*(1<<12);  // note pi = 1<<14
	if (x==0 && y==0) {
		return 0;
	}
	yabs = y;
	if (yabs < 0) {
		yabs = -yabs;
	}
	/* 64-bit: the products of full scale samples reach 2^29 */
	if (x >= 0) {
		angle = pi4  - (int)((int64_t)pi4 * ((int64_t)x-yabs) / ((int64_t)x+yabs));
	} else {
		angle = pi34 - (int)((int64_t)pi4 * ((int64_t)x+yabs) / ((int64_t)yabs-x));
	}
	if (y < 0) {
		return -angle;
	}
	return angle;
}

int polar_disc_fast(int ar, int aj, int br, int bj)
{
	int cr, cj;
	multiply(ar, aj, br, -bj, &cr, &cj);
	return fast_atan2(cj, cr);
}

int atan_lut_init(void)
{
	int i = 0;

	if (!(atan_lut = malloc(atan_lut_size * sizeof(int)))) {
		return -1;}

	for (i = 0; i < atan_lut_size; i++) {
		atan_lut[i] = (int) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
	}

	return 0;
}

int polar_disc_lut(int ar, int aj, int br, int bj)
{
	int cr, cj;
	int64_t x, x_abs;

	multiply(ar, aj, br, -bj, &cr, &cj);

	/* special cases */
	if (cr == 0 || cj == 0) {
		if (cr == 0 && cj == 0)
			{return 0;}
		if (cr == 0 && cj > 0)
			{return 1 << 13;}
		if (cr == 0 && cj < 0)
			{return -(1 << 13);}
		if (cj == 0 && cr > 0)
			{return 0;}
		if (cj == 0 && cr < 0)
			{return 1 << 14;}
	}

	/* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
	x = ((int64_t)cj * (1 << atan_lut_coef)) / cr;
	x_abs = x < 0 ? -x : x;

	if (x_abs >= atan_lut_size) {
		/* we can use linear range, but it is not necessary */
		return (cj > 0) ? 1<<13 : -(1<<13);
	}

	/* by the side of cr: x rounds to 0 at small angles, which says nothing of
	   which half it is in */
	if (cr > 0) {
		return (x >= 0) ? atan_lut[x] : -atan_lut[-x];}
	return (cj > 0) ? (1<<14) - atan_lut[x_abs] : atan_lut[x_abs] - (1<<14);
}

void fm_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	pcm = polar_discriminant(lp[0], lp[1],
		fm->pre_r, fm->pre_j);
	fm->result[0] = (int16_t)pcm;
	for (i = 2; i < (fm->lp_len-1); i += 2) {
		switch (fm->custom_atan) {
		case 0:
			pcm = polar_discriminant(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		case 1:
			pcm = polar_disc_fast(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		case 2:
			pcm = polar_disc_lut(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		}
		fm->result[i/2] = (int16_t)pcm;
	}
	fm->pre_r = lp[fm->lp_len - 2];
	fm->pre_j = lp[fm->lp_len - 1];
	fm->result_len = fm->lp_len/2;
}

void am_demod(struct demod_state *fm)
// todo, fix this extreme laziness
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		// hypot uses floats but won't overflow
		//r[i/2] = (int16_t)hypot(lp[i], lp[i+1]);
		pcm = lp[i] * lp[i];
		pcm += lp[i+1] * lp[i+1];
		r[i/2] = clip16((int)sqrt(pcm) * fm->output_scale);
	}
	fm->result_len = fm->lp_len/2;
	// lowpass? (3khz)  highpass?  (dc)
}

void usb_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = lp[i] + lp[i+1];
		r[i/2] = clip16(pcm * fm->output_scale);
	}
	fm->result_len = fm->lp_len/2;
}

void lsb_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = lp[i] - lp[i+1];
		r[i/2] = clip16(pcm * fm->output_scale);
	}
	fm->result_len = fm->lp_len/2;
}

void raw_demod(struct demod_state *fm)
{
	int i;
	for (i = 0; i < fm->lp_len; i++) {
		fm->result[i] = (int16_t)fm->lowpassed[i];
	}
	fm->result_len = fm->lp_len;
}

void deemph_filter(struct demod_state *fm)
{
	static int avg;  // cheating...
	int i, d;
	// de-emph IIR
	// avg = avg * (1 - alpha) + sample * alpha;
	for (i = 0; i < fm->result_len; i++) {
		d = fm->result[i] - avg;
		if (d > 0) {
			avg += (d + fm->deemph_a/2) / fm->deemph_a;
		} else {
			avg += (d - fm->deemph_a/2) / fm->deemph_a;
		}
		fm->result[i] = (int16_t)avg;
	}
}

void dc_block_filter(struct demod_state *fm)
{
	int i, avg;
	int64_t sum = 0;
	for (i=0; i < fm->result_len; i++) {
		sum += fm->result[i];
	}
	avg = sum / fm->result_len;
	avg = (avg + fm->dc_avg * 9) / 10;
	for (i=0; i < fm->result_len; i++) {
		fm->result[i] -= avg;
	}
	fm->dc_avg = avg;
}

int mad(int16_t *samples, int len, int step)
/* mean average deviation */
{
	int i=0, sum=0, ave=0;
	if (len == 0)
		{return 0;}
	for (i=0; i<len; i+=step) {
		sum += samples[i];
	}
	ave = sum / (len * step);
	sum = 0;
	for (i=0; i<len; i+=step) {
		sum += abs(samples[i] - ave);
	}
	return sum / (len / step);
}

int rms(int16_t *samples, int len, int step)
/* largely lifted from rtl_power */
{
	int i;
	int64_t p, t, s;
	double dc, err;

	/* 64-bit: a buffer of full scale squares passes 2^31 */
	p = t = 0;
	for (i=0; i<len; i+=step) {
		s = (int64_t)samples[i];
		t += s;
		p += s * s;
	}
	/* correct for dc offset in squares */
	dc = (double)(t*step) / (double)len;
	err = t * 2 * dc - dc * dc * len;

	return (int)sqrt((p-err) / len);
}

void full_demod(struct demod_state *d)
{
	int i, ds_p;
	int sr = 0;
	ds_p = d->downsample_passes;
	if (ds_p) {
		for (i=0; i < ds_p; i++) {
			fifth_order(d->lowpassed,   (d->lp_len >> i), d->lp_i_hist[i]);
			fifth_order(d->lowpassed+1, (d->lp_len >> i) - 1, d->lp_q_hist[i]);
		}
		d->lp_len = d->lp_len >> ds_p;
		/* droop compensation */
		if (d->comp_fir_size == 9 && ds_p <= CIC_TABLE_MAX) {
			generic_fir(d->lowpassed, d->lp_len,
				cic_9_tables[ds_p], d->droop_i_hist);
			generic_fir(d->lowpassed+1, d->lp_len-1,
				cic_9_tables[ds_p], d->droop_q_hist);
		}
	} else {
		low_pass(d);
	}
	/* power squelch */
	if (d->squelch_level) {
		sr = rms(d->lowpassed, d->lp_len, 1);
		if (sr < d->squelch_level) {
			d->squelch_hits++;
			for (i=0; i<d->lp_len; i++) {
				d->lowpassed[i] = 0;
			}
		} else {
			d->squelch_hits = 0;}
	}
	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return;
	}
	/* todo, fm noise squelch */
	// use nicer filter here too?
	if (d->post_downsample > 1) {
		d->result_len = low_pass_simple(d->result, d->result_len, d->post_downsample);}
	if (d->deemph) {
		deemph_filter(d);}
	if (d->dc_block) {
		dc_block_filter(d);}
	if (d->rate_out2 > 0) {
		low_pass_real(d);}
}

static void mirisdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	uint32_t i;
	struct dongle_state *s = ctx;
	const float *iq = (const float*) buf;
	int16_t *out = (int16_t*) s->buf16;
	/* a cancel before the stream ran was lost */
	if (do_exit) {
		mirisdr_cancel_async(dongle.dev);
		return;}
	if (!ctx) {
		return;}
	struct demod_state *d = s->demod_target;
	/* complex float from the library, the station at 0 Hz (or a quarter of the
	   rate below at zero IF) */
	len /= sizeof(float);
	if (len > MAXIMUM_BUF_LENGTH) {
		len = MAXIMUM_BUF_LENGTH;}
	if (s->mute > 0) {
		s->mute -= (int)len / 2;
		return;}
	for (i = 0; i < len; i++) {
		float v = iq[i] * s->scale;
		out[i] = v > 32767.0f ? 32767 : v < -32767.0f ? -32767 : (int16_t)lrintf(v);
	}
	if (s->rotate) {
		rotate_90_s16(out, len);}
	pthread_rwlock_wrlock(&d->rw);
	memcpy(d->lowpassed, s->buf16, 2*len);
	d->lp_len = len;
	pthread_rwlock_unlock(&d->rw);
	safe_cond_signal(&d->ready, &d->ready_m);
}

static volatile int read_result;

/* the stream ends only on a cancel: anything else ends the program too */
static void *dongle_thread_fn(void *arg)
{
	struct dongle_state *s = arg;
	read_result = mirisdr_read_async(s->dev, mirisdr_callback, s,
		DEFAULT_ASYNC_BUF_NUMBER, s->buf_len);
	do_exit = 1;
	return 0;
}

static void *demod_thread_fn(void *arg)
{
	struct demod_state *d = arg;
	struct output_state *o = d->output_target;
	while (!do_exit) {
		safe_cond_wait(&d->ready, &d->ready_m);
		pthread_rwlock_wrlock(&d->rw);
		full_demod(d);
		pthread_rwlock_unlock(&d->rw);
		if (d->exit_flag) {
			do_exit = 1;
		}
		if (d->squelch_level && d->squelch_hits > d->conseq_squelch) {
			d->squelch_hits = d->conseq_squelch + 1;  /* hair trigger */
			if (d->terminate_on_squelch) {
				do_exit = 1;
				break;}
			safe_cond_signal(&controller.hop, &controller.hop_m);
			continue;
		}
		pthread_rwlock_wrlock(&o->rw);
		memcpy(o->result, d->result, 2*d->result_len);
		o->result_len = d->result_len;
		pthread_rwlock_unlock(&o->rw);
		safe_cond_signal(&o->ready, &o->ready_m);
	}
	return 0;
}

static void *output_thread_fn(void *arg)
{
	struct output_state *s = arg;
	while (!do_exit) {
		// use timedwait and pad out under runs
		safe_cond_wait(&s->ready, &s->ready_m);
		pthread_rwlock_rdlock(&s->rw);
		fwrite(s->result, 2, s->result_len, s->file);
		pthread_rwlock_unlock(&s->rw);
	}
	return 0;
}

/* The low IFs, the least rate each gives (just over its converter rate / 512) and its
   widest filter */
static const struct { uint32_t hz, least, widest; } low_ifs[] = {
	{ 450000, 3516, 600000 }, { 1620000, 12657, 1536000 }, { 2048000, 13201, 1536000 } };

/* The capture for a demod rate of at least rate_in. The library gives any rate with
   a low IF, so the demod rate is the audio rate times the oversampling times a whole
   number: the audio then comes from it by averaging, with no resampling. A low IF
   gives the station at 0 Hz: by default the 450 kHz one where its filter passes the
   oversampled band, else 1620 kHz. Zero IF captures at 1.3 MS/s or more, a quarter
   of the rate off the station, and the box filter takes it down to rate_in */
static void optimal_settings(void)
{
	struct dongle_state *d = &dongle;
	struct demod_state *dm = &demod;
	int raw = dm->mode_demod == &raw_demod;
	uint32_t audio = (uint32_t)(dm->rate_out2 > 0 && !raw ? dm->rate_out2 : dm->rate_out);
	uint32_t unit = audio * (uint32_t)dm->post_downsample, least = 0, j;
	uint32_t band = (uint32_t)dm->rate_in / (uint32_t)dm->post_downsample;    /* -s */
	int i = -1, k;

	/* the IF: as asked, or by the band the demod takes in */
	if (d->if_mode > 0) {
		for (i = 0; i < 3 && low_ifs[i].hz != (uint32_t)d->if_mode; i++);
		if (i == 3) {
			fprintf(stderr, "No %d Hz IF.\n", d->if_mode);
			exit(1);}
	} else if (d->if_mode < 0) {
		i = (band <= 0.8 * low_ifs[0].widest) ? 0 : 1;}
	if (i >= 0) {
		least = low_ifs[i].least;}

	/* the least whole multiple that reaches the demod rate and the IF's least */
	j = ((uint32_t)dm->rate_in + unit - 1) / unit;
	if (j < 1) {
		j = 1;}
	while (unit * j < least) {
		j++;}
	dm->rate_in = (int)(unit * j);
	dm->rate_out = dm->rate_in / dm->post_downsample;
	dm->rate_out2 = (!raw && j > 1) ? (int)audio : -1;

	if (i >= 0) {
		if (dm->downsample_passes) {
			fprintf(stderr, "-F has no effect with a low IF.\n");}
		dm->downsample = 1;
		dm->downsample_passes = 0;
		d->if_mode = (int)low_ifs[i].hz;
		d->rate = (uint32_t)dm->rate_in;
		d->rotate = 0;
	} else {
		dm->downsample = (1300000 / dm->rate_in) + 1;
		if (dm->downsample_passes) {
			dm->downsample_passes = (int)log2(dm->downsample) + 1;
			dm->downsample = 1 << dm->downsample_passes;
		}
		d->if_mode = 0;
		d->rate = (uint32_t)(dm->downsample * dm->rate_in);
		d->rotate = 1;
	}

	/* full scale 16384 after the channel filter, the box filter's sum included */
	d->scale = 16384.0f / dm->downsample;
	dm->output_scale = 2;
	if (dm->mode_demod == &fm_demod) {
		dm->output_scale = 1;}

	/* about 20 ms a buffer, of whole groups of four for the rotation and of the
	   oversampling */
	k = 16 * lcm_post[dm->post_downsample];
	i = ((int)(d->rate / 50) / k) * k;
	if (i < k) {
		i = k;}
	if (i > MAXIMUM_BUF_LENGTH / 2) {
		i = (MAXIMUM_BUF_LENGTH / 2 / k) * k;}
	d->buf_len = (uint32_t)i * 2 * sizeof(float);
}

/* Where to tune for a station: a reference ppm_error high puts the LO as much
   above what it is told, so it is told less */
static uint32_t capture_freq(uint32_t freq)
{
	freq += controller.edge * demod.rate_in / 2;
	if (dongle.rotate) {
		freq += dongle.rate / 4;}
	return (uint32_t)llround((double)freq / (1.0 + dongle.ppm_error * 1e-6));
}

/* The first channel's tune and the stream, before any thread runs: the stream
   starts on them. Hops keep the gain in force, which follows the band */
static mirisdr_tune_config_t hop_tc;

static int dongle_setup(struct controller_state *s)
{
	static const char *formats[] = { "AUTO", "504_S8", "384_S16", "336_S16", "252_S16" };
	mirisdr_stream_config_t sc;
	mirisdr_stream_result_t sr;
	mirisdr_tune_config_t tc;
	mirisdr_tune_result_t tr;

	/* set up primary channel */
	dongle.freq = capture_freq(s->freqs[0]);
	if (dongle.direct_sampling) {
		verbose_direct_sampling(dongle.dev, 1);}

	if (dongle.rotate) {
		fprintf(stderr, "Oversampling input by: %ix.\n", demod.downsample);}
	fprintf(stderr, "Oversampling output by: %ix.\n", demod.post_downsample);
	fprintf(stderr, "Buffer size: %0.2fms\n",
		1000 * (float)dongle.buf_len / (2 * sizeof(float)) / (float)dongle.rate);

	mirisdr_get_tune(dongle.dev, 0, &tc, NULL);
	tc.frequency = dongle.freq;
	tc.if_freq = (uint32_t)dongle.if_mode;
	tc.bandwidth = dongle.bw;
	if (dongle.gain != AUTO_GAIN) {
		tc.gain.mode = MIRISDR_GAIN_TOTAL;
		tc.gain.total = dongle.gain;
	}
	if (mirisdr_tune(dongle.dev, 0, &tc, &tr) < 0) {
		fprintf(stderr, "Failed to tune.\n");
		return -1;
	}

	mirisdr_get_stream(dongle.dev, &sc, NULL);
	sc.baseband = 1;
	sc.rate = dongle.rate;
	sc.format = formats[dongle.format];
	if (dongle.transfer == 1) {
		sc.transfer = "ISOC";}
	if (dongle.transfer == 2) {
		sc.transfer = "BULK";}
	if (mirisdr_set_stream(dongle.dev, &sc, &sr) < 0) {
		fprintf(stderr, "Failed to set up the stream.\n");
		return -1;
	}
	mirisdr_get_tune(dongle.dev, 0, NULL, &tr);
	if (dongle.if_mode) {
		fprintf(stderr, "Sampling at %u S/s, %s, %d Hz IF, filtered down to %u S/s%s%s.\n",
			sr.adc_rate, sr.format, dongle.if_mode, sr.rate, sr.resampled ? ", resampled" : "",
			sr.nco ? ", with the NCO" : "");
	} else {
		fprintf(stderr, "Sampling at %u S/s, %s, zero IF.\n", sr.rate, sr.format);}
	if (sr.adc_clock > MIRISDR_ADC_CLOCK_BEST)
		fprintf(stderr, "The converters clock at %.1f MHz, past their best %.0f MHz: their resolution drops.\n",
		        sr.adc_clock / 1e6, MIRISDR_ADC_CLOCK_BEST / 1e6);
	if (demod.mode_demod == &raw_demod) {
		fprintf(stderr, "Output at %d Hz.\n", demod.rate_in);
	} else if (demod.rate_out2 > 0 && demod.rate_out2 != demod.rate_out) {
		fprintf(stderr, "Output at %d Hz, averaged down from %d Hz.\n", demod.rate_out2, demod.rate_out);
	} else {
		fprintf(stderr, "Output at %d Hz.\n", demod.rate_out);}
	fprintf(stderr, "Tuned to %u Hz, bandwidth %u Hz, tuner gain %d dB.\n", dongle.freq, tr.bandwidth, tr.gain.total);

	hop_tc = tc;
	hop_tc.gain.mode = MIRISDR_GAIN_KEEP;
	return 0;
}

static void *controller_thread_fn(void *arg)
{
	struct controller_state *s = arg;
	mirisdr_tune_config_t tc = hop_tc;

	while (!do_exit) {
		safe_cond_wait(&s->hop, &s->hop_m);
		if (s->freq_len <= 1) {
			continue;}
		/* hacky hopping */
		s->freq_now = (s->freq_now + 1) % s->freq_len;
		dongle.freq = capture_freq(s->freqs[s->freq_now]);
		tc.frequency = dongle.freq;
		if (mirisdr_tune(dongle.dev, 0, &tc, NULL) < 0) {
			fprintf(stderr, "Failed to tune to %u Hz.\n", dongle.freq);}
		dongle.mute = (int)(dongle.rate * HOP_SETTLE_MS / 1000);
	}
	return 0;
}

void frequency_range(struct controller_state *s, char *arg)
{
	char *start, *stop, *step;
	int i;
	start = arg;
	if (!(stop = strchr(start, ':')) || !(step = strchr(stop + 1, ':'))) {
		fprintf(stderr, "A range is start:stop:step, %s is not.\n", arg);
		exit(1);}
	stop++;
	step++;
	stop[-1] = '\0';
	step[-1] = '\0';
	if ((int)atofs(step) <= 0 || (int)atofs(stop) < (int)atofs(start)) {
		fprintf(stderr, "The range %s:%s:%s has no frequencies.\n", start, stop, step);
		exit(1);}
	for(i=(int)atofs(start); i<=(int)atofs(stop); i+=(int)atofs(step))
	{
		s->freqs[s->freq_len] = (uint32_t)i;
		s->freq_len++;
		if (s->freq_len >= FREQUENCIES_LIMIT) {
			break;}
	}
	stop[-1] = ':';
	step[-1] = ':';
}

void dongle_init(struct dongle_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	s->gain = AUTO_GAIN; /* dB, AUTO_GAIN: the library's */
	s->mute = 0;
	s->direct_sampling = 0;
	s->demod_target = &demod;
	s->format = 0;
#if !defined (_WIN32) || defined(__MINGW32__)
	s->transfer = 1;
#else
	s->transfer = 2;
#endif
	s->bw = 0;              /* the widest the IF allows */
	s->if_mode = -1;        /* auto */
}

void demod_init(struct demod_state *s)
{
	s->rate_in = DEFAULT_SAMPLE_RATE;
	s->rate_out = DEFAULT_SAMPLE_RATE;
	s->squelch_level = 0;
	s->conseq_squelch = 10;
	s->terminate_on_squelch = 0;
	s->squelch_hits = 11;
	s->downsample_passes = 0;
	s->comp_fir_size = 0;
	s->prev_index = 0;
	s->post_downsample = 1;  // once this works, default = 4
	s->custom_atan = 0;
	s->deemph = 0;
	s->rate_out2 = -1;  // flag for disabled
	s->mode_demod = &fm_demod;
	s->pre_j = s->pre_r = s->now_r = s->now_j = 0;
	s->prev_lpr_index = 0;
	s->deemph_a = 0;
	s->now_lpr = 0;
	s->dc_block = 0;
	s->dc_avg = 0;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
	s->output_target = &output;
}

void demod_cleanup(struct demod_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void output_init(struct output_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void output_cleanup(struct output_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void controller_init(struct controller_state *s)
{
	s->freqs[0] = 100000000;
	s->freq_len = 0;
	s->edge = 0;
	s->wb_mode = 0;
	pthread_cond_init(&s->hop, NULL);
	pthread_mutex_init(&s->hop_m, NULL);
}

void controller_cleanup(struct controller_state *s)
{
	pthread_cond_destroy(&s->hop);
	pthread_mutex_destroy(&s->hop_m);
}

void sanity_checks(void)
{
	if (controller.freq_len == 0) {
		fprintf(stderr, "Please specify a frequency.\n");
		exit(1);
	}

	if (controller.freq_len >= FREQUENCIES_LIMIT) {
		fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
		exit(1);
	}

	if (controller.freq_len > 1 && demod.squelch_level == 0) {
		fprintf(stderr, "Please specify a squelch level.  Required for scanning multiple frequencies.\n");
		exit(1);
	}

}

int main(int argc, char **argv)
{
#if !defined (_WIN32) || defined(__MINGW32__)
	struct sigaction sigact;
#endif
	int r, opt;
	int dev_given = 0;
	int custom_ppm = 0;
	dongle_init(&dongle);
	demod_init(&demod);
	output_init(&output);
	controller_init(&controller);
    mirisdr_hw_flavour_t hw_flavour = MIRISDR_HW_AUTO;
    int intval;

	while ((opt = getopt(argc, argv, "b:d:T:e:f:g:i:l:m:o:p:r:s:t:w:E:F:A:M:h")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
        case 'T':
            intval = atoi(optarg);
            if ((intval < 0) || (intval > MIRISDR_HW_RSPDUO))
            {
                fprintf(stderr, "-T takes 0 to %d.\n", MIRISDR_HW_RSPDUO);
                exit(1);
            }
            hw_flavour = (mirisdr_hw_flavour_t) intval;
            break;
		case 'e':
			if ((strcmp("ISOC", optarg) == 0) ||
			    (strcmp("1", optarg) == 0)) {
				dongle.transfer = 1;}
			if ((strcmp("BULK", optarg) == 0) ||
			    (strcmp("2", optarg) == 0)) {
				dongle.transfer = 2;}
			break;
		case 'f':
			if (controller.freq_len >= FREQUENCIES_LIMIT) {
				break;}
			if (strchr(optarg, ':'))
				{frequency_range(&controller, optarg);}
			else
			{
				controller.freqs[controller.freq_len] = (uint32_t)atofs(optarg);
				controller.freq_len++;
			}
			break;
		case 'g':
			dongle.gain = (int)lround(atof(optarg) * 10);
			break;
		case 'i':
			dongle.if_mode = strcmp(optarg, "auto") ? atoi(optarg) : -1;
			break;
		case 'l':
			demod.squelch_level = (int)atof(optarg);
			break;
		case 'm':
			if (strcmp("504", optarg) == 0) {
				dongle.format = 1;}
			if (strcmp("384", optarg) == 0) {
				dongle.format = 2;}
			if (strcmp("336", optarg) == 0) {
				dongle.format = 3;}
			if (strcmp("252", optarg) == 0) {
				dongle.format = 4;}
			if (strcmp("auto", optarg) == 0) {
				dongle.format = 0;}
			else if (!dongle.format) {
				fprintf(stderr, "-m %s is not a format.\n", optarg);
				exit(1);}
			break;
		case 's':
			demod.rate_in = (int)atofs(optarg);
			demod.rate_out = (int)atofs(optarg);
			if (demod.rate_in <= 0) {
				fprintf(stderr, "-s needs a rate.\n");
				exit(1);}
			break;
		case 'r':
			output.rate = (int)atofs(optarg);
			demod.rate_out2 = (int)atofs(optarg);
			if (demod.rate_out2 <= 0) {
				fprintf(stderr, "-r needs a rate.\n");
				exit(1);}
			break;
		case 'o':
			demod.post_downsample = (int)atof(optarg);
			if (demod.post_downsample < 1 || demod.post_downsample > MAXIMUM_OVERSAMPLE) {
				fprintf(stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE);
				exit(1);}
			break;
		case 't':
			demod.conseq_squelch = (int)atof(optarg);
			if (demod.conseq_squelch < 0) {
				demod.conseq_squelch = -demod.conseq_squelch;
				demod.terminate_on_squelch = 1;
			}
			break;
		case 'p':
			dongle.ppm_error = atoi(optarg);
			custom_ppm = 1;
			break;
		case 'w':
			dongle.bw = atoi(optarg);
			break;
		case 'E':
			if (strcmp("edge",  optarg) == 0) {
				controller.edge = 1;}
			if (strcmp("dc", optarg) == 0) {
				demod.dc_block = 1;}
			if (strcmp("deemp",  optarg) == 0) {
				demod.deemph = 1;}
			if (strcmp("direct",  optarg) == 0) {
				dongle.direct_sampling = 1;}
			if (strcmp("offset",  optarg) == 0) {
				fprintf(stderr, "Warning: -E offset is not supported and has no effect.\n");}
			break;
		case 'F':
			demod.downsample_passes = 1;  /* truthy placeholder */
			demod.comp_fir_size = atoi(optarg);
			break;
		case 'A':
			if (strcmp("std",  optarg) == 0) {
				demod.custom_atan = 0;}
			if (strcmp("fast", optarg) == 0) {
				demod.custom_atan = 1;}
			if (strcmp("lut",  optarg) == 0) {
				if (atan_lut_init() < 0) {
					fprintf(stderr, "Out of memory for the atan table.\n");
					exit(1);}
				demod.custom_atan = 2;}
			break;
		case 'M':
			if (strcmp("fm",  optarg) == 0) {
				demod.mode_demod = &fm_demod;}
			if (strcmp("raw",  optarg) == 0) {
				demod.mode_demod = &raw_demod;}
			if (strcmp("am",  optarg) == 0) {
				demod.mode_demod = &am_demod;}
			if (strcmp("usb", optarg) == 0) {
				demod.mode_demod = &usb_demod;}
			if (strcmp("lsb", optarg) == 0) {
				demod.mode_demod = &lsb_demod;}
			if (strcmp("wbfm",  optarg) == 0) {
				controller.wb_mode = 1;
				demod.mode_demod = &fm_demod;
				demod.rate_in = 170000;
				demod.rate_out = 170000;
				demod.rate_out2 = 32000;
				demod.custom_atan = 1;
				demod.post_downsample = 4;
				demod.deemph = 1;
				demod.squelch_level = 0;}
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	demod.rate_in *= demod.post_downsample;

	if (!output.rate) {
		output.rate = demod.rate_out;}

	optimal_settings();

	sanity_checks();

	if (controller.freq_len > 1) {
		demod.terminate_on_squelch = 0;}

	if (argc <= optind) {
		output.filename = "-";
	} else {
		output.filename = argv[optind];
	}


	if (!dev_given) {
		dongle.dev_index = verbose_device_search("0");
	}

	if (dongle.dev_index < 0) {
		exit(1);
	}

	{
		mirisdr_open_config_t cfg;

		mirisdr_open_config_default(&cfg);
		cfg.index = (uint32_t)dongle.dev_index;
		cfg.hw_flavour = hw_flavour;
		r = mirisdr_open_ex(&dongle.dev, &cfg);
	}
	if (r < 0) {
		fprintf(stderr, "Failed to open Mirics device #%d.\n", dongle.dev_index);
		exit(1);
	}


#if !defined (_WIN32) || defined(__MINGW32__)
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	if (demod.deemph) {
		demod.deemph_a = (int)round(1.0/((1.0-exp(-1.0/(demod.rate_out * 75e-6)))));
	}

	if (!custom_ppm) {
		verbose_ppm_eeprom(dongle.dev, &(dongle.ppm_error));
	}
	if (dongle.ppm_error) {
		fprintf(stderr, "Tuning corrected for a reference %d ppm off.\n", dongle.ppm_error);}

	if (strcmp(output.filename, "-") == 0) { /* Write samples to stdout */
		output.file = stdout;
#if defined (_WIN32) && !defined(__MINGW32__)
		_setmode(_fileno(output.file), _O_BINARY);
#endif
	} else {
		output.file = fopen(output.filename, "wb");
		if (!output.file) {
			fprintf(stderr, "Failed to open %s\n", output.filename);
			exit(1);
		}
	}

	//r = mirisdr_set_testmode(dongle.dev, 1);

	if (dongle_setup(&controller) < 0) {
		mirisdr_close(dongle.dev);
		exit(1);
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dongle.dev);

	pthread_create(&controller.thread, NULL, controller_thread_fn, (void *)(&controller));
	pthread_create(&output.thread, NULL, output_thread_fn, (void *)(&output));
	pthread_create(&demod.thread, NULL, demod_thread_fn, (void *)(&demod));
	pthread_create(&dongle.thread, NULL, dongle_thread_fn, (void *)(&dongle));

	while (!do_exit) {
		usleep(100000);
	}

	r = read_result;
	if (r < 0) {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}
	else {
		fprintf(stderr, "\nUser cancel, exiting...\n");}

	mirisdr_cancel_async(dongle.dev);
	pthread_join(dongle.thread, NULL);
	safe_cond_signal(&demod.ready, &demod.ready_m);
	pthread_join(demod.thread, NULL);
	safe_cond_signal(&output.ready, &output.ready_m);
	pthread_join(output.thread, NULL);
	safe_cond_signal(&controller.hop, &controller.hop_m);
	pthread_join(controller.thread, NULL);

	//dongle_cleanup(&dongle);
	demod_cleanup(&demod);
	output_cleanup(&output);
	controller_cleanup(&controller);

	if (output.file != stdout) {
		fclose(output.file);}

	mirisdr_close(dongle.dev);
	return (r < 0) ? 1 : 0;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
