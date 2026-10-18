/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
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

#include "hard.h"




/* samples in each 1 kB block, which sets what a rate costs the engine */
static uint32_t mirisdr_format_spp (int format)
{
	switch (format)
	{
	case MIRISDR_FORMAT_252_S16:      return 252;
	case MIRISDR_FORMAT_336_S16:      return 336;
	case MIRISDR_FORMAT_384_S16:      return 384;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:
	case MIRISDR_FORMAT_504_REAL_S16: return 504;
	case MIRISDR_FORMAT_672_REAL_S16: return 672;
	case MIRISDR_FORMAT_768_REAL_S16: return 768;
	}

	return 252;
}

/* Every divider whose VCO is in range, nearest the middle first. */
static unsigned mirisdr_pll_candidates (uint32_t pll_rate, uint64_t *out)
{
	uint64_t want = (MIRISDR_VCO_MIN + MIRISDR_VCO_MAX) / 2, i;
	unsigned n = 0, a, b;

	for (i = 4; i <= 16; i += 2)
	{
		uint64_t v = (uint64_t) pll_rate * i * 12;

		if (v > MIRISDR_VCO_MAX) break;
		if (v < MIRISDR_VCO_MIN) continue;

		out[n++] = i;
	}

	for (a = 0; a < n; a++)
		for (b = a + 1; b < n; b++)
		{
			uint64_t va = (uint64_t) pll_rate * out[a] * 12;
			uint64_t vb = (uint64_t) pll_rate * out[b] * 12;
			uint64_t da = (va > want) ? (va - want) : (want - va);
			uint64_t db = (vb > want) ? (vb - want) : (want - vb);

			if (db < da) { uint64_t t = out[a]; out[a] = out[b]; out[b] = t; }
		}

	return n;
}

/* parameters that require restart */
/* The converters a stream captures, MIRISDR_IQ_* */
static int mirisdr_stream_adc (mirisdr_dev_t *p)
{
    switch (p->format)
    {
    case MIRISDR_FORMAT_504_REAL_S16:
    case MIRISDR_FORMAT_672_REAL_S16:
    case MIRISDR_FORMAT_768_REAL_S16:
        return p->swap_iq ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I;
    default:
        return MIRISDR_IQ_BOTH;
    }
}

/* ------------------------------------------------------------------ */
/* the stream config                                                  */
/* ------------------------------------------------------------------ */

#define MIRISDR_STREAM_ADJUST   1       /* the single setters: clamp and warn rather than refuse, and
                                           no combination checks */
#define MIRISDR_STREAM_FORCE    2       /* write it all even if nothing changed */

static const struct { const char *name; int format; } mirisdr_formats[] = {
	{ "252_S16",      MIRISDR_FORMAT_252_S16 },
	{ "336_S16",      MIRISDR_FORMAT_336_S16 },
	{ "384_S16",      MIRISDR_FORMAT_384_S16 },
	{ "504_S16",      MIRISDR_FORMAT_504_S16 },
	{ "504_S8",       MIRISDR_FORMAT_504_S8 },
	{ "504_REAL_S16", MIRISDR_FORMAT_504_REAL_S16 },
	{ "672_REAL_S16", MIRISDR_FORMAT_672_REAL_S16 },
	{ "768_REAL_S16", MIRISDR_FORMAT_768_REAL_S16 },
};

/* The isochronous settings differ in how many 1kB slots they reserve in the microframe:
 * ISOC=3, ISOC1=1, ISOC2=2 (2 requires custom fw) */
static const struct { const char *name; int transfer; uint8_t alt; } mirisdr_transfers[] = {
	{ "BULK",  MIRISDR_TRANSFER_BULK, 3 },
	{ "ISOC",  MIRISDR_TRANSFER_ISOC, 1 },
	{ "ISOC1", MIRISDR_TRANSFER_ISOC, 2 },
	{ "ISOC2", MIRISDR_TRANSFER_ISOC, 4 },
};

/* ISOC is more stable but works only on Unix systems, MinGW builds included in Windows */
#if !defined (_WIN32)
#define MIRISDR_TRANSFER_DEFAULT        "ISOC"
#else
#define MIRISDR_TRANSFER_DEFAULT        "BULK"
#endif

static const char *mirisdr_decim_names[] = { "AUTO", "OFF", "ON" };     /* by MIRISDR_DECIMATION_BYPASS_* */

static int mirisdr_format_real (int format)
{
	return (format == MIRISDR_FORMAT_504_REAL_S16) || (format == MIRISDR_FORMAT_672_REAL_S16) ||
	       (format == MIRISDR_FORMAT_768_REAL_S16);
}

static const char *mirisdr_format_name (int format)
{
	unsigned i;

	for (i = 0; i < sizeof mirisdr_formats / sizeof mirisdr_formats[0]; i++)
		if (mirisdr_formats[i].format == format) return mirisdr_formats[i].name;

	return "";
}

/* the register 7 word of a format, without the swap, decimation and burst bits */
static uint32_t mirisdr_format_reg7 (int format)
{
	switch (format)
	{
	case MIRISDR_FORMAT_252_S16:      return 0x000014;
	case MIRISDR_FORMAT_336_S16:      return 0x000005;
	case MIRISDR_FORMAT_384_S16:      return 0x000025;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:       return 0x000c14;
	case MIRISDR_FORMAT_504_REAL_S16: return 0x000414;
	case MIRISDR_FORMAT_672_REAL_S16: return 0x000405;
	case MIRISDR_FORMAT_768_REAL_S16: return 0x000425;
	}

	return 0x000014;
}

/* the register 3 AGC field of a format */
static uint32_t mirisdr_format_agc (int format)
{
	switch (format)
	{
	case MIRISDR_FORMAT_336_S16:
	case MIRISDR_FORMAT_672_REAL_S16: return 0x05;
	case MIRISDR_FORMAT_384_S16:
	case MIRISDR_FORMAT_768_REAL_S16: return 0x09;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:       return 0x0d;
	default:                          return 0x01;
	}
}

/* "AUTO", "AUTO_REAL" or a packing; NULL is dflt */
static int mirisdr_parse_format (const char *v, const char *dflt, int *fauto, int *format, const char **canon)
{
	unsigned i;

	if (!v) v = dflt;

	if (!strcmp(v, "AUTO") || !strcmp(v, "AUTO_REAL"))
	{
		*fauto = strcmp(v, "AUTO") ? MIRISDR_FORMAT_AUTO_REAL : MIRISDR_FORMAT_AUTO_ON;
		*format = strcmp(v, "AUTO") ? MIRISDR_FORMAT_504_REAL_S16 : MIRISDR_FORMAT_252_S16;
		*canon = strcmp(v, "AUTO") ? "AUTO_REAL" : "AUTO";

		return 0;
	}

	for (i = 0; (i < sizeof mirisdr_formats / sizeof mirisdr_formats[0]) && strcmp(v, mirisdr_formats[i].name); i++);

	if (i == sizeof mirisdr_formats / sizeof mirisdr_formats[0])
	{
		fprintf(stderr, "unsupported format: %s\n", v);
		return -1;
	}

	*fauto = MIRISDR_FORMAT_AUTO_OFF;
	*format = mirisdr_formats[i].format;
	*canon = mirisdr_formats[i].name;

	return 0;
}

/* B/s the automatic choice plans on: what was asked for in bulk, the reservation (or
   less) in an isochronous mode */
static uint32_t mirisdr_stream_cap (int transfer, uint8_t alt, uint32_t usb_capacity)
{
	uint32_t res;

	if (transfer == MIRISDR_TRANSFER_BULK) return usb_capacity ? usb_capacity : MIRISDR_BULK_CAPACITY;

	res = 1024 * mirisdr_alt_burst(alt) * 8000;

	return (usb_capacity && usb_capacity < res) ? usb_capacity : res;
}

typedef struct mirisdr_stream_plan
{
	mirisdr_stream_config_t cfg;    /* as asked, the strings canonical, the rate reached */
	uint32_t rate, pll_rate, cap;
	int format, format_auto, decimation_bypass, decim, transfer, swap;
	uint8_t alt;
	uint64_t cand[8];
	unsigned ncand;
	int bb_path, bb_stages;         /* MIRISDR_BASEBAND_*, and its half-band stages */
	uint32_t bb_rate;               /* what the callback gets a second */
	uint32_t bb_if;                 /* Hz, a low IF path's IF (tuner 0's with two tuners) */
	uint32_t bb_if2;                /* Hz, tuner 1's IF with two tuners, 0 otherwise */
	int bb_ds;                      /* the IF is a directly sampled frequency (baseband.c) */
	int tuners;                     /* the tuners streamed, bit n tuner n, never 0 */
} mirisdr_stream_plan_t;

static int mirisdr_set_soft (mirisdr_dev_t *p);
static int mirisdr_bw_auto (int ifm, uint32_t rate);

/* the widest filter a low IF has, Hz */
static uint32_t mirisdr_if_widest_hz (uint32_t if_hz);

/* The least converter rate a low IF baseband stream takes: the IF's widest filter
   around the IF ends at 1 / 2.4 of it, short of half the rate by what the receiver's
   own filter (unless bypassed) and the tuner filter's skirts need. 450 kHz at 1.8 Msps,
   the closest of the 4 x IF rates, is just that. Direct sampling (ds_edge, the low
   pass's edge) the same with the whole low pass */
static uint64_t mirisdr_bb_conv_least (uint32_t if_hz, uint32_t ds_edge)
{
	if (ds_edge) return (2 * (uint64_t) ds_edge * 6 + 4) / 5;
	return ((2 * (uint64_t) if_hz + mirisdr_if_widest_hz(if_hz)) * 6 + 4) / 5;
}

/* The converter rate a low IF baseband stream prefers, the converters running lo to hi:
   4 x IF, where the shift takes the band to 0 Hz for free, up to MIRISDR_BB_PREFER_MAX;
   else the least the IF takes, the NCO moving the band the rest of the way. Direct
   sampling takes the least, so a retune moves only the NCO */
static uint64_t mirisdr_bb_conv_pref (uint32_t if_hz, uint32_t ds_edge, uint32_t lo, uint32_t hi)
{
	uint64_t r4 = 4 * (uint64_t) if_hz, need = mirisdr_bb_conv_least(if_hz, ds_edge);

	if (need < lo) need = lo;
	return (!ds_edge && r4 >= need && r4 <= MIRISDR_BB_PREFER_MAX && r4 <= hi) ? r4 : need;
}

/* The output rates a low IF takes: the decimation is under 512 (8 halvings, the one
   output's converter counted, and less than 2 more in the resampler), from just above
   the preferred converter rate over 512 to the converters' most (halved for one output) */
static void mirisdr_bb_span (uint32_t if_hz, uint32_t ds_edge, int real, uint32_t lo, uint32_t hi,
                             uint32_t *least, uint32_t *most)
{
	*least = (uint32_t) (mirisdr_bb_conv_pref(if_hz, ds_edge, lo, hi) / 512 + 1);
	*most = hi / (real ? 2 : 1);
}

/* Whether direct sampling as mode takes a tune at f: ON at the low pass's frequencies,
   AUTO when a baseband stream's band (f +- rate / 2) fits under it too (without baseband
   the frequency alone) */
static int mirisdr_ds_takes (int mode, const mirisdr_stream_config_t *s, uint32_t f)
{
	if (mode == MIRISDR_DS_ON) return f <= MIRISDR_DS_MAX_HZ;
	if (mode == MIRISDR_DS_AUTO) return (uint64_t) f + (s->baseband ? s->rate / 2 : 0) <= MIRISDR_DS_MAX_HZ;

	return 0;
}

/* The low pass direct sampling at f takes: the 500 kHz one when a baseband stream's
   band fits under it */
static uint32_t mirisdr_ds_edge (const mirisdr_stream_config_t *s, uint32_t f)
{
	return (s->baseband && (uint64_t) f + s->rate / 2 <= MIRISDR_DS_LP500_HZ) ? MIRISDR_DS_LP500_HZ
	                                                                        : MIRISDR_DS_MAX_HZ;
}

/* A low IF baseband stream's converter rate and k, its half-band stages after the shift
   or the converter (which halves once more with one output). The free path first: the
   converters at 4 x IF (up to MIRISDR_BB_PREFER_MAX) and the output that halved k times,
   the band at 0 Hz with no more. Any other output takes the preferred converter rate (or the output's own, if
   higher) and as many halvings as leave between 1 and 2 to go, which the resampler
   makes. Returns k with the converter rate in *conv, or -1. With adjust a rate out of
   reach becomes the nearest there is, in *rate */
static int mirisdr_bb_stages_for (uint32_t if_hz, uint32_t ds_edge, int real, uint32_t *rate, uint32_t lo,
                                  uint32_t hi, int adjust, uint32_t *conv)
{
	const uint64_t m = real ? 2 : 1;
	const int kmax = 8 - (real ? 1 : 0);
	uint64_t r4 = 4 * (uint64_t) if_hz, need = mirisdr_bb_conv_least(if_hz, ds_edge), x, r;
	uint32_t least, most;
	int k;

	if (need < lo) need = lo;
	mirisdr_bb_span(if_hz, ds_edge, real, lo, hi, &least, &most);

	if ((*rate < least) || (*rate > most)) {
		if (!adjust) return -1;
		*rate = (*rate < least) ? least : most;
	}
	x = *rate;

	/* not past MIRISDR_BB_PREFER_MAX either: 2048 kHz would clock the converters past
	   their best (MIRISDR_ADC_CLOCK_BEST) */
	for (k = 0; !ds_edge && (r4 <= MIRISDR_BB_PREFER_MAX) && (k <= kmax); k++)
		if (((x * m << k) == r4) && (r4 >= need) && (r4 <= hi)) {
			*conv = (uint32_t) r4;
			return k;
		}

	r = mirisdr_bb_conv_pref(if_hz, ds_edge, lo, hi);
	if (x * m > r) r = x * m;
	for (k = 0; (k < kmax) && ((x * m << (k + 1)) <= r); k++);

	*conv = (uint32_t) r;
	return k;
}

/* iq: the tuner outputs the tune runs, which a stream following it takes its kind from,
   if_hz: its IF, which a baseband stream takes its rate from. Direct sampling: iq is
   the I converter, if_hz the frequency, ds_edge the low pass's edge (0 otherwise) */
static int mirisdr_stream_plan (mirisdr_dev_t *p, const mirisdr_stream_config_t *c, int flags, int iq,
                                uint32_t if_hz, uint32_t ds_edge, mirisdr_stream_plan_t *pl)
{
	int adjust = flags & MIRISDR_STREAM_ADJUST, sauto, sformat, follow, snap;
	uint32_t rate_min, rate_max, spp;
	uint64_t most;
	unsigned i;

	pl->cfg = *c;
	pl->cfg.strict = c->strict ? 1 : 0;

	/* a rate out of reach: the nearest there is, unless strict */
	snap = adjust || !pl->cfg.strict;

	if (MIRISDR_RESERVED_SET(*c))
	{
		mirisdr_refuse(p, MIRISDR_RESERVED_MSG, "the stream config");
		return -1;
	}

	/* formats: one, or with follow_tune one for each kind */
	if ((mirisdr_parse_format(c->format, "AUTO", &pl->format_auto, &pl->format, &pl->cfg.format) < 0) ||
	    (mirisdr_parse_format(c->format_single, "AUTO_REAL", &sauto, &sformat, &pl->cfg.format_single) < 0))
		return -1;

	pl->cfg.follow_tune = c->follow_tune ? 1 : 0;
	pl->cfg.baseband = c->baseband ? 1 : 0;
	pl->swap = pl->cfg.swap_iq = c->swap_iq ? 1 : 0;
	pl->bb_path = MIRISDR_BASEBAND_OFF;
	pl->bb_stages = 0;
	pl->bb_rate = 0;
	pl->bb_if = 0;
	pl->bb_if2 = 0;
	pl->bb_ds = 0;

	/* the tuners: one of the receiver's, or on the RSPduo both, each a low IF on its own
	   converter with the baseband filters taking it to 0 Hz. iq and if_hz are the selected
	   tuner's tune, the other's is as last left */
	pl->tuners = c->tuners ? c->tuners : 1;
	if ((pl->tuners & ~((1 << mirisdr_tuners_of(p)) - 1)) || (pl->tuners < 0))
	{
		mirisdr_refuse(p, "unsupported tuners 0x%x: this receiver has %d\n", (unsigned) c->tuners,
		               mirisdr_tuners_of(p));
		return -1;
	}

	if (pl->tuners == 3)
	{
		const int other = !p->tuner_cur;
		uint32_t ifs[2];
		int iqs[2];

		ifs[p->tuner_cur] = if_hz;
		iqs[p->tuner_cur] = iq;
		ifs[other] = mirisdr_if_hz[p->tctx[other].if_freq];
		iqs[other] = p->tctx[other].tune_iq;

		if (!pl->cfg.baseband)
		{
			mirisdr_refuse(p, "two tuners need baseband\n");
			return -1;
		}

		if (!ifs[0] || !ifs[1] || ds_edge)
		{
			mirisdr_refuse(p, "two tuners need a low IF on each: tuner %d is at zero IF\n", ifs[0] ? 1 : 0);
			return -1;
		}

		if ((iqs[0] != MIRISDR_IQ_BOTH) || (iqs[1] != MIRISDR_IQ_BOTH))
		{
			mirisdr_refuse(p, "two tuners need both outputs of each tuner\n");
			return -1;
		}

		/* the swap would trade the converters, so the tuners: each chain the other's */
		if (pl->swap)
		{
			mirisdr_refuse(p, "two tuners do not swap I and Q\n");
			return -1;
		}

		pl->bb_if = ifs[0];
		pl->bb_if2 = ifs[1];
		iq = MIRISDR_IQ_BOTH;
	}

	/* baseband follows the tune's outputs too */
	follow = pl->cfg.follow_tune || pl->cfg.baseband;

	if (follow)
	{
		if ((pl->format_auto == MIRISDR_FORMAT_AUTO_REAL) || mirisdr_format_real(pl->format))
		{
			mirisdr_refuse(p, "following the tune, format is the complex one\n");
			return -1;
		}

		if ((sauto == MIRISDR_FORMAT_AUTO_ON) || !mirisdr_format_real(sformat))
		{
			mirisdr_refuse(p, "following the tune, format_single is a real one\n");
			return -1;
		}

		/* one tuner output: a real stream on its converter */
		if (iq != MIRISDR_IQ_BOTH)
		{
			pl->format_auto = sauto;
			pl->format = sformat;
			pl->swap = (iq == MIRISDR_IQ_ONLY_Q);
		}
	}

	/* transfer */
	if (!c->transfer) pl->cfg.transfer = MIRISDR_TRANSFER_DEFAULT;

	for (i = 0; (i < sizeof mirisdr_transfers / sizeof mirisdr_transfers[0]) &&
	            strcmp(pl->cfg.transfer, mirisdr_transfers[i].name); i++);

	if (i == sizeof mirisdr_transfers / sizeof mirisdr_transfers[0])
	{
		mirisdr_refuse(p, "unsupported transfer type: %s\n", pl->cfg.transfer);
		return -1;
	}

	pl->transfer = mirisdr_transfers[i].transfer;
	pl->alt = mirisdr_transfers[i].alt;
	pl->cfg.transfer = mirisdr_transfers[i].name;

	/* decimation */
	if (!c->decimation_bypass) pl->decimation_bypass = MIRISDR_DECIMATION_BYPASS_AUTO;
	else
	{
		for (i = 0; (i < 3) && strcmp(c->decimation_bypass, mirisdr_decim_names[i]); i++);

		if (i == 3)
		{
			mirisdr_refuse(p, "unsupported decimation bypass: %s\n", c->decimation_bypass);
			return -1;
		}

		pl->decimation_bypass = (int) i;
	}

	pl->cfg.decimation_bypass = mirisdr_decim_names[pl->decimation_bypass];

	pl->cfg.gap_fill = c->gap_fill ? 1 : 0;

	/* rate, within what the decimation setting allows */
	pl->rate = c->rate;

	/* two tuners: each IF on its own converter as one output would be, at a converter rate
	   that holds both, with the same output rate and stages for each, the NCO taking each IF the
	   rest of the way */
	if (pl->tuners == 3)
	{
		uint32_t lo = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ? 2 * MIRISDR_SAMPLE_RATE_MIN
		                                                                      : MIRISDR_SAMPLE_RATE_MIN;
		uint32_t hi = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_OFF) ? MIRISDR_SAMPLE_RATE_MAX
		                                                                       : 2 * MIRISDR_SAMPLE_RATE_MAX;
		uint32_t l0, m0, l1, m1, least, most, out = c->rate, o0, o1, cv0 = 0, cv1 = 0, conv;
		int k0, k1, k;

		mirisdr_bb_span(pl->bb_if, 0, 1, lo, hi, &l0, &m0);
		mirisdr_bb_span(pl->bb_if2, 0, 1, lo, hi, &l1, &m1);
		least = l0 > l1 ? l0 : l1;
		most = m0 < m1 ? m0 : m1;

		if ((out < least) || (out > most))
		{
			if (!snap || (least > most))
			{
				mirisdr_refuse(p, "rate %u is not a baseband rate for both tuners (%u and %u Hz IF): %u to %u sps\n",
				               c->rate, pl->bb_if, pl->bb_if2, least, most);
				return -1;
			}
			out = (out < least) ? least : most;
		}

		o0 = o1 = out;
		k0 = mirisdr_bb_stages_for(pl->bb_if, 0, 1, &o0, lo, hi, 0, &cv0);
		k1 = mirisdr_bb_stages_for(pl->bb_if2, 0, 1, &o1, lo, hi, 0, &cv1);
		if ((k0 < 0) || (k1 < 0))
		{
			mirisdr_refuse(p, "rate %u is not a baseband rate for both tuners (%u and %u Hz IF)\n", out,
			               pl->bb_if, pl->bb_if2);
			return -1;
		}

		/* the higher converter rate, and its stages for the output (as each chain is
		   one output, the converter halves first) */
		conv = cv0 > cv1 ? cv0 : cv1;
		k = (conv == cv0) ? k0 : k1;
		if (cv0 != cv1)
			for (k = 0; (k < 7) && (((uint64_t) out * 2 << (k + 1)) <= conv); k++);

		pl->bb_path = MIRISDR_BASEBAND_DUAL;
		pl->bb_stages = k;
		pl->bb_rate = out;
		pl->rate = conv;
	}
	/* baseband with a low IF: the converters at a rate the IF fits in, the rate asked for
	   after the filters */
	else if (pl->cfg.baseband && (if_hz || ds_edge))
	{
		int real = iq != MIRISDR_IQ_BOTH, k;
		uint32_t out = c->rate, conv = 0;
		uint32_t lo = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ? 2 * MIRISDR_SAMPLE_RATE_MIN
		                                                                      : MIRISDR_SAMPLE_RATE_MIN;
		uint32_t hi = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_OFF) ? MIRISDR_SAMPLE_RATE_MAX
		                                                                       : 2 * MIRISDR_SAMPLE_RATE_MAX;

		if ((k = mirisdr_bb_stages_for(if_hz, ds_edge, real, &out, lo, hi, snap, &conv)) < 0)
		{
			uint32_t least, most;

			mirisdr_bb_span(if_hz, ds_edge, real, lo, hi, &least, &most);
			if (ds_edge) mirisdr_refuse(p, "rate %u is not a direct sampling baseband rate: %u to %u sps\n",
			                            c->rate, least, most);
			else mirisdr_refuse(p, "rate %u is not a baseband rate with a %u Hz IF%s: %u to %u sps\n", c->rate,
			                    if_hz, real ? " and one output" : "", least, most);
			return -1;
		}

		pl->bb_path = real ? MIRISDR_BASEBAND_REAL : MIRISDR_BASEBAND_COMPLEX;
		pl->bb_stages = k;
		pl->bb_rate = out;
		pl->bb_if = if_hz;
		pl->bb_ds = ds_edge != 0;
		pl->rate = conv;
	}
	else if (pl->cfg.baseband) pl->bb_path = MIRISDR_BASEBAND_ZERO_IF;
	pl->decim = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ||
	            ((pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_AUTO) &&
	             (pl->rate > MIRISDR_DECIMATION_AUTO_RATE));

	rate_min = pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN;
	rate_max = pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MAX : MIRISDR_SAMPLE_RATE_MAX;

	if ((pl->rate < rate_min) || (pl->rate > rate_max))
	{
		/* a low IF baseband stream's converters run at 4 x IF, which the filters are
		   planned for: no other rate will do */
		if (!snap || (pl->bb_path >= MIRISDR_BASEBAND_COMPLEX))
		{
			mirisdr_refuse(p, "rate %u is outside %u to %u sps\n", pl->rate, rate_min, rate_max);
			return -1;
		}

		pl->rate = (pl->rate < rate_min) ? rate_min : rate_max;
	}

	pl->cap = mirisdr_stream_cap(pl->transfer, pl->alt, c->usb_capacity);

	/* the automatic choice: the least dense packing that fits */
	if (pl->format_auto != MIRISDR_FORMAT_AUTO_OFF)
	{
		static const struct { uint32_t spp; int format; } complex[] = {
			{ 252, MIRISDR_FORMAT_252_S16 }, { 336, MIRISDR_FORMAT_336_S16 },
			{ 384, MIRISDR_FORMAT_384_S16 }, { 504, MIRISDR_FORMAT_504_S16 } },
		real[] = {
			{ 504, MIRISDR_FORMAT_504_REAL_S16 }, { 672, MIRISDR_FORMAT_672_REAL_S16 },
			{ 768, MIRISDR_FORMAT_768_REAL_S16 } };
		int is_real = pl->format_auto == MIRISDR_FORMAT_AUTO_REAL;
		unsigned n = is_real ? 3 : 4;

		for (i = 0; i < n; i++)
		{
			spp = is_real ? real[i].spp : complex[i].spp;
			pl->format = is_real ? real[i].format : complex[i].format;
			if ((uint64_t) pl->rate * 1024 / spp <= pl->cap) break;
		}

		/* Nothing fits. Bulk carries what the host manages, so it takes the densest
		   (the loop ends on it): gaps tell if the host cannot keep up. An isochronous
		   reservation is a hard limit */
		if ((i == n) && (pl->transfer != MIRISDR_TRANSFER_BULK))
		{
			mirisdr_refuse(p, "rate %u needs %lu B/s, more than the %u B/s this mode supports\n", pl->rate,
			        (long unsigned) ((uint64_t) pl->rate * 1024 / spp), pl->cap);

			if (!adjust) return -1;
		}
	}
	/* an isochronous reservation is a hard limit, what bulk carries is up to the host */
	else if (!adjust && (pl->transfer != MIRISDR_TRANSFER_BULK) &&
	         ((uint64_t) pl->rate * 1024 / mirisdr_format_spp(pl->format) > pl->cap))
	{
		mirisdr_refuse(p, "rate %u in %s needs %lu B/s, more than the %u B/s this mode supports\n", pl->rate,
		        pl->cfg.format, (long unsigned) ((uint64_t) pl->rate * 1024 / mirisdr_format_spp(pl->format)),
		        pl->cap);
		return -1;
	}

	/* The format is settled now, so the engine's own limit can be applied: after the
	   automatic choice, which needs the rate, and before register 7, since clamping can
	   drop the rate back under the decimation threshold and change that bit. */
	spp = mirisdr_format_spp(pl->format);
	most = MIRISDR_ENGINE_BLOCK_RATE * spp / 1024;

	if ((uint64_t) pl->rate > most)
	{
		/* snapped quietly, the result says, and the single setters warn */
		if (!snap || adjust || (pl->bb_path >= MIRISDR_BASEBAND_COMPLEX))
		{
			mirisdr_refuse(p, "rate %u needs %lu B/s of blocks, more than the engine's %lu", pl->rate,
			        (long unsigned) ((uint64_t) pl->rate * 1024 / spp), (long unsigned) MIRISDR_ENGINE_BLOCK_RATE);
			if (snap && (pl->bb_path < MIRISDR_BASEBAND_COMPLEX)) mirisdr_refuse(p, ", using %lu", (long unsigned) most);
			mirisdr_refuse(p, "\n");
		}

		if (!snap || (pl->bb_path >= MIRISDR_BASEBAND_COMPLEX)) return -1;

		pl->rate = (uint32_t) most;

		/* the clamp only ever lowers it, which can turn decimation off */
		pl->decim = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ||
		            ((pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_AUTO) &&
		             (pl->rate > MIRISDR_DECIMATION_AUTO_RATE));

		if (pl->rate < (pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN))
			pl->rate = pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN;
	}

	pl->pll_rate = pl->decim ? pl->rate / 2 : pl->rate;
	if (pl->bb_path == MIRISDR_BASEBAND_ZERO_IF) pl->bb_rate = pl->rate;
	pl->cfg.rate = (pl->bb_path >= MIRISDR_BASEBAND_COMPLEX) ? pl->bb_rate : pl->rate;

	/* The sample clock: N of at least 2, or the rate cannot be switched back (see the
	   rate limits in hard.h) */
	if (!(pl->ncand = mirisdr_pll_candidates(pl->pll_rate, pl->cand)))
	{
		mirisdr_refuse(p, "no PLL divider puts the VCO in range for %u sps\n", pl->rate);
		return -1;
	}

	/* the tune switched off the output this would capture */
	if (!adjust && !follow && mirisdr_format_real(pl->format) && (iq != MIRISDR_IQ_BOTH) &&
	    ((pl->swap ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I) != iq))
	{
		mirisdr_refuse(p, "the tune switched off the tuner output this would capture\n");
		return -1;
	}

	return 0;
}

/* the bits a sample keeps in a format: its packing, complex or real alike */
static int mirisdr_format_bits (int format)
{
	switch (mirisdr_format_spp(format))
	{
	case 252: return 14;
	case 504: return mirisdr_format_real(format) ? 14 : 8;     /* two bytes a pair, or a real sample */
	case 336: case 672: return 12;
	case 384: case 768: return 10;
	default: return 0;
	}
}

static void mirisdr_stream_result_of (uint32_t rate, int format, int swap, int decim, uint32_t cap,
                                      int bb_path, uint32_t bb_rate, uint32_t bb_if, uint32_t bb_if2, int bb_ds,
                                      int gap_fill, int tuners, mirisdr_stream_result_t *res)
{
	const int low = (bb_path == MIRISDR_BASEBAND_COMPLEX) || (bb_path == MIRISDR_BASEBAND_REAL) ||
	                (bb_path == MIRISDR_BASEBAND_DUAL);

	memset(res, 0, sizeof *res);
	res->rate = bb_path ? bb_rate : rate;
	res->adc_rate = rate;
	res->baseband = bb_path;
	res->decimation = (bb_path && bb_rate) ? (double) rate / bb_rate : 1;
	res->group_delay = mirisdr_bb_delay(bb_path, rate, bb_rate);
	res->gap_fill = gap_fill || (bb_path != MIRISDR_BASEBAND_OFF);
	res->nco = low && (bb_ds || ((uint64_t) rate != 4 * (uint64_t) bb_if) ||
	                   (bb_if2 && ((uint64_t) rate != 4 * (uint64_t) bb_if2)));
	res->resampled = low && bb_rate && ((uint64_t) mirisdr_bb_pow2(rate, bb_rate) * bb_rate != rate);
	res->format = mirisdr_format_name(format);
	res->adc = !mirisdr_format_real(format) ? MIRISDR_IQ_BOTH : swap ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I;
	res->decimation_bypassed = decim;
	res->usb_bytes = (uint32_t) ((uint64_t) rate * 1024 / mirisdr_format_spp(format));
	res->usb_capacity = cap;
	res->adc_clock = decim ? rate : 2 * rate;
	res->bits = mirisdr_format_bits(format);
	res->tuners = tuners;
}

/* the registers only: the stream is stopped around this */
static void mirisdr_stream_regs (mirisdr_dev_t *p, const mirisdr_stream_plan_t *pl)
{
	uint32_t reg3 = 0, reg4 = 0, reg7;
	uint64_t i, vco, n, fract;
	unsigned try;

	p->rate = pl->rate;
	p->format = pl->format;
	p->format_auto = pl->format_auto;
	p->decimation_bypass = pl->decimation_bypass;
	p->decim_on = pl->decim;
	p->swap_iq = pl->swap;
	p->transfer = pl->transfer;
	p->alt_setting = pl->alt;
	p->gap_fill = pl->cfg.gap_fill || (pl->bb_path != MIRISDR_BASEBAND_OFF);
	p->stream = pl->cfg;
	p->bb_path = pl->bb_path;
	p->bb_stages = pl->bb_stages;
	p->bb_rate = pl->bb_rate;
	p->bb_if = pl->bb_if;
	p->bb_if2 = pl->bb_if2;
	p->bb_ds = pl->bb_ds;
	mirisdr_bb_setup(p);
	p->xfer_out_len = p->bb ? 0 : p->user_out_len;
	p->xfer_out_pos = 0;

	reg7 = mirisdr_format_reg7(pl->format) | (pl->swap ? (1 << 9) : 0) | (pl->decim ? (1 << 3) : 0) |
	       ((mirisdr_alt_burst(pl->alt) - 1) << 6);
	mirisdr_write_reg(p, 0x07, reg7);
	p->addr_step = mirisdr_format_spp(pl->format);
	p->addr = p->addr_step + 2;

	i = pl->cand[0];
	vco = (uint64_t) pl->pll_rate * i * 12;

	/* N is at least 4 here */
	n = vco / 48000000UL;
	fract = 0x200000UL * (vco % 48000000UL) / 48000000UL;
#if MIRISDR_DEBUG >= 1
	fprintf(stderr, "rate: %u, pll: %u, vco: %lu (%lu), n: %lu, fraction: %lu\n", pl->rate, pl->pll_rate,
	        (long unsigned) vco, (long unsigned) (i / 2) - 1, (long unsigned) n, (long unsigned) fract);
#endif

	reg3 |= 3 << 0;                                 /* ?? */
	reg3 |= (0x07 & (i / 2 - 1)) << 2;              /* the divider */
	reg3 |= (0x01 & (fract >> 20)) << 7;            /* +0.5 */
	reg3 |= (0x0f & n) << 8;                        /* the main range */
	reg3 |= mirisdr_format_agc(pl->format) << 12;
	reg3 |= 1 << 16;                                /* ?? */

	/* The real formats digitise one converter, so gate the other off: bit 5
	   powers down I, bit 6 Q.  reg7 bit 9 chooses which one is read. */
	if (mirisdr_format_real(pl->format)) reg3 |= pl->swap ? (1 << 5) : (1 << 6);

	reg4 |= (0xfffff & fract) << 0;

	mirisdr_write_reg(p, 0x04, reg4);
	mirisdr_write_reg(p, 0x03, reg3);

	/* The part reports its VCO band selection in read index 0, low nibble.
	   Extremes: 0xF when it wants more capacitance than the bank has, 0 when it has
	   none left to remove. We try to avoid the edges to increase tolerance to
	   variation */
	for (try = 1; (pl->ncand > 1) && (try <= pl->ncand); try++)
	{
		uint8_t rd[4];
		uint64_t alt, v;

		/* Let the capacitance search settle before asking what it chose. */
#if defined (_WIN32) && !defined(__MINGW32__)
		Sleep(1);               /* 1ms */
#else
		usleep(200);
#endif

		if (mirisdr_read_reg(p, 0, rd, sizeof(rd)) != (int) sizeof(rd)) break;

		rd[0] &= 0x0f;

		if ((rd[0] > 1) && (rd[0] < 0x0e)) break;

		/* Once the list is exhausted nothing was better than where it started,
		   so put the first choice back rather than leave the last one tried. */
		alt = (try < pl->ncand) ? pl->cand[try] : pl->cand[0];
		v = (uint64_t) pl->pll_rate * alt * 12;
		n = v / 48000000UL;
		fract = 0x200000UL * (v % 48000000UL) / 48000000UL;

		reg3 = (reg3 & ~0xf9cUL)
		     | ((0x07 & (uint32_t) (alt / 2 - 1)) << 2)
		     | ((0x01 & (uint32_t) (fract >> 20)) << 7)
		     | ((0x0f & (uint32_t) n) << 8);
		reg4 = (uint32_t) (fract & 0xfffff);

#if MIRISDR_DEBUG >= 1
		fprintf(stderr, "vco band %X at %lu, trying %lu\n", rd[0],
		        (long unsigned int) vco, (long unsigned int) v);
#endif
		vco = v;

		mirisdr_write_reg(p, 0x04, reg4);
		mirisdr_write_reg(p, 0x03, reg3);

		if (try == pl->ncand) break;
	}
}

/* stopped and started around it if it runs; returns whether it ran */
static int mirisdr_stream_pause (mirisdr_dev_t *p)
{
	int s;

	/* a stream still starting sets up its transfers from what this changes */
	while (mirisdr_async_get(p) == MIRISDR_ASYNC_STARTING)
#if defined (_WIN32) && !defined(__MINGW32__)
		Sleep(1);
#else
		usleep(1000);
#endif

	/* the synchronous reads: stopped, the next one starts it again */
	if (p->sync_ready) return (mirisdr_streaming_stop(p) < 0) ? -1 : 0;

	/* a stream ending: its transfers still come in, and feed the filters this replaces.
	   Back here first, under the lock the run drains and frees them under. Once in, a
	   canceled stream submits none */
	s = mirisdr_async_get(p);
	if ((s == MIRISDR_ASYNC_CANCELING) || (s == MIRISDR_ASYNC_FAILED))
	{
		int r = 0;

		/* as a pause does: the firmware stopped first, so the registers do not change
		   under a running stream (the run's own stop may not have gone out yet) */
		mirisdr_xfer_lock(p);
		if (p->xfer && !p->xfer_abandoned)
		{
			mirisdr_streaming_stop(p);
			r = mirisdr_async_drain(p, 5);
		}
		mirisdr_xfer_unlock(p);

		return ((r < 0) || (mirisdr_adc_stop(p) < 0)) ? -1 : 0;
	}

	if (s != MIRISDR_ASYNC_RUNNING) return 0;

	return ((mirisdr_stop_async(p) < 0) || (mirisdr_adc_stop(p) < 0)) ? -1 : 1;
}

static int mirisdr_stream_send (mirisdr_dev_t *p, const mirisdr_stream_plan_t *pl)
{
	int streaming;

	if ((streaming = mirisdr_stream_pause(p)) < 0) return -1;

	mirisdr_stream_regs(p, pl);

	return (streaming && (mirisdr_stream_resume(p) < 0)) ? -1 : 0;
}

/* What a stream c takes from the tune in force: under direct sampling the I converter,
   the frequency as its IF and the low pass c leads to. Under MIRISDR_DS_AUTO whether it
   is in force follows c's band, so a new rate can switch it (the tune follows, as moved) */
static uint32_t mirisdr_stream_tune_of (mirisdr_dev_t *p, const mirisdr_stream_config_t *c, uint32_t *ds_edge,
                                        int *iq)
{
	int ds = mirisdr_ds_takes(p->tune.frontend.direct_sampling, c, p->tune.frequency);

	*ds_edge = ds ? mirisdr_ds_edge(c, p->tune.frequency) : 0;
	*iq = (ds == p->ds) ? p->tune_iq : ds ? MIRISDR_IQ_ONLY_I : p->tune.iq;

	return ds ? p->tune.frequency : mirisdr_if_hz[p->if_freq];
}

static int mirisdr_stream_apply (mirisdr_dev_t *p, const mirisdr_stream_config_t *c, int flags,
                                 mirisdr_stream_result_t *res)
{
	mirisdr_stream_plan_t pl;
	uint32_t if_hz, ds_edge;
	int moved, old, t, sel, iq;

	if (!p || !c) return -1;

	/* planned on the stream's first tuner, as the single setters find it after */
	old = mirisdr_stream_tuners(p);
	sel = (c->tuners && !(c->tuners & 1) && (mirisdr_tuners_of(p) > 1)) ? 1 : 0;
	mirisdr_tuner_select(p, sel);

	if_hz = mirisdr_stream_tune_of(p, c, &ds_edge, &iq);
	if (mirisdr_stream_plan(p, c, flags, iq, if_hz, ds_edge, &pl) < 0)
	{
		mirisdr_tuner_select(p, mirisdr_tuner_primary(p));
		return -1;
	}

	/* baseband on or off with a low IF moves the LO. Under direct sampling the stream
	   picks the low pass, which goes out with the tune, and other tuners switch the converters */
	moved = ((pl.cfg.baseband != p->stream.baseband) && (p->if_freq != MIRISDR_IF_ZERO)) ||
	        (ds_edge != p->ds_edge) || (pl.tuners != old);

	/* nothing the hardware or the filters hold changes: no restart. Other tuners do, as the
	   buffers under way are labelled with the tuner they are from */
	if (!(flags & MIRISDR_STREAM_FORCE) && (pl.tuners == old) && (pl.rate == p->rate) && (pl.format == (int) p->format) &&
	    (pl.decim == p->decim_on) && (pl.swap == p->swap_iq) && (pl.alt == p->alt_setting) &&
	    (pl.bb_path == p->bb_path) && (pl.bb_stages == p->bb_stages) && (pl.bb_rate == p->bb_rate) &&
	    (pl.bb_if == p->bb_if) && (pl.bb_if2 == p->bb_if2) && (pl.bb_ds == p->bb_ds))
	{
		p->format_auto = pl.format_auto;
		p->decimation_bypass = pl.decimation_bypass;
		p->transfer = pl.transfer;
		p->gap_fill = pl.cfg.gap_fill || (pl.bb_path != MIRISDR_BASEBAND_OFF);
		p->stream = pl.cfg;
	}
	else if (mirisdr_stream_send(p, &pl) < 0)
	{
		mirisdr_tuner_select(p, mirisdr_tuner_primary(p));
		return -1;
	}

	/* at zero IF a filter the tune left to the library follows the rate */
	if (!p->tune.bandwidth && (p->if_freq == MIRISDR_IF_ZERO) &&
	    (mirisdr_bw_auto(MIRISDR_IF_ZERO, p->rate) != (int) p->bandwidth)) moved = 1;

	/* the tunes again, each tuner streamed: the converter switch and the LO follow */
	if (moved)
	{
		int r = 0;

		for (t = 0; t < mirisdr_tuners_of(p); t++)
		{
			if (!(pl.tuners & (1 << t))) continue;
			mirisdr_tuner_select(p, t);
			if (mirisdr_set_soft(p) < 0) r = -1;
		}
		mirisdr_tuner_select(p, mirisdr_tuner_primary(p));
		if (r < 0) return -1;
	}

	if (res) mirisdr_stream_result_of(p->rate, p->format, p->swap_iq, p->decim_on, pl.cap, p->bb_path, p->bb_rate,
	                                  p->bb_if, p->bb_if2, p->bb_ds, p->stream.gap_fill, pl.tuners, res);

	return 0;
}

static int mirisdr_set_hard(mirisdr_dev_t *p)
{
	return mirisdr_stream_apply(p, &p->stream, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

void mirisdr_stream_config_default (mirisdr_stream_config_t *cfg)
{
	if (!cfg) return;

	memset(cfg, 0, sizeof *cfg);
	cfg->rate = DEFAULT_RATE;
}

int mirisdr_set_stream (mirisdr_dev_t *p, const mirisdr_stream_config_t *cfg, mirisdr_stream_result_t *res)
{
	return mirisdr_stream_apply(p, cfg, 0, res);
}

int mirisdr_stream_check (mirisdr_dev_t *p, const mirisdr_stream_config_t *cfg, mirisdr_stream_result_t *res)
{
	mirisdr_stream_plan_t pl;
	int r;

	if (!p || !cfg) return -1;
	p->checking = 1;
	mirisdr_tuner_select(p, (cfg->tuners && !(cfg->tuners & 1) && (mirisdr_tuners_of(p) > 1)) ? 1 : 0);
	{
		uint32_t ds_edge;
		int iq;
		uint32_t if_hz = mirisdr_stream_tune_of(p, cfg, &ds_edge, &iq);

		r = mirisdr_stream_plan(p, cfg, 0, iq, if_hz, ds_edge, &pl);
	}
	mirisdr_tuner_select(p, mirisdr_tuner_primary(p));
	p->checking = 0;
	if (r < 0) return -1;

	if (res) mirisdr_stream_result_of(pl.rate, pl.format, pl.swap, pl.decim, pl.cap, pl.bb_path, pl.bb_rate,
	                                  pl.bb_if, pl.bb_if2, pl.bb_ds, pl.cfg.gap_fill, pl.tuners, res);

	return 0;
}

int mirisdr_get_stream (mirisdr_dev_t *p, mirisdr_stream_config_t *cfg, mirisdr_stream_result_t *res)
{
	if (!p) return -1;

	if (cfg) *cfg = p->stream;
	if (res) mirisdr_stream_result_of(p->rate, p->format, p->swap_iq, p->decim_on,
	                                  mirisdr_stream_cap(p->transfer, p->alt_setting, p->stream.usb_capacity),
	                                  p->bb_path, p->bb_rate, p->bb_if, p->bb_if2, p->bb_ds, p->stream.gap_fill,
	                                  mirisdr_stream_tuners(p), res);

	return 0;
}

/* ------------------------------------------------------------------ */
/* keep the old API                                                   */
/* ------------------------------------------------------------------ */

int mirisdr_set_sample_rate(mirisdr_dev_t *p, uint32_t rate)
{
	mirisdr_stream_config_t c;
	int r;

	if (!p) return -1;

	c = p->stream;
	c.rate = rate;
	r = mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);

	/* the range depends on the decimation bypass, report what was reached: the output's,
	   which under baseband is not the converters' */
	if (p->stream.rate != rate) fprintf(stderr, "can't set rate %u, using %u\n", rate, p->stream.rate);

	return r;
}

int mirisdr_set_sample_format(mirisdr_dev_t *p, const char *v)
{
	mirisdr_stream_config_t c;

	if (!p || !v) return -1;

	c = p->stream;
	c.format = v;
	c.follow_tune = 0;

	return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

int mirisdr_set_decimation_bypass(mirisdr_dev_t *p, const char *v)
{
	mirisdr_stream_config_t c;

	if (!p || !v) return -1;

	c = p->stream;
	c.decimation_bypass = v;

	return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

int mirisdr_set_swap_iq(mirisdr_dev_t *p, int swap)
{
	mirisdr_stream_config_t c;

	if (!p) return -1;

	c = p->stream;
	c.swap_iq = swap;

	return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

/* the callback's rate, as set: under baseband the output's */
uint32_t mirisdr_get_sample_rate(mirisdr_dev_t *p)
{
	if (!p) return 0;

	return p->stream.rate;
}

const char *mirisdr_get_decimation_bypass(mirisdr_dev_t *p)
{
	if (!p) return "";

	switch (p->decimation_bypass)
	{
	case MIRISDR_DECIMATION_BYPASS_ON:
		return "ON";
	case MIRISDR_DECIMATION_BYPASS_OFF:
		return "OFF";
	default:
		return "AUTO";
	}
}

int mirisdr_get_swap_iq(mirisdr_dev_t *p)
{
	if (!p) return -1;

	return p->swap_iq;
}

const char *mirisdr_get_sample_format(mirisdr_dev_t *p)
{
	if (!p) return "";

	if (p->format_auto == MIRISDR_FORMAT_AUTO_ON) {
		return "AUTO";
	}

	if (p->format_auto == MIRISDR_FORMAT_AUTO_REAL) {
		return "AUTO_REAL";
	}

	return mirisdr_get_sample_format_selected(p);
}

const char *mirisdr_get_sample_format_selected(mirisdr_dev_t *p)
{
	if (!p) return "";

	switch (p->format)
	{
	case MIRISDR_FORMAT_252_S16:
		return "252_S16";
	case MIRISDR_FORMAT_336_S16:
		return "336_S16";
	case MIRISDR_FORMAT_384_S16:
		return "384_S16";
	case MIRISDR_FORMAT_504_S16:
		return "504_S16";
	case MIRISDR_FORMAT_504_S8:
		return "504_S8";
	case MIRISDR_FORMAT_504_REAL_S16:
		return "504_REAL_S16";
	case MIRISDR_FORMAT_672_REAL_S16:
		return "672_REAL_S16";
	case MIRISDR_FORMAT_768_REAL_S16:
		return "768_REAL_S16";
	}

	return "";
}
