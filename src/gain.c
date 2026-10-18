/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
 * Copyright (C) 2025 by Peter Hackenberg <170885528+Peter3579@users.noreply.github.com>
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

#include "gain.h"

static int mirisdr_set_gain_words(mirisdr_dev_t *p);
static int mirisdr_gain_tune(mirisdr_dev_t *p);

int mirisdr_set_gain(mirisdr_dev_t *p)
{
    int r;

    if (!p) return -1;

    mirisdr_batch_begin(p);
    r = mirisdr_set_gain_words(p);
    if (mirisdr_batch_end(p) < 0) r = -1;

    return r;
}

/* XTALSEL naming the board's crystal */
static uint32_t mirisdr_xtalsel (mirisdr_dev_t *p)
{
    switch (p->xtal)
    {
    case MIRISDR_XTAL_19_2M:  return 0;
    case MIRISDR_XTAL_22M:    return 1;
    case MIRISDR_XTAL_26M:    return 3;
    case MIRISDR_XTAL_38_4M:  return 4;
    default:                  return 2;   /* 24 and 24.576 MHz */
    }
}

/* DC calibration divider N per XTALSEL (datasheet table 21) */
static const uint8_t mirisdr_xtalsel_n[8] = { 48, 55, 72, 65, 48, 48, 48, 48 };

/* DC tracking counts in N / fref: when a band names another crystal, scale the
   track and refresh times so they stay what they are under the real one */
static uint32_t mirisdr_dc_word (mirisdr_dev_t *p)
{
    uint32_t want = mirisdr_xtalsel_n[mirisdr_xtalsel(p)], n = p->dc_n ? p->dc_n : want;
    uint32_t track = ((p->dc_track & 0x3f) * want + n / 2) / n;
    uint32_t period = ((p->dc_period & 0xfff) * want + n / 2) / n;

    if (track > 63) track = 63;
    if (period > 4095) period = 4095;

    return track | period << 6;
}

static int mirisdr_set_gain_words(mirisdr_dev_t *p)
{
    uint32_t reg1 = 0, reg6 = 0;
#if MIRISDR_DEBUG >= 1
    fprintf(stderr,
#if MIRISDR_DEBUG >= 3
        "mirisdr_set_gain:\n"
#endif
        "mirisdr tuner gain: %d dB (band: %d, "
        "attenuations: baseband: %d, lna: %d, mixbuffer: %d,"
        " mixer: %d)\nmirisdr ",
        mirisdr_get_tuner_gain(p), p->band, p->gain_reduction_baseband,
        p->gain_reduction_lna, p->gain_reduction_mixbuffer,
        p->gain_reduction_mixer);
    if (p->dc_mode == MIRISDR_DC_STATIC)
        fprintf(stderr,"dc-cal: off");
    else if (p->dc_mode == MIRISDR_DC_CONTINUOUS)
        fprintf(stderr,"dc-cal: continuous");
    else if (p->dc_mode > MIRISDR_DC_CONTINUOUS)
        fprintf(stderr,"dc-cal: INVALID mode");
    else if (p->dc_mode == MIRISDR_DC_ONE_SHOT)
        fprintf(stderr,"dc-cal: %d µs once", 12 * p->dc_track);
        // Calculation is only valid for 24 MHz XTAL.
    else
        fprintf(stderr,"dc-cal: %d µs @ %.1f Hz",
             3 * p->dc_track * p->dc_mode,
             1e6 / (3. * p->dc_period * p->dc_mode));
    fprintf(stderr, " (mode: %d, speedup: %d, track: %d, period: %d)\n",
        p->dc_mode, p->dc_speedup, p->dc_track, p->dc_period);
#endif
// Reset to 0xf380 to enable gain control added Dec 5 2014 SM5BSZ
//    mirisdr_write_reg(p, 0x08, 0xf380);

    /* Receiver Gain Control */
    /* data bits, without the address */
    /* 0-5 => baseband, 0 - 59, 60-63 is the same as 59 */
    /* 6-7 => mixer gain reduction, AM mode only */
    /* 8 => mixer gain reduction -19dB */
    /* 9 => lna gain reduction -24dB */
    /* 10-12 => DC calibration */
    /* 13 => fast DC calibration */
    reg1 |= p->gain_reduction_baseband;

    // Mixbuffer is on AM1 and AM2 inputs only
    if (p->band == MIRISDR_BAND_AM1)
    {
        reg1 |= (p->gain_reduction_mixbuffer & 0x03) << 6;
    }
    else if (p->band == MIRISDR_BAND_AM2)
    {
        reg1 |= (p->gain_reduction_mixbuffer == 0 ? 0x0 : 0x03) << 6;
    }
    else
    {
        reg1 |= 0x0 << 6;
    }

    reg1 |= p->gain_reduction_mixer << 8;

    // LNA is not on AM1 nor AM2 inputs
    if ((p->band == MIRISDR_BAND_AM1) || (p->band == MIRISDR_BAND_AM2))
    {
        reg1 |= 0x0 << 9;
    }
    else
    {
        reg1 |= p->gain_reduction_lna << 9;
    }

    reg1 |= (p->dc_mode & 0x7) << 10;
    reg1 |= ((p->dc_speedup)? MIRISDR_DC_OFFSET_CALIBRATION_SPEEDUP_ON :
                              MIRISDR_DC_OFFSET_CALIBRATION_SPEEDUP_OFF) << 13;
    if (p->external_tuner) return 0;

    if (mirisdr_tuner_write(p, 1, reg1, 0) < 0) return -1;

    /* DC Offset Calibration setup */
    reg6 = mirisdr_dc_word(p);
    if (mirisdr_tuner_write(p, 6, reg6, 0) < 0) return -1;
//// set to 0xf300 to select AM input added Dec 5 2014 SM5BSZ
//    if (p->freq < 50000000)
//      {
//      mirisdr_write_reg(p, 0x08, 0xf300);
//      }
//    else
//      {
//      if (p->freq >= 108000000)
//        {
//// Nothing between 00 and 0xff helps to switch in signals above 108 MHz.
////        mirisdr_write_reg(p, 0x08, 0xf3ff);
//        }
//      }

    return 0;
}

static int mirisdr_front_gain_of(mirisdr_band_t band)
{
    switch (band)
    {
    case MIRISDR_BAND_AM1:  return 18;
    case MIRISDR_BAND_45:   return 7;
    case MIRISDR_BAND_L:    return 4;
    default:                return 24;
    }
}

static int mirisdr_front_gain(mirisdr_dev_t *p)
{
    return mirisdr_front_gain_of(p->band);
}

static int mirisdr_max_gain_of(mirisdr_band_t band)
{
    return 59 + 19 + mirisdr_front_gain_of(band);
}

static int mirisdr_max_gain(mirisdr_dev_t *p)
{
    return mirisdr_max_gain_of(p->band) + p->amp_db;
}

/* Gain reductions as the tuner takes them, with the board's amplifier before it */
typedef struct mirisdr_gr
{
    int lna, mixbuffer, mixer, baseband;
    int amp;            /* the board's amplifier bypassed */
    int amp_db;         /* its gain on the path in force, 0 none: given, not chosen */
} mirisdr_gr_t;

/* A total split for a band and r->amp_db: the front end first, so noise figure and IIP3
   stay low. The board's amplifier is on while the total holds it. On a path without one its
   state is left as it was, for the next path that has one */
static void mirisdr_split_of(mirisdr_band_t band, int gain, mirisdr_gr_t *r)
{
    int front = mirisdr_front_gain_of(band);

    if (gain > mirisdr_max_gain_of(band) + r->amp_db) gain = mirisdr_max_gain_of(band) + r->amp_db;
    if (gain < 0) gain = 0;

    if (r->amp_db)
    {
        r->amp = gain < r->amp_db;
        if (!r->amp) gain -= r->amp_db;
    }

    if (gain >= front + 19)
    {
        r->lna = 0;
        r->mixbuffer = 0;
        r->mixer = 0;
        r->baseband = 59 - (gain - front - 19);
    }
    else if (gain >= 19)
    {
        r->lna = 1;
        r->mixbuffer = 3;
        r->mixer = 0;
        r->baseband = 59 - (gain - 19);
    }
    else
    {
        r->lna = 1;
        r->mixbuffer = 3;
        r->mixer = 1;
        r->baseband = 59 - gain;
    }
}

static int mirisdr_mixbuffer_of(mirisdr_band_t band, const mirisdr_gr_t *r)
{
    if (band == MIRISDR_BAND_AM2) return r->mixbuffer ? 0 : 24;
    if (band == MIRISDR_BAND_AM1) return 18 - 6 * r->mixbuffer;

    return 0;   /* no such stage */
}

static int mirisdr_lna_of(mirisdr_band_t band, const mirisdr_gr_t *r)
{
    if (r->lna) return 0;
    if (band == MIRISDR_BAND_45) return 7;
    if (band == MIRISDR_BAND_L) return 4;   /* mean of measured values, with LNA cal */

    return 24;
}

/* What the stages add up to in a band, and each of them */
static void mirisdr_stages_of(mirisdr_band_t band, const mirisdr_gr_t *r, mirisdr_gain_config_t *g)
{
    int am = (band == MIRISDR_BAND_AM1) || (band == MIRISDR_BAND_AM2);

    memset(g, 0, sizeof *g);
    g->mode = MIRISDR_GAIN_STAGES;
    g->lna = !r->lna;
    g->mixer = !r->mixer;
    g->mixbuffer = mirisdr_mixbuffer_of(band, r);
    g->baseband = 59 - r->baseband;
    g->amp_bypass = r->amp_db ? r->amp : 0;
    g->total = g->baseband + (g->mixer ? 19 : 0) + (am ? g->mixbuffer : mirisdr_lna_of(band, r)) +
               (g->amp_bypass ? 0 : r->amp_db);
}

static void mirisdr_gr_get(const mirisdr_dev_t *p, mirisdr_gr_t *r)
{
    r->lna = p->gain_reduction_lna;
    r->mixbuffer = p->gain_reduction_mixbuffer;
    r->mixer = p->gain_reduction_mixer;
    r->baseband = p->gain_reduction_baseband;
    r->amp = p->gain_reduction_amp;
    r->amp_db = p->amp_db;
}

static void mirisdr_gr_set(mirisdr_dev_t *p, const mirisdr_gr_t *r)
{
    p->gain_reduction_lna = r->lna;
    p->gain_reduction_mixbuffer = r->mixbuffer;
    p->gain_reduction_mixer = r->mixer;
    p->gain_reduction_baseband = r->baseband;
    p->gain_reduction_amp = r->amp;
    p->amp_db = r->amp_db;
}

/* Stages asked for as reductions, for a band and r->amp_db. Strict: dB values the band
   cannot do are refused, and a bypass where there is no amplifier. Otherwise each is taken
   to the nearest the band has (on or off, baseband 0-59, the mixbuffer's steps). On a path
   without the amplifier its state is left as it was */
static int mirisdr_gr_of_stages(mirisdr_band_t band, const mirisdr_gain_config_t *g, mirisdr_gr_t *r, int strict)
{
    int mb = g->mixbuffer, bb = g->baseband;

    if (strict && ((g->lna & ~1) || (g->mixer & ~1) || (g->amp_bypass & ~1) || (bb < 0) || (bb > 59) ||
                   (g->amp_bypass && !r->amp_db)))
        return -1;

    if (bb < 0) bb = 0;
    if (bb > 59) bb = 59;
    r->lna = !g->lna;
    r->mixer = !g->mixer;
    r->baseband = 59 - bb;
    if (r->amp_db) r->amp = !!g->amp_bypass;

    if (band == MIRISDR_BAND_AM2)
    {
        if (strict && (mb != 0) && (mb != 24)) return -1;
        r->mixbuffer = (mb >= 12) ? 0 : 3;
    }
    else if (band == MIRISDR_BAND_AM1)
    {
        if (strict && ((mb < 0) || (mb > 18) || (mb % 6))) return -1;
        if (mb < 0) mb = 0;
        if (mb > 18) mb = 18;
        r->mixbuffer = 3 - (mb + 3) / 6;
    }
    else r->mixbuffer = 3;      /* no such stage */

    return 0;
}

/* A kept mixbuffer code from one AM input to the other: its dB to the nearest step the
   new one has (AM2 0 or 24, AM1 0 to 18 by 6). Other bands keep the code */
static int mirisdr_mixbuffer_keep(mirisdr_band_t from, mirisdr_band_t to, int code)
{
    mirisdr_gr_t r;
    int db;

    if ((from == to) || ((from != MIRISDR_BAND_AM1) && (from != MIRISDR_BAND_AM2)) ||
        ((to != MIRISDR_BAND_AM1) && (to != MIRISDR_BAND_AM2))) return code;

    memset(&r, 0, sizeof r);
    r.mixbuffer = code;
    db = mirisdr_mixbuffer_of(from, &r);

    if (to == MIRISDR_BAND_AM2) return (db >= 12) ? 0 : 3;

    return 3 - ((db >= 18) ? 3 : (db + 3) / 6);
}

static void mirisdr_gain_split(mirisdr_dev_t *p)
{
    mirisdr_gr_t r;

    mirisdr_gr_get(p, &r);
    mirisdr_split_of(p->band, p->gain, &r);
    mirisdr_gr_set(p, &r);
}

static void mirisdr_gain_retune (mirisdr_dev_t *p)
{
    if (p->gain >= 0 && !p->gain_stages_set) mirisdr_gain_split(p);
}

/*
 * Provide list of available gain settings.
 * Used e.g. from gnuradio-osmosdr/lib/miri
 * The (first) call with *gains==NULL returns number of available gains.
 * The (second) call with *gains!=NULL fills the array and returns the count.
 *
 * The max. available gain depends on the selected band: 102, 85 in band IV/V, 82 in
 * L band and 96 on AM port 1, plus the board's amplifier: from 30 MHz on the RSP1B and
 * RSPdx-R2, from 55 MHz on the RSPduo.
 */
int mirisdr_get_tuner_gains(mirisdr_dev_t *p, int *gains)
{
    int i;

    if (!p) return -1;
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_tuner_gains: %p (band: %d)\n", gains, p->band);
#endif
    i = mirisdr_max_gain(p) + 1;
    if (gains)
    {
        for (i = 0; i <= mirisdr_max_gain(p); i++)
        {
            gains[i] = i;
        }
    }

    return i;
}

int mirisdr_set_tuner_gain(mirisdr_dev_t *p, int gain)
{
    if (!p)
    {
        fprintf(stderr, "mirisdr_set_tuner_gain: error: nil device pointer!\n");
        return -1;
    }
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_tuner_gain: %d dB (band: %d)\n", gain, p->band);
#endif
    /* there is no automatic gain, and the gain in force stays */
    if (gain < 0)
    {
        goto gain_auto;
    }

    p->gain = gain;
    p->gain_stages_set = 0;
    if (p->ds) return mirisdr_gain_tune(p);

    /* the board's amplifier switched on or off is an expander pin: the tune again */
    if (p->amp_db)
    {
        mirisdr_gr_t r;

        mirisdr_gr_get(p, &r);
        mirisdr_split_of(p->band, gain, &r);
        if (r.amp != p->gain_reduction_amp) return mirisdr_gain_tune(p);
    }
    mirisdr_gain_split(p);

    return mirisdr_set_gain(p);

    gain_auto: return mirisdr_set_tuner_gain_mode(p, 0);
}

/* gain 0 corresponds to the maximal attenuated RF input */
int mirisdr_get_tuner_gain(mirisdr_dev_t *p)
{
    mirisdr_gain_config_t g;
    mirisdr_gr_t r;

    if (!p) return -1;

    if (p->gain < 0)
        goto gain_auto;

    mirisdr_gr_get(p, &r);
    mirisdr_stages_of(p->band, &r, &g);

#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_tuner_gain: %d dB (band: %d)\n", g.total, p->band);
#endif
    return g.total;

    gain_auto:
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_tuner_gain: -1 (no automatic gain)\n");
#endif
        return -1;
}

/*
 * Used e.g. from gnuradio-osmosdr/lib/miri
 * mode==false is "automatic", mode==true is "manual".
 * Returns 0 on success, -1 on error.
 * Returns error (-1) if automatic mode is requested.
 */
int mirisdr_set_tuner_gain_mode(mirisdr_dev_t *p, int mode)
{
    if (!p) return -1;
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_tuner_gain_mode: %d (%s)\n", mode, (mode)?"manual":"automatic -> rejected");
#endif
//    if (!mode) {
//        p->gain = -1;
//#if MIRISDR_DEBUG >= 1
//        fprintf( stderr, "gain mode: auto\n");
//#endif
//        mirisdr_write_reg(p, 0x09, 0x014281);
//        mirisdr_write_reg(p, 0x09, 0x3FFFF6);
//    } else if (p->gain < 0) {
//#if MIRISDR_DEBUG >= 1
//        fprintf( stderr, "gain mode: manual\n");
//#endif
//        p->gain = DEFAULT_GAIN;
//    }

    return (mode) ? 0 : -1;
}

int mirisdr_get_tuner_gain_mode(mirisdr_dev_t *p)
{
    if (!p) return -1;
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_tuner_gain_mode: %d\n", 1);
#endif
//    return (p->gain < 0) ? 0 : 1;
    return 1;  // manual mode
}

/*
 * Gain reduction is an index that depends on the AM mode (only applies to AM inputs)
 *          AM1     AM2
 * 0x00    0 dB    0 dB
 * 0x01    6 dB   24 dB
 * 0x10   12 dB   24 dB
 * 0x11   18 dB   24 dB
 */
int mirisdr_set_mixer_gain(mirisdr_dev_t *p, int gain)
{
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_mixer_gain: %d (-> %d dB)\n", gain, gain ? 19 : 0);
    fprintf(stderr, "_set_mixer_gain -> ");
#endif
    if (!p)
    {
        fprintf(stderr, "mirisdr_set_mixer_gain: error: nil device pointer!\n");
        return -1;
    }
    p->gain_reduction_mixer = gain ? 0 : 1;

    p->gain_stages_set = 1;

    return mirisdr_set_gain(p);
}

int mirisdr_set_mixbuffer_gain(mirisdr_dev_t *p, int gain)
{
    if (!p)
    {
        fprintf(stderr, "mirisdr_set_mixbuffer_gain: error: nil device pointer!\n");
        return -1;
    }
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_mixbuffer_gain: %d dB (band: %d)\n", gain, p->band);
#endif
    if (gain < 0) {
        fprintf(stderr, "ERROR: mirisdr_set_mixbuffer_gain: negative number provided: %d\n", gain);
        return -1;
    }

    if (gain > 18) {
        gain = 18; // clamp
    }

    p->gain_reduction_mixbuffer = (3 - gain / 6) & 0x03;

    p->gain_stages_set = 1;

    return mirisdr_set_gain(p);
}

int mirisdr_set_lna_gain(mirisdr_dev_t *p, int gain)
{
    if (!p)
    {
        fprintf(stderr, "mirisdr_set_lna_gain: error: nil device pointer!\n");
        return -1;
    }
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_lna_gain: %d (band: %d)\n", gain, p->band);
#endif
    p->gain_reduction_lna = gain ? 0 : 1;

    p->gain_stages_set = 1;
    if (p->ds) return mirisdr_gain_tune(p);

    return mirisdr_set_gain(p);
}

int mirisdr_set_baseband_gain(mirisdr_dev_t *p, int gain)
{
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_baseband_gain: %d dB\n", gain);
#endif
    if (!p)
    {
        fprintf(stderr, "mirisdr_set_baseband_gain: error: nil device pointer!\n");
        return -1;
    }
    if (gain < 0) gain = 0;
    if (gain > 59) gain = 59;
    p->gain_reduction_baseband = 59 - gain;

    p->gain_stages_set = 1;

    return mirisdr_set_gain(p);
}

int mirisdr_get_mixer_gain(mirisdr_dev_t *p)
{
    int gain;

    if (!p) return -1;

    gain = p->gain_reduction_mixer ? 0 : 19;
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_mixer_gain: %d dB\n", gain);
#endif
    return gain;
}

int mirisdr_get_mixbuffer_gain(mirisdr_dev_t *p)
{
    int gain;

    if (!p) return -1;

    gain = 18 - 6*p->gain_reduction_mixbuffer;
    if (p->band == MIRISDR_BAND_AM2)
    {
        gain = p->gain_reduction_mixbuffer ? 0 : 24;
    }
    // report mixbuffer gain even if it's not really used on other bands to not confuse clients
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_mixbuffer_gain: %d dB (band: %d)\n", gain, p->band);
#endif
    return gain;
}

int mirisdr_get_lna_gain(mirisdr_dev_t *p)
{
    int gain;

    if (!p) return -1;

    gain = 24;
    if (p->gain_reduction_lna) gain = 0;
    else if (p->band == MIRISDR_BAND_45) gain = 7;
    else if (p->band == MIRISDR_BAND_L) gain = 4; /* mean of measured values, with LNA cal */
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_lna_gain: %d dB (band: %d)\n", gain, p->band);
#endif
    return gain;
}

int mirisdr_get_baseband_gain(mirisdr_dev_t *p)
{
    int gain;

    if (!p) return -1;

    gain = 59 - p->gain_reduction_baseband;
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_baseband_gain: %d dB\n", gain);
#endif
    return gain;
}

int mirisdr_set_dc_raw (mirisdr_dev_t *p, uint32_t raw)
{
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_set_dc_raw: %u\n", raw);
#endif
    if (!p)
    {
        fprintf(stderr, "mirisdr_set_dc_raw: error: nil device pointer!\n");
        return -1;
    }
    p->dc_mode = raw & 0x7;
    raw >>= 3;
    p->dc_speedup = raw & 0x1;
    raw >>= 1;
    p->dc_track = raw & 0x3f;
    raw >>= 12;
    p->dc_period = raw & 0xfff;

    return mirisdr_set_gain(p);
}

uint32_t mirisdr_get_dc_raw (mirisdr_dev_t *p)
{
    uint32_t rv;
    if (!p)
    {
        fprintf(stderr, "mirisdr_get_dc_raw: error: nil device pointer!\n");
        rv = 0;
    }
    else
    {
        rv  = p->dc_mode;
        rv |= p->dc_speedup << 3;
        rv |= p->dc_track << 4;
        rv |= p->dc_period<< 16;
    }
#if MIRISDR_DEBUG >= 3
    fprintf(stderr, "mirisdr_get_dc_raw: %u\n", rv);
#endif
    return rv;
}

