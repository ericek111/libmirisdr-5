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

#include "soft.h"
#include "sdrplay.c"

//float band_limits[] = {
//        0.,     12.,    30.,    50.,    108.,   250.,   390.,   960.,   2400,   -1.
//};
//
//uint32_t band_select[] = {
//        0xf780, 0xff80, 0xf280, 0xf380, 0xfa80, 0xf680, 0xf380, 0xfa80, 0x0000, 0x0000
//};
//GPIO0 - DAB notch
//GPIO2 - Broadcast FM notch

static hw_switch_freq_plan_t hw_switch_freq_plan_default[] = {
        {0,       MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf780, 0, 0, 0},
        {12000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xff80, 0, 0, 0},
        {30000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf280, 0, 0, 0},
        {55000,   MIRISDR_MODE_VHF, 0, 0, 32, 0xf380, 0, 0, 0},
        {108000,  MIRISDR_MODE_B3,  0, 0, 16, 0xfa80, 0, 0, 0},
        {250000,  MIRISDR_MODE_B3,  0, 0, 16, 0xf680, 0, 0, 0},
        {255000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf680, 6, 0, 0},
        {280000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf680, 7, 0, 0},
        {420000,  MIRISDR_MODE_B45, 0, 0, 4,  0xf380, 0, 0, 0},
        {960000,  MIRISDR_MODE_BL,  0, 0, 2,  0xfa80, 0, 0, 0},
        {2400000, -1, 0, 0, 0, 0x0000, 0, 0, 0},
};

static hw_switch_freq_plan_t hw_switch_freq_plan_sdrplay[] = {
        {0,       MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580, 0, 0, 0},
        {12000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580, 0, 0, 0},
        {30000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580, 0, 0, 0},
        {50000,   MIRISDR_MODE_VHF, 0, 0, 32, 0xf180, 0, 0, 0},
        {112000,  MIRISDR_MODE_B3,  0, 0, 16, 0xf580, 0, 0, 0},
        {250000,  MIRISDR_MODE_B3,  0, 0, 16, 0xf480, 0, 0, 0},
        {261000,  6              ,  0, 0, 8,  0xf480, 0, 0, 0},
        {404000,  MIRISDR_MODE_B45, 0, 0, 4,  0xf580, 0, 0, 0},
        {1000000, MIRISDR_MODE_BL,  0, 0, 2,  0xf580, 0, 0, 0},
        {2400000, -1, 0, 0, 0, 0x0000, 0, 0, 0},
};

/* RSP1B: the default plan's tuner modes, register 8 holding GPIO_1 and GPIO_3 high with
   GPIO_0 (MISO) an input, and the front end on the expander. A5 low costs 35-40 dB below
   3 MHz and gains 13-16 dB at 40 MHz, so HF keeps it released up to 30 MHz. */
#define RSP1B_VHF_B45   0xBFDF  /* B6 A5 low: the VHF and B45 inputs' path */
#define RSP1B_B3_L      0x7FDF  /* B7 A5 low: the B3 and L inputs' path */
#define RSP1B_LPF2      0x3FFF  /* B7 B6 low: the 2 MHz low pass */
#define RSP1B_2_12      0x3CFF  /* and B1 B0 low */
#define RSP1B_12_30     0x3EFF  /* and B0 low */
#define RSP1B_30_60     0x3D5F  /* B7 B6 A7 B1 low, A5 low */
#define RSP1B_250_300   0x3F5F  /* B7 B6 A7 low, A5 low */
#define RSP1B_300_380   0x3C5F  /* and B1 B0 low */
#define RSP1B_380_420   0x3E5F  /* and B0 low */
/* The 30-60 filter sits in the 250-420 bank, behind the amplifier (A5 low), which rolls off
   below 55 MHz: on both the RSP1B and the RSPdx-R2 (within 1 dB of each other, 2026-10-18)
   0.7-0.8 dB at 30 MHz, 7 at 33, 11-12 at 36, 15-16 at 40, 19-20 at 45, 22-23 at 50, 24 at
   54, the receiver's floor not rising with it. Rows by the gain in each stretch */
#define RSP1B_30_60_ROW(khz, db) \
        {khz,     MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_30_60, db}
#define RSP1B_30_60_ROWS \
        RSP1B_30_60_ROW(30000, 2), RSP1B_30_60_ROW(32000, 7), RSP1B_30_60_ROW(35000, 11), \
        RSP1B_30_60_ROW(38000, 15), RSP1B_30_60_ROW(42000, 19), RSP1B_30_60_ROW(47000, 23)
#define RSP1B_380_420_KHZ 403000 /* it beats the 300-380 filter from 403 MHz (0.6 dB at 380, 2026-10-17) */

/* The amplifier A5 low takes in (released bypasses it), against the bypass, 2026-10-17: 25
   dB at 60-145 MHz, 24.5 at 200, 23.7 at 300, 22.4-23.4 at 433-900, 21.5 at 1.2 GHz, 17 at
   1.5. It improves sensitivity: the receiver's own floor rises 7-19 dB with it. Counted
   from 55 MHz */
#define RSP1B_AMP_VHF   25
#define RSP1B_AMP_B3    25
#define RSP1B_AMP_UHF   24
#define RSP1B_AMP_B45   23
#define RSP1B_AMP_L     19

static hw_switch_freq_plan_t hw_switch_freq_plan_rsp1b[] = {
        {0,       MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_LPF2, 0},
        {2000,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_2_12, 0},
        {12000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_12_30, 0},
        RSP1B_30_60_ROWS,
        {55000,   MIRISDR_MODE_VHF, 0, 0, 32, 0xea80, 0, RSP1B_VHF_B45, RSP1B_AMP_VHF},
        {108000,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_B3_L, RSP1B_AMP_B3},
        {250000,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_B3_L, RSP1B_AMP_B3},
        {255000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 6, RSP1B_250_300, RSP1B_AMP_UHF},
        {280000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_250_300, RSP1B_AMP_UHF},
        {300000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_300_380, RSP1B_AMP_UHF},
        {RSP1B_380_420_KHZ, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_380_420,
         RSP1B_AMP_UHF},
        {420000,  MIRISDR_MODE_B45, 0, 0, 4,  0xea80, 0, RSP1B_VHF_B45, RSP1B_AMP_B45},
        {960000,  MIRISDR_MODE_BL,  0, 0, 2,  0xea80, 0, RSP1B_B3_L, RSP1B_AMP_L},
        {2400000, -1, 0, 0, 0, 0x0000, 0, 0, 0},
};

/* RSPdx-R2: the RSP1B's plan with its own HF codes. The antenna bits (A3 A2) stay
   released here, sdrplay.c pulls them for the port. A5 stays released at HF: low, it puts
   the amplifier stage (below its passband) after the 12-30 filter, which raises the floor
   3-5 dB for at most 0.9 dB more signal (18-22.5 MHz), and moves the filter's dip from 16.5
   to 27 MHz (2026-10-17, through 30 dB. An earlier sweep through 60 dB was distorted by the
   generator's leak). 2-12 beats 12-30 up to about 12.7 MHz. The 500 kHz low pass replaces
   the 2 MHz one where the whole filter band fits under it (mirisdr_tune_words()). */
#define DXR2_LPF500     0x3FFF  /* B7 B6 low: the 500 kHz low pass */
#define DXR2_LPF500_HZ  500000
#define DXR2_LPF2       0x3DFF  /* and B1 low: the 2 MHz low pass */

/* The amplifier A5 low takes in (released bypasses it), on port A against the bypass,
   2026-10-17: 19-22 dB at 45-50 MHz (it rolls off below 40), 24-25 from 60 to 300, 22-23
   at 433-600, 20 at 900, 17-16 at 1.2-1.5 GHz, 13-14 at 1.8-2 GHz. Counted from 55 MHz */
#define DXR2_AMP_VHF    24
#define DXR2_AMP_B3     25
#define DXR2_AMP_UHF    24
#define DXR2_AMP_B45    21
#define DXR2_AMP_L      15
#define DXR2_2_12_KHZ   1700    /* the 2-12 filter beats the 2 MHz low pass from 1.7 MHz: 1.4 dB at 1.85,
                                   6.2 at 1.99 (2026-10-17) */
#define DXR2_380_420_KHZ 345000 /* the 380-420 filter beats the 300-380 one from 344 MHz, up to 1.4 dB
                                   at 360 (2026-10-17) */

static hw_switch_freq_plan_t hw_switch_freq_plan_rspdxr2[] = {
        {0,       MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, DXR2_LPF2, 0},
        {DXR2_2_12_KHZ, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_2_12, 0},
        {13000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_12_30, 0},
        RSP1B_30_60_ROWS,
        {55000,   MIRISDR_MODE_VHF, 0, 0, 32, 0xea80, 0, RSP1B_VHF_B45, DXR2_AMP_VHF},
        {108000,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_B3_L, DXR2_AMP_B3},
        {250000,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_B3_L, DXR2_AMP_B3},
        {255000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 6, RSP1B_250_300, DXR2_AMP_UHF},
        {280000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_250_300, DXR2_AMP_UHF},
        {300000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_300_380, DXR2_AMP_UHF},
        {DXR2_380_420_KHZ, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_380_420,
         DXR2_AMP_UHF},
        {420000,  MIRISDR_MODE_B45, 0, 0, 4,  0xea80, 0, RSP1B_VHF_B45, DXR2_AMP_B45},
        {960000,  MIRISDR_MODE_BL,  0, 0, 2,  0xea80, 0, RSP1B_B3_L, DXR2_AMP_L},
        {2400000, -1, 0, 0, 0, 0x0000, 0, 0, 0},
};

/* RSPduo (sdrplay.c): the filter in tuner 1's pins of the first expander, B1-B5, and tuner 0
   takes the same code on A0-A4 (mirisdr_duo_filter). Checked on each tuner's own port,
   2026-10-17: the plan's word the best or within 0.5 dB from 50 MHz to 1.8 GHz. HF: B4 B5
   low take the HF bank, B1-B3 pick its filter. (A5, once held low here, switches the
   reference output on: MIRISDR_FRONTEND_REF_OUT) */
#define DUO_LPF2        0xC07F  /* B1-B5 low: the 2 MHz low pass */
#define DUO_2_12        0xC47F  /* B1 B3 B4 B5 */
#define DUO_12_30       0xC27F  /* B2 B3 B4 B5 */
#define DUO_30_60       0xC67F  /* B3 B4 B5 low */
#define DUO_VHF_UHF     0xCC7F  /* B1 B4 B5: 60-100 and 420-900 MHz */
#define DUO_B3          0xEC7F  /* B1 B4: 110-240 MHz, and L band */
#define DUO_250_300     0xE47F  /* B1 B3 B4 */
#define DUO_300_380     0xD47F  /* B1 B3 B5 */
#define DUO_380_420     0xF47F  /* B1 B3 */
#define DUO_2_12_KHZ    1600    /* the 2-12 filter beats the 2 MHz low pass from 1.65 MHz on tuner 0,
                                   1.55 on tuner 1, and 2.4-2.8 dB at 1.95 MHz (2026-10-17) */
#define DUO_380_420_KHZ 350000  /* it beats the 300-380 filter from here with the amplifier on: they
                                   cross at 349 MHz on tuner 0, 351 on tuner 1 (2026-10-17) */

/* The amplifier from 55 MHz: the code with B2 low as well (sdrplay.c's DUO1_AMP1) and its
   supply on (chip 2, DUO2_AMP0/1), against the direct path, the plan's code, on each
   chain's own port, 2026-10-17: 24-25 dB from 60 to 150 MHz, 22-25 from 200 to 260, 20-23
   from 280 to 430, 19-21 from 500 to 900, 15-19 from 1 to 1.8 GHz, 7-12 at 2 GHz. The
   means of both chains */
#define DUO_AMP_VHF     25
#define DUO_AMP_B3      24
#define DUO_AMP_UHF     22
#define DUO_AMP_B45     20
#define DUO_AMP_L       17

static hw_switch_freq_plan_t hw_switch_freq_plan_rspduo[] = {
        {0,       MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, DUO_LPF2, 0},
        {DUO_2_12_KHZ, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, DUO_2_12, 0},
        {12000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, DUO_12_30, 0},
        {30000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, DUO_30_60, 0},
        {55000,   MIRISDR_MODE_VHF, 0, 0, 32, 0xea80, 0, DUO_VHF_UHF, DUO_AMP_VHF},
        {108000,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, DUO_B3, DUO_AMP_B3},
        {255000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 6, DUO_250_300, DUO_AMP_UHF},
        {280000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, DUO_250_300, DUO_AMP_UHF},
        {300000,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, DUO_300_380, DUO_AMP_UHF},
        {DUO_380_420_KHZ, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, DUO_380_420,
         DUO_AMP_UHF},
        {420000,  MIRISDR_MODE_B45, 0, 0, 4,  0xea80, 0, DUO_VHF_UHF, DUO_AMP_B45},
        {960000,  MIRISDR_MODE_BL,  0, 0, 2,  0xea80, 0, DUO_B3, DUO_AMP_L},
        {2400000, -1, 0, 0, 0, 0x0000, 0, 0, 0},
};

/* RSPduo tuner 0 on the Hi-Z input: the tuner's AM input 1, past the filters (whose words,
   switching where the 50 ohm plan's do, keep that port's path as without it). Flat -6..-8
   dBFS from 0.1 to 12 MHz at mixbuffer 0, ~8 dB less above (2026-10-17). A tune ends where
   its LO reaches 30 MHz, at a low IF the frequency an IF below that */
#define DUO_HIZ_END_MHZ 30

static hw_switch_freq_plan_t hw_switch_freq_plan_rspduo_hiz[] = {
        {0,       MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT1, 16, 0xea80, 0, DUO_LPF2, 0},
        {DUO_2_12_KHZ, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT1, 16, 0xea80, 0, DUO_2_12, 0},
        {12000,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT1, 16, 0xea80, 0, DUO_12_30, 0},
        {DUO_HIZ_END_MHZ * 1000, -1, 0, 0, 0, 0x0000, 0, 0, 0},
};

static hw_switch_freq_plan_t *hw_switch_freq_plan[5] = {
        hw_switch_freq_plan_default,
        hw_switch_freq_plan_sdrplay,
        hw_switch_freq_plan_rsp1b,
        hw_switch_freq_plan_rspdxr2,
        hw_switch_freq_plan_rspduo
};

/* The board's plan, for the input in p->antenna. Both mirisdr_open_ex() and
   mirisdr_set_hw_flavour() refuse other values */
static const hw_switch_freq_plan_t *mirisdr_plan_of (mirisdr_dev_t *p)
{
    int f = (int) p->hw_flavour;

    if ((f == MIRISDR_HW_RSPDUO) && !p->tuner_cur && (p->antenna == MIRISDR_ANTENNA_HIZ))
        return hw_switch_freq_plan_rspduo_hiz;

    return hw_switch_freq_plan[(f >= MIRISDR_HW_DEFAULT && f <= MIRISDR_HW_RSPDUO) ? f : MIRISDR_HW_DEFAULT];
}

/* The row a frequency falls in */
static const hw_switch_freq_plan_t *mirisdr_plan_row_at (mirisdr_dev_t *p, uint32_t freq)
{
    const hw_switch_freq_plan_t *plan = mirisdr_plan_of(p);
    int i = 0;

    while ((uint64_t) freq >= 1000ULL * plan[i].low_khz && plan[i].mode >= 0) i++;

    return &plan[i - 1];
}

static const hw_switch_freq_plan_t *mirisdr_plan_row (mirisdr_dev_t *p)
{
    return mirisdr_plan_row_at(p, p->freq);
}

/* Where the plan ends: tuning past it is refused by mirisdr_tune() */
static uint32_t mirisdr_plan_end (mirisdr_dev_t *p)
{
    const hw_switch_freq_plan_t *plan = mirisdr_plan_of(p);
    int i = 0;

    while (plan[i].mode >= 0) i++;

    return plan[i].low_khz * 1000U;
}

/* The tuner's row follows the LO. The front end's filter follows the received
   frequency, which a low IF or an offset moves a little from it, as long as both rows
   use the same tuner input: otherwise the path to that input wins. */
static uint16_t mirisdr_frontend_word (mirisdr_dev_t *p, const hw_switch_freq_plan_t *row, uint32_t rx)
{
    const hw_switch_freq_plan_t *r = mirisdr_plan_row_at(p, rx);

    if ((r->mode == row->mode) && (r->am_port == row->am_port) &&
        (r->upconvert_mixer_on == row->upconvert_mixer_on)) return r->expander;

    return row->expander;
}

/* The tuner's IF filters, Hz, by bandwidth index */
static const uint32_t mirisdr_bw_hz[] = { 200000, 300000, 600000, 1536000, 5000000, 6000000,
                                          7000000, 8000000, 14000000 };

/* What one tune writes, worked out without touching the device */
typedef struct mirisdr_tune_words
{
    uint32_t reg0, reg2, reg3, reg5, regd;
    uint32_t lo_real;           /* Hz the LO reaches, as the received frequency */
    mirisdr_band_t band;
    uint8_t dc_n, gap;
    uint8_t fits;               /* N fits its 6 bits: past ~3.07 GHz it would wrap */
    uint16_t expander;
    const hw_switch_freq_plan_t *row;
} mirisdr_tune_words_t;

static void mirisdr_tune_words (mirisdr_dev_t *p, uint32_t lo, uint32_t rx, int ifm, int bw, int iq,
                                int cal, uint32_t fixed, mirisdr_tune_words_t *w)
{
    uint32_t reg0 = 0, reg2 = 0, reg5 = 0, reg3 = 0, regd = 0, xsel;
    uint64_t n, thresh, frac, lo_div = 0, fvco = 0, rfvco = 0, offset = 0, afc = 0, a, b, c, flo, synth;
    const hw_switch_freq_plan_t *row = mirisdr_plan_row_at(p, lo);

    w->row = row;
    w->expander = mirisdr_frontend_word(p, row, rx);

    /* the 500 kHz low pass only where it cuts nothing the 2 MHz one passes */
    if ((p->hw_flavour == MIRISDR_HW_RSPDXR2) && (w->expander == DXR2_LPF2) &&
        ((uint64_t) rx + mirisdr_bw_hz[bw] / 2 <= DXR2_LPF500_HZ)) w->expander = DXR2_LPF500;

    if (row->mode == MIRISDR_MODE_AM)
    {
        reg0 |= MIRISDR_MODE_AM;
        reg0 |= row->upconvert_mixer_on << 5;
        reg0 |= row->am_port << 7;

        /* 255-420 MHz goes low side instead (below), high side puts the VCO far out of range */
        if (row->upconvert_mixer_on && !row->if1_low) offset += 120000000UL;

        lo_div = 16;
        w->band = (row->am_port == 0) ? MIRISDR_BAND_AM1 : MIRISDR_BAND_AM2;
    }
    else
    {
        reg0 |= row->mode;
        lo_div = row->lo_div;

        if (row->mode == MIRISDR_MODE_VHF) w->band = MIRISDR_BAND_VHF;
        /* the RSP1's 261-404 MHz row sets the VHF bit with it */
        else if (row->mode & MIRISDR_MODE_B3) w->band = MIRISDR_BAND_3;
        else if (row->mode == MIRISDR_MODE_B45) w->band = MIRISDR_BAND_45;
        else w->band = MIRISDR_BAND_L;
    }

    /* the RF synthesizer is always on */
    reg0 |= MIRISDR_RF_SYNTHESIZER_ON << 6;

    switch (ifm)
    {
    case MIRISDR_IF_ZERO:    reg0 |= MIRISDR_IF_MODE_ZERO << 8; break;
    case MIRISDR_IF_450KHZ:  reg0 |= MIRISDR_IF_MODE_450KHZ << 8; break;
    case MIRISDR_IF_1620KHZ: reg0 |= MIRISDR_IF_MODE_1620KHZ << 8; break;
    case MIRISDR_IF_2048KHZ: reg0 |= MIRISDR_IF_MODE_2048KHZ << 8; break;
    }

    /* the IF filter: the widest code with its filter bypassed (register 13) is BW_MAX */
    reg0 |= (uint32_t) ((bw == MIRISDR_BW_MAX) ? 7 : bw) << 10;
    if (bw == MIRISDR_BW_MAX) regd |= 1;

    /* the gap rows' first IF follows XTALSEL instead: 001 x6, 000 x7 */
    xsel = row->if1_low ? (uint32_t) (7 - row->if1_low) : mirisdr_xtalsel(p);
    reg0 |= xsel << 13;
    w->dc_n = mirisdr_xtalsel_n[xsel];

    /* which recalibrates the IF filter for that crystal, see mirisdr_reg13 */
    w->gap = row->if1_low != 0;

    /* the baseband amplifiers of the output not wanted can be switched off */
    reg0 |= (uint32_t) ((iq == MIRISDR_IQ_ONLY_I) ? MIRISDR_IF_LPMODE_ONLY_I :
                        (iq == MIRISDR_IQ_ONLY_Q) ? MIRISDR_IF_LPMODE_ONLY_Q : MIRISDR_IF_LPMODE_NORMAL) << 16;
    reg0 |= MIRISDR_VCO_LPMODE_NORMAL << 19;

    flo = lo + offset - (uint64_t) row->if1_low * 24000000UL;
    fvco = flo * lo_div;

    /* N, then the fraction over a threshold reduced to fit 12 bits */
    n = fvco / 96000000UL;

    if (fixed)
    {
        /* the nearest point of the grid, no AFC: only register 2 differs between them */
        thresh = fixed;
        frac = ((fvco % 96000000UL) * thresh + 48000000UL) / 96000000UL;
        if (frac == thresh)
        {
            n++;
            frac = 0;
        }
    }
    else
    {
        thresh = 96000000UL / lo_div;
        frac = (fvco % 96000000UL) / lo_div;

        for (a = thresh, b = frac; a != 0;)
        {
            c = a;
            a = b % a;
            b = c;
        }

        thresh /= b;
        frac /= b;

        a = (thresh + 4094) / 4095;
        thresh = (thresh + (a / 2)) / a;
        frac = (frac + (a / 2)) / a;

        rfvco = (96000000UL * (n * thresh * 4096UL + (frac * 4096UL))) / (thresh * 4096UL * lo_div);
        if (flo < rfvco) frac--;
        rfvco = (96000000UL * (n * thresh * 4096UL + (frac * 4096UL + afc))) / (thresh * 4096UL * lo_div);
        afc = ((flo - rfvco) * thresh * 4096UL * lo_div) / 96000000UL;
    }

    reg3 |= (afc & 4095);
    reg5 |= (0xFFF & thresh);
    reg5 |= MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING << 12;   /* reserved, must be 0x28 */

    w->fits = n <= 0x3F;

    reg2 |= (0xFFF & frac);
    reg2 |= (0x3F & n) << 12;
    reg2 |= (uint32_t) (cal ? MIRISDR_LBAND_LNA_CALIBRATION_ON : MIRISDR_LBAND_LNA_CALIBRATION_OFF) << 18;

    /* the synthesizer as programmed, back in terms of the received frequency */
    synth = (96000000ULL * (n * thresh * 4096ULL + frac * 4096ULL + (afc & 4095)) +
             thresh * 2048ULL * lo_div) / (thresh * 4096ULL * lo_div);
    w->lo_real = (uint32_t) (synth - offset + (uint64_t) row->if1_low * 24000000UL);

    w->reg0 = reg0;
    w->reg2 = reg2;
    w->reg3 = reg3;
    w->reg5 = reg5;
    w->regd = regd;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr tune: lo %u rx %u sel %x, n %lu, fraction %lu/%lu, LO %u\n", lo, rx,
            row->band_select_word, (long unsigned) n, (long unsigned) frac, (long unsigned) thresh, w->lo_real);
#endif
}

/* RSPduo: the tuners share the latch, so each word to one makes the other take its last
   word again. A synthesizer or filter word taken again restarts it, a ~0.2 ms glitch and
   a phase step (seen 2026-10-17, no expander pin keeps it out). Before the first expander
   word to moves the words to another tuner, the tuner they leave gets a register 4 word,
   which does nothing, however often it is taken. It also ends a readback's hold on the
   answer line, which a route moved away from it would turn against the other tuner */
static int mirisdr_duo_close (mirisdr_dev_t *p, uint16_t to)
{
    int t, cur = p->tuner_cur, r = 0;

    if ((p->hw_flavour != MIRISDR_HW_RSPDUO) || p->external_tuner || !(p->exp_valid & 1)) return 0;

    t = mirisdr_duo_routed(p->exp_iodir[0]);
    if ((t < 0) || (t == mirisdr_duo_routed(to))) return 0;

    if (t != cur) mirisdr_tuner_select(p, t);
    if ((p->tuner_last != MIRISDR_TUNER_IDLE) && (mirisdr_tuner_write(p, MIRISDR_TUNER_IDLE, 0, 1) < 0)) r = -1;
    if (t != cur) mirisdr_tuner_select(p, cur);

    return r;
}

/* RSPduo: the tuner words to tuner n, for what is sent outside a tune (the readback). A
   raw first word stays as given */
static int mirisdr_duo_route_to (mirisdr_dev_t *p, int n)
{
    uint16_t w;

    if ((p->hw_flavour != MIRISDR_HW_RSPDUO) || !(p->fe.fields & MIRISDR_FRONTEND_RAW_EXP1) ||
        (p->fe_raw & MIRISDR_FRONTEND_RAW_EXP1)) return 0;

    w = mirisdr_duo_route(p->fe.expander[0], mirisdr_stream_tuners(p), n);
    if (w == p->fe.expander[0]) return 0;
    p->fe.expander[0] = w;

    if (mirisdr_duo_close(p, w) < 0) return -1;

    return mirisdr_sdrplay_send(p, 0, w);
}

/* The parts of the front end a board has, MIRISDR_FRONTEND_RAW_* */
static uint32_t mirisdr_frontend_parts (int flavour)
{
    return MIRISDR_FRONTEND_RAW_GPIO | (MIRISDR_HW_EXPANDER(flavour) ? MIRISDR_FRONTEND_RAW_EXP1 : 0) |
           ((flavour == MIRISDR_HW_RSPDUO) ? MIRISDR_FRONTEND_RAW_EXP2 : 0);
}

/* The values frontend.attenuator takes on a board, dB from 0 up, into v (room for
   MIRISDR_ATTENUATOR_MAX). Returns the count */
static int mirisdr_attenuator_list (int flavour, int *v)
{
    int db, n = 0;

    for (db = 0; db <= 36; db++)
        if ((flavour == MIRISDR_HW_RSPDUO || flavour == MIRISDR_HW_RSP1B) ? (mirisdr_duo_pad(db) >= 0) :
            (flavour == MIRISDR_HW_RSPDXR2) ? (mirisdr_dxr2_pads(db) >= 0) : !db)
        {
            if (v) v[n] = db;
            n++;
        }

    return n;
}

static int mirisdr_attenuator_ok (int flavour, int db)
{
    int v[MIRISDR_ATTENUATOR_MAX], n = mirisdr_attenuator_list(flavour, v);

    while (n--) if (v[n] == db) return 1;

    return 0;
}

/* The pads for an attenuation: strict, the value itself or -1. Otherwise the nearest step
   (the smaller of two as near), into 0 to the board's most. A board without pads takes 0
   only, either way */
static int mirisdr_attenuator_snap (int flavour, int db, int strict)
{
    int v[MIRISDR_ATTENUATOR_MAX], n = mirisdr_attenuator_list(flavour, v), k, best = 0;

    if (strict || (n < 2)) return mirisdr_attenuator_ok(flavour, db) ? db : -1;

    for (k = 1; k < n; k++)
        if (abs(v[k] - db) < abs(v[best] - db)) best = k;

    return v[best];
}

/* The flags of the front end a board has, MIRISDR_FRONTEND_* */
static uint32_t mirisdr_frontend_flags (int flavour)
{
    return (flavour == MIRISDR_HW_RSPDUO) ? (MIRISDR_FRONTEND_PORT_TIE | MIRISDR_FRONTEND_REF_OUT) : 0;
}

/* The tuner words, after the front end: on the RSPduo the first expander decides which
   tuner takes them. raw: the parts of fe a tune replaced */
static int mirisdr_tune_send (mirisdr_dev_t *p, const mirisdr_tune_words_t *w, const mirisdr_frontend_raw_t *fe,
                              uint32_t raw)
{
    int r = 0;

    if ((fe->fields & MIRISDR_FRONTEND_RAW_EXP1) &&
        ((mirisdr_duo_close(p, fe->expander[0]) < 0) || (mirisdr_sdrplay_send(p, 0, fe->expander[0]) < 0))) r = -1;
    if ((fe->fields & MIRISDR_FRONTEND_RAW_EXP2) && (mirisdr_sdrplay_send(p, 1, fe->expander[1]) < 0)) r = -1;

    p->reg8 = (w->row->band_select_word & ~0xFF00u) | ((uint32_t) fe->gpio_dir << 12) | ((uint32_t) fe->gpio_level << 8);
    p->fe_raw = raw;
    if (update_reg_8(p) < 0) r = -1;

    p->band = w->band;
    p->dc_n = w->dc_n;
    p->tuner_gap = w->gap;

    /* register 9 is the MSi001's serial port: a board with another tuner on it
       has this port wired somewhere else, or nowhere */
    if (!p->external_tuner)
    {
        int synth = 0, s[7], i;

        /* only what changed; register 2 starts the calibration, so it follows any of 0, 3 and 5 */
        s[0] = mirisdr_tuner_write(p, 14, p->tuner_ovr14, 0);
        s[1] = mirisdr_tuner_write(p, 6, mirisdr_dc_word(p), 0);
        synth |= (s[2] = mirisdr_tuner_write(p, 3, w->reg3, 0)) != 0;
        synth |= (s[3] = mirisdr_tuner_write(p, 0, w->reg0, 0)) != 0;
        synth |= (s[4] = mirisdr_tuner_write(p, 5, w->reg5, 0)) != 0;
        s[5] = mirisdr_tuner_write(p, 2, w->reg2, synth);
        p->tuner_regd = w->regd;
        s[6] = mirisdr_tuner_write(p, 13, mirisdr_reg13(p), 0);

        for (i = 0; i < 7; i++) if (s[i] < 0) r = -1;
    }

    return r;
}

/* The gap rows name another crystal for their first IF, and the IF filter
   calibrates against the named one.  Read the code the real setting gave once,
   on the way in from a normal band (open tunes one), to hold it there. */
static void mirisdr_filter_learn (mirisdr_dev_t *p, const hw_switch_freq_plan_t *row)
{
    mirisdr_tuner_status_t st;

    if (p->filter_cal >= 0 || p->external_tuner || !row->if1_low || !(p->tuner_valid & 1)) return;
    if ((p->tuner_reg[0] & (7UL << 17)) != mirisdr_xtalsel(p) << 17) return;

    if (mirisdr_get_tuner_status(p, 0, &st) == 0) p->filter_cal = st.filter;
}

/* Datasheet table 19: the synthesizer frequency that calibrates the L-band LNA to centre */
#define MIRISDR_LNA_CAL_US      1000
#define MIRISDR_LNA_CAL_MIN     1000    /* MHz: lower lands on code 15 as well, outside L band */

static const uint16_t mirisdr_lna_cal_vco[] = { 2380, 2400, 2420, 2440, 2480, 2500, 2520, 2540, 2560, 2580 };
static const uint16_t mirisdr_lna_cal_centre[] = { 1440, 1460, 1500, 1530, 1570, 1600, 1640, 1670, 1700, 1750 };

/* The tune that calibrates the LNA for this frequency, MHz; beyond the table the end
   segments carry on, which still lands on the end codes (15 below ~1350, 0 above ~1800) */
static int mirisdr_lna_cal_tune (int mhz)
{
    int n = sizeof mirisdr_lna_cal_vco / sizeof mirisdr_lna_cal_vco[0], i = 1, f;

    while (i < n - 1 && mhz > mirisdr_lna_cal_centre[i]) i++;

    f = (mirisdr_lna_cal_vco[i - 1] + (mhz - mirisdr_lna_cal_centre[i - 1]) *
         (mirisdr_lna_cal_vco[i] - mirisdr_lna_cal_vco[i - 1]) /
         (mirisdr_lna_cal_centre[i] - mirisdr_lna_cal_centre[i - 1])) / 2;

    return (f < MIRISDR_LNA_CAL_MIN) ? MIRISDR_LNA_CAL_MIN : f;
}

/* centred on the frequency received */
static int mirisdr_lna_calibrate (mirisdr_dev_t *p, const hw_switch_freq_plan_t *row, uint32_t rx,
                                  int ifm, int bw, int iq, const mirisdr_frontend_raw_t *fe, uint32_t raw)
{
    int mhz = (int) (rx / 1000000), r;
    uint32_t cal;
    mirisdr_tune_words_t w;

    if (p->external_tuner || row->mode != MIRISDR_MODE_BL || (p->tuner_ovr13 & 0x40)) return 0;
    if ((p->tuner_valid & 1) && p->lna_cal_mhz && abs(mhz - p->lna_cal_mhz) <= 50) return 0;

    cal = (uint32_t) mirisdr_lna_cal_tune(mhz) * 1000000;
    mirisdr_tune_words(p, cal, cal, ifm, bw, iq, 1, 0, &w);
    r = mirisdr_tune_send(p, &w, fe, raw);

    /* it needs the synthesizer settled, which can take ~400 us; 20 us sufficed in tests */
    if (p->fw_ours && p->fw_list_at) r |= mirisdr_write_reg(p, MIRISDR_LIST_WAIT_US, MIRISDR_LNA_CAL_US);
    else { r |= mirisdr_batch_flush(p); usleep(MIRISDR_LNA_CAL_US); }

    if (r >= 0) p->lna_cal_mhz = mhz;

    return r < 0 ? r : 0;
}

/* ------------------------------------------------------------------ */
/* the tune config                                                     */
/* ------------------------------------------------------------------ */

#define MIRISDR_TUNE_ADJUST     1       /* the single setters: no combination checks */

static int mirisdr_if_mode (uint32_t hz)
{
    switch (hz)
    {
    case 0:       return MIRISDR_IF_ZERO;
    case 450000:  return MIRISDR_IF_450KHZ;
    case 1620000: return MIRISDR_IF_1620KHZ;
    case 2048000: return MIRISDR_IF_2048KHZ;
    }

    return -1;
}


/* which filters each IF allows, by bit MIRISDR_BW_*: table 12 notes 3 and table 13 */
static const uint16_t mirisdr_if_bws[] = {
    (1 << MIRISDR_BW_1536KHZ) | (1 << MIRISDR_BW_5MHZ) | (1 << MIRISDR_BW_6MHZ) |
    (1 << MIRISDR_BW_7MHZ) | (1 << MIRISDR_BW_8MHZ) | (1 << MIRISDR_BW_MAX),
    (1 << MIRISDR_BW_200KHZ) | (1 << MIRISDR_BW_300KHZ) | (1 << MIRISDR_BW_600KHZ),
    (1 << MIRISDR_BW_600KHZ) | (1 << MIRISDR_BW_1536KHZ),
    (1 << MIRISDR_BW_600KHZ) | (1 << MIRISDR_BW_1536KHZ),
};

/* The filter for a bandwidth with an IF: the narrowest it has as wide, the widest if none is */
static int mirisdr_bw_snap (int ifm, uint32_t hz)
{
    int i, n = (int) (sizeof mirisdr_bw_hz / sizeof mirisdr_bw_hz[0]), widest = -1;

    for (i = 0; i < n; i++)
    {
        if (!(mirisdr_if_bws[ifm] & (1 << i))) continue;
        if (mirisdr_bw_hz[i] >= hz) return i;
        widest = i;
    }

    return widest;
}
static const uint8_t mirisdr_if_bw_widest[] = { MIRISDR_BW_8MHZ, MIRISDR_BW_600KHZ,
                                                MIRISDR_BW_1536KHZ, MIRISDR_BW_1536KHZ };

/* the widest filter of a low IF, in Hz: what a baseband stream's converters must hold */
static uint32_t mirisdr_if_widest_hz (uint32_t if_hz)
{
    unsigned i;

    for (i = 1; i < sizeof mirisdr_if_hz / sizeof mirisdr_if_hz[0]; i++)
        if (mirisdr_if_hz[i] == if_hz) return mirisdr_bw_hz[mirisdr_if_bw_widest[i]];

    return 0;
}

/* The rate the stream runs at after a tune to zero IF. One that follows the tune
   may change it there, a baseband one from its low IF rate. Without a stream yet
   (the null device), the rate a device opens with */
static uint32_t mirisdr_zero_if_rate (mirisdr_dev_t *p)
{
    mirisdr_stream_plan_t spl;
    uint32_t rate = p->rate ? p->rate : DEFAULT_RATE;
    int checking = p->checking, r;

    if ((p->if_freq == MIRISDR_IF_ZERO) || !(p->stream.follow_tune || p->stream.baseband)) return rate;

    p->checking = 1;
    r = mirisdr_stream_plan(p, &p->stream, MIRISDR_STREAM_ADJUST, MIRISDR_IQ_BOTH, 0, 0, &spl);
    p->checking = checking;

    return (r < 0) ? rate : spl.rate;
}

/* Bandwidth 0: at zero IF the narrowest filter as wide as the rate, which passes the
   whole stream and keeps out what the converter would only have to filter away. 8 MHz
   at most, as the wider setting has no filter. A low IF filters in software, so its
   widest */
static int mirisdr_bw_auto (int ifm, uint32_t rate)
{
    int b = MIRISDR_BW_1536KHZ;

    if (ifm != MIRISDR_IF_ZERO) return mirisdr_if_bw_widest[ifm];

    while ((b < MIRISDR_BW_8MHZ) && (mirisdr_bw_hz[b] < rate)) b++;

    return b;
}

typedef struct mirisdr_tune_plan
{
    mirisdr_tune_config_t cfg;  /* as asked, bandwidth 0 kept */
    int ifm, bw, iq;
    int ds, ds_lna;             /* direct sampling in force, its amplifier on */
    int siq;                    /* what the stream sees: iq, or the I converter alone */
    uint32_t sif, ds_edge;      /* the stream's IF (the frequency under direct sampling),
                                   the low pass direct sampling runs through */
    uint32_t lo, ovr13, ovr14;
    mirisdr_tune_words_t w;
    mirisdr_frontend_raw_t fe;  /* the front end words it sends, every part the board has */
    uint32_t fe_raw;            /* the parts of fe the tune replaced */
    mirisdr_gr_t gr;            /* the gain reductions it ends up with */
    int att;                    /* dB of pads in force, frontend.attenuator snapped */
} mirisdr_tune_plan_t;

/* The words the front end gets: the plan's with the frontend fields, then the raw parts
   asked for in their place */
static void mirisdr_frontend_words (mirisdr_dev_t *p, const mirisdr_tune_config_t *c, mirisdr_tune_plan_t *pl)
{
    const mirisdr_frontend_raw_t *raw = &c->frontend.raw;
    uint32_t r8 = pl->w.row->band_select_word;

    if (c->frontend.bias && !MIRISDR_HW_EXPANDER(p->hw_flavour)) r8 |= 1u << (BIAS_GPIO + 8);
    if (pl->ds) r8 |= 1u << (DS_GPIO + 8);

    memset(&pl->fe, 0, sizeof pl->fe);
    pl->fe.fields = mirisdr_frontend_parts(p->hw_flavour);
    pl->fe.gpio_dir = (uint8_t) ((r8 >> 12) & 15);
    pl->fe.gpio_level = (uint8_t) ((r8 >> 8) & 15);
    if (pl->fe.fields & MIRISDR_FRONTEND_RAW_EXP1)
        pl->fe.expander[0] = mirisdr_sdrplay_word(p->hw_flavour, pl->w.expander, c->frontend.notch, c->frontend.bias,
                                                  c->frontend.antenna, pl->ds, pl->ds_lna,
                                                  pl->gr.amp_db && pl->gr.amp, pl->att);
    /* RSPduo: the filters of the tuners the stream takes, this one's and the other's (of
       its tune in force, if any), each through its amplifier (path and supply) unless
       bypassed. A chain not streamed gets no path and its amplifier no supply, which keeps
       it from passing its port to the other (up to 20 dB at 300-433 MHz). Both tuners'
       pads and notches, tuner 1's bias-T, the tie between the ports and the reference
       output, and the converters as the stream takes the tuners, the tuner words to this one */
    if (p->hw_flavour == MIRISDR_HW_RSPDUO)
    {
        int n = p->tuner_cur, s = mirisdr_stream_tuners(p), notch[2], att[2], amp[2] = { 0, 0 };
        const mirisdr_tuner_ctx_t *o = &p->tctx[1 - n];
        uint16_t w = 0xFFFF;

        if (s & (1 << n))
        {
            amp[n] = pl->gr.amp_db && !pl->gr.amp;
            w = mirisdr_duo_filter(w, n, amp[n] ? pl->w.expander & ~DUO1_AMP1 : pl->w.expander);
        }
        if ((s & (1 << (1 - n))) && o->tune_exp)
        {
            amp[1 - n] = o->amp_db && !o->gain_reduction_amp;
            w = mirisdr_duo_filter(w, 1 - n, amp[1 - n] ? o->tune_exp & ~DUO1_AMP1 : o->tune_exp);
        }
        notch[n] = c->frontend.notch;
        notch[1 - n] = o->notch;
        att[n] = pl->att;
        att[1 - n] = o->attenuator;
        pl->fe.expander[0] = mirisdr_duo_route(w, mirisdr_stream_tuners(p), n);
        if (c->frontend.flags & MIRISDR_FRONTEND_REF_OUT) pl->fe.expander[0] &= (uint16_t) ~DUO1_REF_OUT;
        pl->fe.expander[1] = mirisdr_duo_word2(notch, n ? c->frontend.bias : p->tctx[1].bias,
                                               (n ? p->tctx[0].antenna : c->frontend.antenna) == MIRISDR_ANTENNA_HIZ, att,
                                               !!(c->frontend.flags & MIRISDR_FRONTEND_PORT_TIE), amp);
    }

    pl->fe_raw = raw->fields;
    if (raw->fields & MIRISDR_FRONTEND_RAW_GPIO)
    {
        pl->fe.gpio_dir = raw->gpio_dir;
        pl->fe.gpio_level = raw->gpio_level;
    }
    if (raw->fields & MIRISDR_FRONTEND_RAW_EXP1) pl->fe.expander[0] = raw->expander[0];
    if (raw->fields & MIRISDR_FRONTEND_RAW_EXP2) pl->fe.expander[1] = raw->expander[1];
}

static int mirisdr_tune_plan (mirisdr_dev_t *p, const mirisdr_tune_config_t *c, int flags,
                              mirisdr_tune_plan_t *pl)
{
    int adjust = flags & MIRISDR_TUNE_ADJUST, i, hiz, ant;
    mirisdr_gain_config_t g;
    int64_t lo;

    pl->cfg = *c;

    if (MIRISDR_RESERVED_SET(*c) || MIRISDR_RESERVED_SET(c->gain) || MIRISDR_RESERVED_SET(c->frontend) ||
        MIRISDR_RESERVED_SET(c->frontend.raw) || MIRISDR_RESERVED_SET(c->override))
    {
        mirisdr_refuse(p, MIRISDR_RESERVED_MSG, "the tune config");
        return -1;
    }

    if ((pl->ifm = mirisdr_if_mode(c->if_freq)) < 0)
    {
        mirisdr_refuse(p, "unsupported if frequency: %u Hz\n", c->if_freq);
        return -1;
    }

    if (!c->bandwidth) pl->bw = mirisdr_bw_auto(pl->ifm, (pl->ifm == MIRISDR_IF_ZERO) ? mirisdr_zero_if_rate(p) : 0);
    else
    {
        for (i = 0; (i < (int) (sizeof mirisdr_bw_hz / sizeof mirisdr_bw_hz[0])) &&
                    (mirisdr_bw_hz[i] != c->bandwidth); i++);

        if (i == (int) (sizeof mirisdr_bw_hz / sizeof mirisdr_bw_hz[0]))
        {
            if (c->strict)
            {
                mirisdr_refuse(p, "unsupported bandwidth: %u Hz\n", c->bandwidth);
                return -1;
            }
            i = mirisdr_bw_snap(pl->ifm, c->bandwidth);
        }

        pl->bw = i;
    }

    /* not with this IF: the nearest it has, as above, unless strict */
    if (!adjust && !c->strict && !(mirisdr_if_bws[pl->ifm] & (1 << pl->bw)))
        pl->bw = mirisdr_bw_snap(pl->ifm, c->bandwidth);

    if (!(mirisdr_if_bws[pl->ifm] & (1 << pl->bw)))
    {
        if (!adjust)
        {
            mirisdr_refuse(p, "a %u Hz bandwidth is not available with a %u Hz IF\n", mirisdr_bw_hz[pl->bw], c->if_freq);
            return -1;
        }

        /* a single setter: the filter goes back to the library's choice, which
           an IF has, so the tune stays one a check takes */
        pl->bw = mirisdr_bw_auto(pl->ifm, (pl->ifm == MIRISDR_IF_ZERO) ? mirisdr_zero_if_rate(p) : 0);
        pl->cfg.bandwidth = 0;
    }

    if ((c->iq < MIRISDR_IQ_BOTH) || (c->iq > MIRISDR_IQ_ONLY_Q))
    {
        mirisdr_refuse(p, "unsupported iq selection: %d\n", c->iq);
        return -1;
    }

    pl->iq = c->iq;

    /* one output alone cannot tell the two sides of the LO apart */
    if (pl->iq != MIRISDR_IQ_BOTH && pl->ifm == MIRISDR_IF_ZERO)
    {
        if (!adjust)
        {
            mirisdr_refuse(p, "a single tuner output needs a low IF\n");
            return -1;
        }

        pl->iq = MIRISDR_IQ_BOTH;
    }

    /* RSPdx-R2 direct sampling: allowed, and in force at its frequencies */
    if (((c->frontend.direct_sampling != MIRISDR_DS_OFF) && (c->frontend.direct_sampling != MIRISDR_DS_ON) &&
         (c->frontend.direct_sampling != MIRISDR_DS_AUTO)) ||
        (c->frontend.direct_sampling && (p->hw_flavour != MIRISDR_HW_RSPDXR2)))
    {
        mirisdr_refuse(p, "unsupported direct sampling %d: off, on or auto, on the RSPdx-R2 only\n",
                       c->frontend.direct_sampling);
        return -1;
    }

    pl->ds = mirisdr_ds_takes(c->frontend.direct_sampling, &p->stream, c->frequency);
    pl->siq = pl->ds ? MIRISDR_IQ_ONLY_I : pl->iq;
    pl->sif = pl->ds ? c->frequency : mirisdr_if_hz[pl->ifm];
    pl->ds_edge = pl->ds ? mirisdr_ds_edge(&p->stream, c->frequency) : 0;

    /* two tuners streamed: each converter takes one tuner's output, which one is not known,
       so both run */
    if ((mirisdr_stream_tuners(p) == 3) && (pl->siq != MIRISDR_IQ_BOTH))
    {
        mirisdr_refuse(p, "two tuners streamed: a tune with one output could cut its converter off\n");
        return -1;
    }

    /* the stream capturing the output switched off would go dead, unless it follows (as
       a baseband one does). A tuner the stream does not take is free */
    if (!adjust && (mirisdr_stream_tuners(p) & (1 << p->tuner_cur)) && !p->stream.follow_tune &&
        !p->stream.baseband && (pl->siq != MIRISDR_IQ_BOTH) &&
        (mirisdr_stream_adc(p) != MIRISDR_IQ_BOTH) && (mirisdr_stream_adc(p) != pl->siq))
    {
        mirisdr_refuse(p, "the stream captures the tuner output this tune switches off\n");
        return -1;
    }

    if (c->synth_thresh > 4095)
    {
        if (c->strict)
        {
            mirisdr_refuse(p, "unsupported synthesizer threshold: %u\n", c->synth_thresh);
            return -1;
        }
        pl->cfg.synth_thresh = 4095;
    }

    if ((c->dc_removal < MIRISDR_DC_AUTO) || (c->dc_removal > MIRISDR_DC_OFF))
    {
        mirisdr_refuse(p, "unsupported DC removal: %d\n", c->dc_removal);
        return -1;
    }

    if (mirisdr_override_words(&c->override, &pl->ovr13, &pl->ovr14) < 0)
    {
        mirisdr_refuse(p, "tuner override code out of range\n");
        return -1;
    }

    if ((c->frontend.notch & ~(MIRISDR_NOTCH_FM | MIRISDR_NOTCH_DAB | MIRISDR_NOTCH_MW)) ||
        (c->frontend.notch && !MIRISDR_HW_EXPANDER(p->hw_flavour)))
    {
        mirisdr_refuse(p, "unsupported notch 0x%x: the notches are on the RSP1B, RSPdx-R2 and RSPduo only\n",
                       c->frontend.notch);
        return -1;
    }

    if (c->frontend.bias && (p->hw_flavour == MIRISDR_HW_RSPDUO) && !p->tuner_cur)
    {
        mirisdr_refuse(p, "unsupported bias-T on tuner 0: the RSPduo's powers the port marked tuner 2, tuner 1\n");
        return -1;
    }

    if (c->frontend.flags & ~mirisdr_frontend_flags(p->hw_flavour))
    {
        mirisdr_refuse(p, "unsupported front end flags 0x%x (this board has 0x%x)\n",
                       c->frontend.flags, mirisdr_frontend_flags(p->hw_flavour));
        return -1;
    }

    if ((c->frontend.raw.fields & ~mirisdr_frontend_parts(p->hw_flavour)) ||
        ((c->frontend.raw.gpio_dir | c->frontend.raw.gpio_level) & ~0x0F))
    {
        mirisdr_refuse(p, "unsupported raw front end: parts 0x%x (this board has 0x%x), GPIO 0x%x 0x%x\n",
                       c->frontend.raw.fields, mirisdr_frontend_parts(p->hw_flavour),
                       c->frontend.raw.gpio_dir, c->frontend.raw.gpio_level);
        return -1;
    }

    hiz = (p->hw_flavour == MIRISDR_HW_RSPDUO) && !p->tuner_cur && (c->frontend.antenna == MIRISDR_ANTENNA_HIZ);
    if ((c->frontend.antenna < MIRISDR_ANTENNA_A) || (c->frontend.antenna > MIRISDR_ANTENNA_C) ||
        ((c->frontend.antenna != MIRISDR_ANTENNA_A) && (p->hw_flavour != MIRISDR_HW_RSPDXR2) && !hiz))
    {
        mirisdr_refuse(p, "unsupported antenna %d: B and C are on the RSPdx-R2 only, and B (its Hi-Z) on the "
                       "RSPduo's tuner 0\n", c->frontend.antenna);
        return -1;
    }

    if ((c->gain.mode < MIRISDR_GAIN_KEEP) || (c->gain.mode > MIRISDR_GAIN_STAGES) ||
        (c->strict && (c->gain.mode == MIRISDR_GAIN_TOTAL) && (c->gain.total < 0)))
    {
        mirisdr_refuse(p, "unsupported gain: mode %d, %d dB\n", c->gain.mode, c->gain.total);
        return -1;
    }

    lo = (int64_t) c->frequency;
    /* a baseband stream wants the band the IF below the LO, as low_if_auto */
    if (c->low_if_auto || (p->stream.baseband && (pl->ifm != MIRISDR_IF_ZERO))) lo += mirisdr_if_hz[pl->ifm];

    if (hiz && (lo >= 1000000LL * DUO_HIZ_END_MHZ))
    {
        mirisdr_refuse(p, "the Hi-Z input ends at %d MHz, the LO would be at %lld Hz\n", DUO_HIZ_END_MHZ, (long long) lo);
        return -1;
    }

    /* the plan of the input asked for */
    ant = p->antenna;
    p->antenna = c->frontend.antenna;

    if ((lo < 0) || (lo > (int64_t) UINT32_MAX) || (!adjust && (lo >= mirisdr_plan_end(p))))
    {
        p->antenna = ant;
        mirisdr_refuse(p, "cannot tune the LO to %lld Hz\n", (long long) lo);
        return -1;
    }

    pl->lo = (uint32_t) lo;

    mirisdr_tune_words(p, pl->lo, c->frequency, pl->ifm, pl->bw, pl->iq, 0, pl->cfg.synth_thresh, &pl->w);
    p->antenna = ant;

    /* past the plan the single setters go on, as far as the synthesizer can count */
    if (!pl->w.fits)
    {
        mirisdr_refuse(p, "cannot tune the LO to %lld Hz\n", (long long) lo);
        return -1;
    }

    /* direct sampling: the low pass, the tuner tuned as usual but not heard */
    if (pl->ds) pl->w.expander = (pl->ds_edge == MIRISDR_DS_LP500_HZ) ? DXR2_LPF500 : DXR2_LPF2;

    /* under direct sampling the gain is its amplifier's, on from half its gain, and the
       tuner takes the rest (no LNA on its AM inputs) */
    g = c->gain;
    pl->ds_lna = p->ds_lna;
    if (pl->ds && (g.mode == MIRISDR_GAIN_STAGES))
    {
        pl->ds_lna = g.lna ? 1 : 0;
        g.lna = 0;
    }
    else if (pl->ds && (g.mode == MIRISDR_GAIN_TOTAL)) pl->ds_lna = g.total >= (MIRISDR_DS_LNA_DB + 1) / 2;

    /* the gain in that band, the board's amplifier on this path */
    mirisdr_gr_get(p, &pl->gr);
    pl->gr.amp_db = pl->ds ? 0 : pl->w.row->amp_db;
    if (g.mode == MIRISDR_GAIN_STAGES)
    {
        if (mirisdr_gr_of_stages(pl->w.band, &g, &pl->gr, c->strict) < 0)
        {
            mirisdr_refuse(p, "unsupported gain stages for the band: lna %d mixer %d mixbuffer %d baseband %d"
                    " amp_bypass %d\n", c->gain.lna, c->gain.mixer, c->gain.mixbuffer, c->gain.baseband,
                    c->gain.amp_bypass);
            return -1;
        }
    }
    else if (c->gain.mode == MIRISDR_GAIN_TOTAL)
    {
        if (c->strict && !pl->ds && (c->gain.total > mirisdr_max_gain_of(pl->w.band) + pl->gr.amp_db))
        {
            mirisdr_refuse(p, "unsupported gain %d dB: the band reaches %d\n", c->gain.total,
                           mirisdr_max_gain_of(pl->w.band) + pl->gr.amp_db);
            return -1;
        }
        mirisdr_split_of(pl->w.band, c->gain.total, &pl->gr);
    }
    /* the stages kept: the mixbuffer by its dB between the two AM inputs */
    else if (p->gain_stages_set) pl->gr.mixbuffer = mirisdr_mixbuffer_keep(p->band, pl->w.band, pl->gr.mixbuffer);
    else mirisdr_split_of(pl->w.band, p->gain, &pl->gr);

    /* the pads, the Hi-Z has none */
    pl->att = mirisdr_attenuator_snap(p->hw_flavour, c->frontend.attenuator, c->strict);
    if (pl->att < 0)
    {
        mirisdr_refuse(p, "unsupported attenuator %d dB: the RSPduo and RSP1B take 0, 6, 13 or 19, the RSPdx-R2 0 to 36 in"
                       " 3 dB steps, other boards none\n", c->frontend.attenuator);
        return -1;
    }
    if (hiz && pl->att)
    {
        if (c->strict)
        {
            mirisdr_refuse(p, "unsupported attenuator %d dB on the Hi-Z input, which has no pads\n", c->frontend.attenuator);
            return -1;
        }
        pl->att = 0;
    }

    mirisdr_frontend_words(p, c, pl);

    return 0;
}

/* The notches a request turns on: the FM and MW notches share a switch */
static int mirisdr_notch_in_force (int notch)
{
    if (notch & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) notch |= MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW;
    return notch;
}

/* Where the received frequency lands, given what the stream captures */
static void mirisdr_tune_result_of (int adc, uint32_t rx, uint32_t lo, int iq, uint32_t bw,
                                    mirisdr_band_t band, const mirisdr_gr_t *gr, mirisdr_tune_result_t *res)
{
    int64_t off = (int64_t) rx - lo;
    int real = (iq != MIRISDR_IQ_BOTH) || (adc != MIRISDR_IQ_BOTH);

    memset(res, 0, sizeof *res);
    res->lo = lo;
    res->iq = iq;
    res->bandwidth = bw;
    res->band = band;
    mirisdr_stages_of(band, gr, &res->gain);

    /* a real signal shows only the size of the offset, mirrored when below the LO */
    res->offset = (int32_t) (real && off < 0 ? -off : off);
    res->inverted = real && off < 0;
}

/* Baseband: where frequency lands is what the callback gets, the band shifted to 0 */
static void mirisdr_tune_result_bb (int bb_path, uint32_t if_hz, uint32_t rx, uint32_t lo,
                                    mirisdr_tune_result_t *res)
{
    if (bb_path == MIRISDR_BASEBAND_OFF) return;

    res->offset = (int32_t) ((int64_t) rx - lo + (bb_path >= MIRISDR_BASEBAND_COMPLEX ? if_hz : 0));
    res->inverted = 0;
}

/* Direct sampling in force: no LO, the I converter with the frequency where it is (at
   0 Hz under baseband), the low pass's edge as the bandwidth, the amplifier as the gain */
static void mirisdr_tune_result_ds (int ds, int lna, uint32_t edge, uint32_t rx, int bb_path,
                                    mirisdr_tune_result_t *res)
{
    res->frontend.direct_sampling = ds;
    if (!ds) return;

    res->lo = 0;
    res->iq = MIRISDR_IQ_ONLY_I;
    res->bandwidth = edge;
    res->offset = (bb_path == MIRISDR_BASEBAND_OFF) ? (int32_t) rx : 0;
    res->inverted = 0;
    res->gain.lna = lna;
    res->gain.mixer = 0;
    res->gain.mixbuffer = 0;
    res->gain.baseband = 0;
    res->gain.total = lna ? MIRISDR_DS_LNA_DB : 0;
}

static int mirisdr_tune_adc (mirisdr_dev_t *p, int iq)
{
    return p->stream.follow_tune ? iq : mirisdr_stream_adc(p);
}

/* the baseband path the selected tuner comes out of: none when the stream does not take it */
static int mirisdr_bb_path_of (mirisdr_dev_t *p)
{
    return (mirisdr_stream_tuners(p) & (1 << p->tuner_cur)) ? p->bb_path : MIRISDR_BASEBAND_OFF;
}

static int mirisdr_tune_apply (mirisdr_dev_t *p, const mirisdr_tune_config_t *c, int flags,
                               mirisdr_tune_result_t *res)
{
    mirisdr_tune_plan_t pl;
    mirisdr_stream_plan_t spl;
    int r, restream = 0, streaming = 0, nco = 0;

    if (!p || !c) return -1;
    if (mirisdr_tune_plan(p, c, flags, &pl) < 0) return -1;

    /* a stream following the tune changes kind or converter with it, in one restart; a
       baseband one its rate too, to the nearest the new IF has. Only a tuner it takes */
    if ((p->stream.follow_tune || p->stream.baseband) && (mirisdr_stream_tuners(p) & (1 << p->tuner_cur)))
    {
        if (mirisdr_stream_plan(p, &p->stream, ((flags & MIRISDR_TUNE_ADJUST) || p->stream.baseband) ?
                                MIRISDR_STREAM_ADJUST : 0, pl.siq, pl.sif, pl.ds_edge, &spl) < 0) return -1;

        /* direct sampling moves its IF with every tune: the NCO alone follows it */
        restream = (spl.format != (int) p->format) || (spl.swap != p->swap_iq) || (spl.rate != p->rate) ||
                   (spl.decim != p->decim_on) || (spl.bb_path != p->bb_path) ||
                   (spl.bb_stages != p->bb_stages) || (spl.bb_rate != p->bb_rate) ||
                   (spl.bb_ds != p->bb_ds) || ((spl.bb_if != p->bb_if) && !spl.bb_ds) ||
                   (spl.bb_if2 != p->bb_if2);
        nco = !restream && (spl.bb_if != p->bb_if);
        if (restream && ((streaming = mirisdr_stream_pause(p)) < 0)) return -1;
    }

    mirisdr_filter_learn(p, pl.w.row);
    mirisdr_batch_begin(p);

    /* the holds go out in the same list, and a held LNA code skips its calibration */
    p->tuner_ovr13 = pl.ovr13;
    p->tuner_ovr14 = pl.ovr14;

    /* the expander (SDRplay) and register 8 go out with the tune, these included */
    p->bias = c->frontend.bias ? 1 : 0;
    p->notch = c->frontend.notch;
    p->antenna = c->frontend.antenna;
    p->attenuator = pl.att;
    p->fe_flags = c->frontend.flags;
    p->ds = pl.ds;
    p->ds_lna = pl.ds_lna;
    p->ds_edge = pl.ds_edge;

    r = mirisdr_lna_calibrate(p, pl.w.row, c->frequency, pl.ifm, pl.bw, pl.iq, &pl.fe, pl.fe_raw);
    if (mirisdr_tune_send(p, &pl.w, &pl.fe, pl.fe_raw) < 0) r = -1;
    p->fe = pl.fe;

    p->freq = pl.lo;
    p->if_freq = pl.ifm;
    p->bandwidth = pl.bw;
    p->tune = pl.cfg;
    p->tune.iq = pl.iq;
    p->tune.gain.mode = MIRISDR_GAIN_KEEP;  /* the gain lives on in the stages, which a tune replaces */
    p->tune_lo = pl.w.lo_real;
    p->tune_exp = pl.w.expander;
    p->tune_iq = pl.siq;
    /* the board's flags: the other tuner's tune keeps them too, so its next tune does not undo them */
    if (mirisdr_tuners_of(p) > 1) p->tctx[1 - p->tuner_cur].tune.frontend.flags = p->fe_flags;

    /* the stages the plan worked out, the board's amplifier with them as sent */
    mirisdr_gr_set(p, &pl.gr);
    if (c->gain.mode == MIRISDR_GAIN_TOTAL)
    {
        p->gain = c->gain.total;
        p->gain_stages_set = 0;
    }
    else if (c->gain.mode == MIRISDR_GAIN_STAGES)
    {
        if (p->gain < 0) p->gain = DEFAULT_GAIN;
        p->gain_stages_set = 1;
    }

    mirisdr_gain_retune(p);
    if (mirisdr_set_gain(p) < 0) r = -1;
    if (mirisdr_batch_end(p) < 0) r = -1;

    if (restream)
    {
        mirisdr_stream_regs(p, &spl);
        if (streaming && (mirisdr_stream_resume(p) < 0)) r = -1;
    }
    else if (nco) mirisdr_bb_move(p, spl.bb_if);

    if (res)
    {
        mirisdr_gr_t gr;

        mirisdr_gr_get(p, &gr);
        mirisdr_tune_result_of(mirisdr_stream_adc(p), c->frequency, p->tune_lo, p->tune_iq,
                               mirisdr_bw_hz[pl.bw], p->band, &gr, res);
        mirisdr_tune_result_bb(mirisdr_bb_path_of(p), mirisdr_if_hz[pl.ifm], c->frequency, p->tune_lo, res);
        res->frontend.bias = p->bias;
        res->frontend.notch = mirisdr_notch_in_force(p->notch);
        res->frontend.antenna = p->antenna;
        res->frontend.attenuator = p->attenuator;
        res->frontend.flags = p->fe_flags;
        res->frontend.raw = p->fe;
        mirisdr_tune_result_ds(p->ds, p->ds_lna, p->ds_edge, c->frequency, mirisdr_bb_path_of(p), res);
    }

    /* as the stream's thread and the other tuner's calls find it */
    mirisdr_tuner_save(p);

    return r;
}

/* The tuner side of a fresh device, the null one included */
static void mirisdr_tuner_defaults (mirisdr_dev_t *p)
{
    mirisdr_tune_config_default(&p->tune);
    p->freq = DEFAULT_FREQ;
    p->gain = DEFAULT_GAIN;
    p->band = MIRISDR_BAND_VHF; // matches always the default frequency of 90 MHz

    p->gain_reduction_lna = 0;
    p->gain_reduction_mixer = 0;
    p->gain_reduction_baseband = 43;
    p->gain_reduction_amp = 0;
    p->amp_db = 0;
    p->attenuator = 0;
    p->fe_flags = 0;
    p->if_freq = MIRISDR_IF_ZERO;
    p->bandwidth = mirisdr_bw_auto(MIRISDR_IF_ZERO, DEFAULT_RATE);    /* as the first tune picks it */
    p->xtal = MIRISDR_XTAL_24M;
    p->bias = 0;
    p->tune_exp = 0;
    p->antenna = MIRISDR_ANTENNA_A;
    p->ds = 0;
    p->ds_lna = 1;
    p->ds_edge = 0;
    p->dc_mode = MIRISDR_DC_PERIODIC2;
    p->dc_speedup = 0;
    p->dc_track = 0x1f;
    p->dc_period = 0x800;
    p->filter_cal = -1;
}

/* the stored tune again, for what changes the front end or the stream's view of it */
static int mirisdr_set_soft(mirisdr_dev_t *p)
{
    return mirisdr_tune_apply(p, &p->tune, MIRISDR_TUNE_ADJUST, NULL);
}

void mirisdr_tune_config_default (mirisdr_tune_config_t *cfg)
{
    if (!cfg) return;

    memset(cfg, 0, sizeof *cfg);
    cfg->frequency = DEFAULT_FREQ;
    cfg->iq = MIRISDR_IQ_BOTH;
    cfg->gain.mode = MIRISDR_GAIN_KEEP;
}

/* The tune calls each take a tuner, selected around them, and the single setters act on the
   one left selected, the stream's first */
int mirisdr_tune (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    int r;

    if (!p || (tuner < 0) || (tuner >= mirisdr_tuners_of(p))) return -1;

    mirisdr_tuner_select(p, tuner);
    r = mirisdr_tune_apply(p, cfg, 0, res);
    mirisdr_tuner_select(p, mirisdr_tuner_primary(p));

    return r;
}

static int mirisdr_tune_check_of (mirisdr_dev_t *p, const mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res);

int mirisdr_tune_check (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    int r;

    if (!p || !cfg || (tuner < 0) || (tuner >= mirisdr_tuners_of(p))) return -1;

    mirisdr_tuner_select(p, tuner);
    r = mirisdr_tune_check_of(p, cfg, res);
    mirisdr_tuner_select(p, mirisdr_tuner_primary(p));

    return r;
}

static int mirisdr_tune_check_of (mirisdr_dev_t *p, const mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    mirisdr_tune_plan_t pl;
    int r;

    p->checking = 1;
    r = mirisdr_tune_plan(p, cfg, 0, &pl);

    /* a stream following the tune has to take it too, as mirisdr_tune() asks */
    if ((r == 0) && (p->stream.follow_tune || p->stream.baseband) && (mirisdr_stream_tuners(p) & (1 << p->tuner_cur)))
    {
        mirisdr_stream_plan_t spl;

        r = mirisdr_stream_plan(p, &p->stream, p->stream.baseband ? MIRISDR_STREAM_ADJUST : 0, pl.siq,
                                pl.sif, pl.ds_edge, &spl);
    }
    p->checking = 0;
    if (r < 0) return -1;

    if (res) {
        int bb = (!p->stream.baseband || !(mirisdr_stream_tuners(p) & (1 << p->tuner_cur))) ? MIRISDR_BASEBAND_OFF :
                 (mirisdr_stream_tuners(p) == 3) ? MIRISDR_BASEBAND_DUAL :
                 (pl.ifm == MIRISDR_IF_ZERO && !pl.ds) ? MIRISDR_BASEBAND_ZERO_IF :
                 (pl.siq == MIRISDR_IQ_BOTH) ? MIRISDR_BASEBAND_COMPLEX : MIRISDR_BASEBAND_REAL;

        mirisdr_tune_result_of(mirisdr_tune_adc(p, pl.siq), cfg->frequency, pl.w.lo_real, pl.siq,
                               mirisdr_bw_hz[pl.bw], pl.w.band, &pl.gr, res);
        mirisdr_tune_result_bb(bb, mirisdr_if_hz[pl.ifm], cfg->frequency, pl.w.lo_real, res);
        res->frontend.bias = cfg->frontend.bias ? 1 : 0;
        res->frontend.notch = mirisdr_notch_in_force(cfg->frontend.notch);
        res->frontend.antenna = cfg->frontend.antenna;
        res->frontend.attenuator = pl.att;
        res->frontend.flags = cfg->frontend.flags;
        res->frontend.raw = pl.fe;
        mirisdr_tune_result_ds(pl.ds, pl.ds_lna, pl.ds_edge, cfg->frequency, bb, res);
    }

    return 0;
}

int mirisdr_get_tune (mirisdr_dev_t *p, int tuner, mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    mirisdr_gr_t gr;

    if (!p || (tuner < 0) || (tuner >= mirisdr_tuners_of(p))) return -1;

    mirisdr_tuner_select(p, tuner);
    mirisdr_gr_get(p, &gr);
    if (cfg) *cfg = p->tune;
    if (res) {
        mirisdr_tune_result_of(mirisdr_stream_adc(p), p->tune.frequency, p->tune_lo, p->tune_iq,
                               mirisdr_bw_hz[p->bandwidth], p->band, &gr, res);
        mirisdr_tune_result_bb(mirisdr_bb_path_of(p), mirisdr_if_hz[p->if_freq], p->tune.frequency, p->tune_lo, res);
        res->frontend.bias = p->bias;
        res->frontend.notch = mirisdr_notch_in_force(p->notch);
        res->frontend.antenna = p->antenna;
        res->frontend.attenuator = p->attenuator;
        res->frontend.flags = p->fe_flags;
        res->frontend.raw = p->fe;
        mirisdr_tune_result_ds(p->ds, p->ds_lna, p->ds_edge, p->tune.frequency, mirisdr_bb_path_of(p), res);
    }
    mirisdr_tuner_select(p, mirisdr_tuner_primary(p));

    return 0;
}

/* ------------------------------------------------------------------ */
/* the single setters, on top of it                                    */
/* ------------------------------------------------------------------ */

int mirisdr_set_tuner_override (mirisdr_dev_t *p, int tuner, const mirisdr_tuner_override_t *ov)
{
    mirisdr_tune_config_t c;

    int r;

    if (!p || (tuner < 0) || (tuner >= mirisdr_tuners_of(p)) || p->external_tuner) return -1;

    mirisdr_tuner_select(p, tuner);
    c = p->tune;
    if (ov) c.override = *ov;
    else memset(&c.override, 0, sizeof c.override);

    r = mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
    mirisdr_tuner_select(p, mirisdr_tuner_primary(p));

    return r;
}

int mirisdr_set_center_freq(mirisdr_dev_t *p, uint32_t freq)
{
    mirisdr_tune_config_t c;

    if (!p) return -1;

    /* always the LO, whatever a tune set before */
    c = p->tune;
    c.frequency = freq;
    c.low_if_auto = 0;

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

uint32_t mirisdr_get_center_freq(mirisdr_dev_t *p)
{
    if (!p) return 0;

    return p->freq;
}

int mirisdr_set_if_freq(mirisdr_dev_t *p, uint32_t freq)
{
    mirisdr_tune_config_t c;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_if_freq: %u Hz\n", freq);
#endif
    if (!p) return -1;

    c = p->tune;
    c.if_freq = freq;

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

uint32_t mirisdr_get_if_freq(mirisdr_dev_t *p)
{
    if (!p) return -1;

    return mirisdr_if_hz[p->if_freq];
}

/* not supported yet */
int mirisdr_set_xtal_freq(mirisdr_dev_t *p, uint32_t freq)
{
    (void) p;
    (void) freq;
    return -1;
}

uint32_t mirisdr_get_xtal_freq(mirisdr_dev_t *p)
{
    if (!p)
        goto failed;

    switch (p->xtal)
    {
    case MIRISDR_XTAL_19_2M:
        return 19200000;
    case MIRISDR_XTAL_22M:
        return 22000000;
    case MIRISDR_XTAL_24M:
    case MIRISDR_XTAL_24_576M:
        /* really 24 MHz ??? */
        return 24000000;
    case MIRISDR_XTAL_26M:
        return 26000000;
    case MIRISDR_XTAL_38_4M:
        return 38400000;
    }

    failed: return -1;
}

int mirisdr_set_freq_correction(mirisdr_dev_t *p, int ppm)
{
    (void) p;
    (void) ppm;
    fprintf(stderr, "frequency correction not implemented yet\n");
    return -1;
}

/* librtlsdr's: 1 the I converter, the only one the RSPdx-R2 has it on */
int mirisdr_set_direct_sampling(mirisdr_dev_t *p, int on)
{
    mirisdr_tune_config_t c;

    if (!p) return -1;
    if ((on != MIRISDR_DS_OFF) && (on != MIRISDR_DS_ON) && (on != MIRISDR_DS_AUTO))
    {
        mirisdr_refuse(p, "unsupported direct sampling %d: 0 off, 1 the I converter, %d auto\n", on, MIRISDR_DS_AUTO);
        return -1;
    }

    c = p->tune;
    c.frontend.direct_sampling = on;
    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_get_direct_sampling(mirisdr_dev_t *p)
{
    return p ? p->tune.frontend.direct_sampling : -1;
}

/* As before: the nearest filter at or above the request, and a narrow one moves the IF */
int mirisdr_set_bandwidth(mirisdr_dev_t *p, uint32_t bw)
{
    mirisdr_tune_config_t c;
    int b = MIRISDR_BW_MAX;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_bandwidth: %u Hz\n", bw);
#endif
    if (!p) return -1;

    c = p->tune;

    if (bw <= 8000000) b = MIRISDR_BW_8MHZ;
    if (bw <= 7000000) b = MIRISDR_BW_7MHZ;
    if (bw <= 6000000) b = MIRISDR_BW_6MHZ;
    if (bw <= 5000000) b = MIRISDR_BW_5MHZ;
    if (bw <= 1536000)
    {
        b = MIRISDR_BW_1536KHZ;
        if (p->rate >= 5000000) c.if_freq = 1620000;
    }
    if (bw <= 600000) { b = MIRISDR_BW_600KHZ; c.if_freq = 450000; }
    if (bw <= 300000) b = MIRISDR_BW_300KHZ;
    if (bw <= 200000) b = MIRISDR_BW_200KHZ;

    c.bandwidth = mirisdr_bw_hz[b];

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

uint32_t mirisdr_get_bandwidth(mirisdr_dev_t *p)
{
    if (!p) return -1;

    return mirisdr_bw_hz[p->bandwidth];
}

int mirisdr_set_offset_tuning(mirisdr_dev_t *p, int on)
{
    mirisdr_tune_config_t c;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_offset_tuning: %d\n", on);
#endif
    if (!p) return -1;

    c = p->tune;
    c.if_freq = on ? 450000 : 0;

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_set_transfer(mirisdr_dev_t *p, const char *v)
{
    mirisdr_stream_config_t c;

    if (!p || !v) return -1;

    c = p->stream;
    c.transfer = v;

    /* the endpoint burst size follows it */
    return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

const char *mirisdr_get_transfer(mirisdr_dev_t *p)
{
    if (!p) return "";

    switch (p->transfer)
    {
    case MIRISDR_TRANSFER_BULK:
        return "BULK";
    case MIRISDR_TRANSFER_ISOC:
        switch (p->alt_setting)
        {
        case 2:  return "ISOC1";
        case 4:  return "ISOC2";
        default: return "ISOC";
        }
    }

    return "";
}

mirisdr_band_t mirisdr_get_band (mirisdr_dev_t *p)
{
    if (!p) return MIRISDR_BAND_AM1;

    return p->band;
}

/* Under direct sampling the gain is the low pass's amplifier, and the RSPduo's amplifier
   is switched, by expander pins the gain words do not reach: the tune in force again, with
   the gain the setter left */
static int mirisdr_gain_tune(mirisdr_dev_t *p)
{
    mirisdr_tune_config_t c = p->tune;
    mirisdr_gr_t r;

    if (p->gain_stages_set)
    {
        mirisdr_gr_get(p, &r);
        mirisdr_stages_of(p->band, &r, &c.gain);
    }
    else
    {
        c.gain.mode = MIRISDR_GAIN_TOTAL;
        c.gain.total = p->gain;
    }

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_set_attenuator (mirisdr_dev_t *p, int db)
{
    mirisdr_tune_config_t c;

    if (!p) return -1;
    c = p->tune;
    c.frontend.attenuator = db;
    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_get_attenuator (mirisdr_dev_t *p)
{
    return p ? p->attenuator : -1;
}

int mirisdr_get_frontend_caps (mirisdr_dev_t *p, int tuner, mirisdr_frontend_caps_t *caps)
{
    const hw_switch_freq_plan_t *plan;
    int f, i;

    if (!p || !caps || (tuner < 0) || (tuner >= mirisdr_tuners_of(p))) return -1;

    f = p->hw_flavour;
    memset(caps, 0, sizeof *caps);
    caps->notches = MIRISDR_HW_EXPANDER(f) ? (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_DAB | MIRISDR_NOTCH_MW) : 0;
    caps->antennas = 1u << MIRISDR_ANTENNA_A;
    caps->antenna_hiz = -1;
    if (f == MIRISDR_HW_RSPDXR2) caps->antennas |= (1u << MIRISDR_ANTENNA_B) | (1u << MIRISDR_ANTENNA_C);
    if ((f == MIRISDR_HW_RSPDUO) && !tuner)
    {
        caps->antennas |= 1u << MIRISDR_ANTENNA_HIZ;
        caps->antenna_hiz = MIRISDR_ANTENNA_HIZ;
    }
    /* the RSPduo's bias-T powers tuner 1's port. Other boards take frontend.bias as
       libmirisdr did (GPIO_3), but no bias-T is known there */
    caps->bias = MIRISDR_HW_EXPANDER(f) && ((f != MIRISDR_HW_RSPDUO) || tuner);
    caps->direct_sampling = f == MIRISDR_HW_RSPDXR2;

    /* the amplifier: the 50 ohm plan's rows that have one */
    plan = hw_switch_freq_plan[(f >= MIRISDR_HW_DEFAULT && f <= MIRISDR_HW_RSPDUO) ? f : MIRISDR_HW_DEFAULT];
    for (i = 0; plan[i].mode >= 0; i++)
    {
        if (!plan[i].amp_db) continue;
        if (!caps->amp_bypass_hz) caps->amp_bypass_hz = plan[i].low_khz * 1000u;
        if (plan[i].amp_db > caps->amp_max_db) caps->amp_max_db = plan[i].amp_db;
    }

    caps->attenuator_count = mirisdr_attenuator_list(f, caps->attenuator);
    caps->flags = mirisdr_frontend_flags(f);
    caps->raw_parts = mirisdr_frontend_parts(f);

    return 0;
}

int mirisdr_set_frontend_flags (mirisdr_dev_t *p, uint32_t flags)
{
    mirisdr_tune_config_t c;

    if (!p) return -1;
    c = p->tune;
    c.frontend.flags = flags;
    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_get_frontend_flags (mirisdr_dev_t *p)
{
    return p ? (int) p->fe_flags : -1;
}

/* The tune in force with its frontend changed */
int mirisdr_set_bias (mirisdr_dev_t *p, int bias)
{
	mirisdr_tune_config_t c;

	if (!p) return -1;
	c = p->tune;
	c.frontend.bias = bias ? 1 : 0;
	return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_set_notch (mirisdr_dev_t *p, int notches)
{
	mirisdr_tune_config_t c;

	if (!p) return -1;
	c = p->tune;
	c.frontend.notch = notches;
	return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_get_notch (mirisdr_dev_t *p)
{
	return p ? p->notch : -1;
}

int mirisdr_set_antenna (mirisdr_dev_t *p, int antenna)
{
	mirisdr_tune_config_t c;

	if (!p) return -1;
	c = p->tune;
	c.frontend.antenna = antenna;
	return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_get_antenna (mirisdr_dev_t *p)
{
	return p ? p->antenna : -1;
}

int mirisdr_get_bias (mirisdr_dev_t *p)
{
	return p ? p->bias : -1;
}
