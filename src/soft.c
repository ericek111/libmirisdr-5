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

//float band_limits[] = {
//        0.,     12.,    30.,    50.,    108.,   250.,   390.,   960.,   2400,   -1.
//};
//
//uint32_t band_select[] = {
//        0xf780, 0xff80, 0xf280, 0xf380, 0xfa80, 0xf680, 0xf380, 0xfa80, 0x0000, 0x0000
//};
//GPIO0 - DAB notch
//GPIO2 - Broadcast FM notch

/*
 * The last column is the word written to MSi2500 register 0x08. It is not an
 * opaque band pattern; it decomposes as:
 *
 *      bits 15:12   GPIO output enable, one bit per pin (GPIO_0 .. GPIO_3)
 *      bits 11:8    GPIO output data,   one bit per pin (GPIO_0 .. GPIO_3)
 *      bit  7       lock LED
 *
 * The MSi2500 has exactly four GPIO pins, so there is no GPIO_4..GPIO_7; the
 * upper nibble is direction, not more data. Every word in both plans below uses
 * enable nibble 0xf, i.e. all four pins driven as outputs, which is only safe on
 * a board that puts nothing else on them. The MSi2500 datasheet's reference
 * application uses GPIO_0 for EEPROM read-data, GPIO_1 for the EEPROM chip
 * select, GPIO_2 as a memory-size strap and GPIO_3 as an IR receiver input, so a
 * board with serial memory fitted needs GPIO_0 left as an input -- enable nibble
 * 0xe -- or it will fight the EEPROM driving that line. The SDRplay service does
 * exactly that for every hardware version except the RSP1, which has no EEPROM.
 *
 * Known data-bit assignments on these boards: GPIO_0 is a DSB/DAB notch, GPIO_2
 * a broadcast FM notch, GPIO_3 the bias-T.
 *
 * Trailing 0x80 in every word is the lock LED; Osmocom's driver clears it on
 * shutdown by writing the same word with bit 7 low.
 */
hw_switch_freq_plan_t hw_switch_freq_plan_default[] = {
        {0,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf780},
        {12,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xff80},
        {30,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf280},
        {50,   MIRISDR_MODE_VHF, 0, 0, 32, 0xf380},
        {108,  MIRISDR_MODE_B3,  0, 0, 16, 0xfa80},
        {250,  MIRISDR_MODE_B3,  0, 0, 16, 0xf680},
        {259,  6              ,  0, 0, 8,  0xf680},
        {330,  MIRISDR_MODE_B45, 0, 0, 4,  0xf380},
        {960,  MIRISDR_MODE_BL,  0, 0, 2,  0xfa80},
        {2400, -1, 0, 0, 0, 0x0000},
};

hw_switch_freq_plan_t hw_switch_freq_plan_sdrplay[] = {
        {0,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580},
        {12,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580},
        {30,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580},
        {50,   MIRISDR_MODE_VHF, 0, 0, 32, 0xf180},
        {112,  MIRISDR_MODE_B3,  0, 0, 16, 0xf580},
        {250,  MIRISDR_MODE_B3,  0, 0, 16, 0xf480},
        {261,  6              ,  0, 0, 8,  0xf480},
        {404,  MIRISDR_MODE_B45, 0, 0, 4,  0xf580},
        {1000, MIRISDR_MODE_BL,  0, 0, 2,  0xf580},
        {2400, -1, 0, 0, 0, 0x0000},
};

hw_switch_freq_plan_t *hw_switch_freq_plan[2] = {
        hw_switch_freq_plan_default,
        hw_switch_freq_plan_sdrplay
};

#define BIAS_GPIO 3

void update_reg_8(mirisdr_dev_t *p)
{
    mirisdr_write_reg(p, 0x08, p->reg8|(p->bias?(1<<(BIAS_GPIO+8)):0));
}

int mirisdr_set_soft(mirisdr_dev_t *p)
{
    uint32_t reg0 = 0, reg2 = 2, reg5 = 5, reg3 = 3, regd = 0x0d;
    uint64_t n, thresh, frac, lo_div = 0, fvco = 0, rfvco = 0, offset = 0, afc = 0, a, b, c;
    uint64_t lo_freq;
    int i;

    /*
     * Everything below tunes the local oscillator, and on a harmonic that is
     * not the frequency being received. A switching mixer answers at odd
     * multiples of its LO, so putting the LO at f/n receives at f with the
     * n-th harmonic doing the downconversion; n == 1 is the ordinary case and
     * divides by one.
     *
     * The division has to happen here, before anything else, because the band
     * plan and the synthesiser both describe the oscillator rather than the
     * signal. That has a consequence worth knowing: the plan row that supplies
     * the right divider also selects an RF input path, and for a harmonic tune
     * that path is the one around f/n, not the one around f. The signal has to
     * reach the mixer through a front end chosen for a fifth of its frequency.
     */
    lo_freq = p->freq / (p->harmonic ? p->harmonic : 1);

    /*** registr0 - parametry pásma ***/
    /*** registr0 - parameters zone ***/

    /* pásmo */
    /* zone */

    i = 0;

    while (lo_freq >= 1000000 * hw_switch_freq_plan[(int) p->hw_flavour][i].low_cut)
    {
        if (hw_switch_freq_plan[(int) p->hw_flavour][i].mode < 0) {
            break;
        }

        i++;
    }

    hw_switch_freq_plan_t switch_plan = hw_switch_freq_plan[(int) p->hw_flavour][i-1];

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_soft: i:%d flavour:%d flow:%u mode:%d up:%d port:%d lo:%d\n",
            i-1,
            (int) p->hw_flavour,
            switch_plan.low_cut,
            switch_plan.mode,
            switch_plan.upconvert_mixer_on,
            switch_plan.am_port,
            switch_plan.lo_div);
#endif

    if (switch_plan.mode == MIRISDR_MODE_AM)
    {
        reg0 |= MIRISDR_MODE_AM << 4;
        reg0 |= switch_plan.upconvert_mixer_on << 9;
        reg0 |= switch_plan.am_port << 11;

        if (switch_plan.upconvert_mixer_on)
        {
            /* the AM path is up-converted to the first IF, 5 * F_REF1 */
            offset += MIRISDR_FIF1_HZ;
        }

        lo_div = 16;

        if (switch_plan.am_port == 0) {
            p->band = MIRISDR_BAND_AM1;
        } else {
            p->band = MIRISDR_BAND_AM2;
        }
    }
    else
    {
        reg0 |= switch_plan.mode << 4;
        lo_div = switch_plan.lo_div;

        if (switch_plan.mode == MIRISDR_MODE_VHF) {
            p->band = MIRISDR_BAND_VHF;
        } else if (switch_plan.mode == MIRISDR_MODE_B3) {
            p->band = MIRISDR_BAND_3;
        } else if (switch_plan.mode == MIRISDR_MODE_B45) {
            p->band = MIRISDR_BAND_45;
        } else if (switch_plan.mode == MIRISDR_MODE_BL) {
            p->band = MIRISDR_BAND_L;
        }
    }

//    if (p->freq < 50000000)
//    {
//        /* AM režim2, antena 2, AM režim1 - 0x61 */
//        /* AM mode2, Antena 2, AM mode1 - 0x61   */
//        reg0 |= MIRISDR_MODE_AM << 4;
//        reg0 |= MIRISDR_UPCONVERT_MIXER_ON << 9;
//        reg0 |= MIRISDR_AM_PORT2 << 11;
//        /* AM režim je posunutý o 5 * referenční frekvence, tj. o 120 MHz */
//        /* AM mode is shifted about 5 * reference frequency, i.e. 120 MHz */
//        lo_div = 16;
//        offset += 120000000UL;
//    }
//    else if (p->freq < 108000000)
//    {
//        /* VHF */
//        reg0 |= MIRISDR_MODE_VHF << 4;
//        lo_div = 32;
//    }
//    else if (p->freq < 330000000)
//    {
//        /* B3 */
//        reg0 |= MIRISDR_MODE_B3 << 4;
//        lo_div = 16;
//    }
//    else if (p->freq < 960000000)
//    {
//        /* B45 */
//        reg0 |= MIRISDR_MODE_B45 << 4;
//        lo_div = 4;
//    }
//    else
//    {
//        /* BL */
//        reg0 |= MIRISDR_MODE_BL << 4;
//        lo_div = 2;
//    }

    /*
     * Datasheet 5.5: F_LO = F_C + F_IF, and F_C + F_IF + F_IF1 for the AM
     * modes. The IF term was missing altogether -- selecting a low IF moved
     * the channel filter but left the oscillator where it was, so the wanted
     * signal sat F_IF away from where the filter was looking. Only the zero-IF
     * path was unaffected, which is the one nearly everything uses, so it went
     * unnoticed.
     */
    offset += mirisdr_get_if_freq(p);

    /* RF syntetizer je vždy aktivní */
    /* RF synthesizer is always active */
    reg0 |= MIRISDR_RF_SYNTHESIZER_ON << 10;

    /* režim IF filtru - zatím nefunguje? */
    /* IF filter mode - has not worked? */
    switch (p->if_freq)
    {
    case MIRISDR_IF_ZERO:
        reg0 |= MIRISDR_IF_MODE_ZERO << 12;
        break;
    case MIRISDR_IF_450KHZ:
        reg0 |= MIRISDR_IF_MODE_450KHZ << 12;
        break;
    case MIRISDR_IF_1620KHZ:
        reg0 |= MIRISDR_IF_MODE_1620KHZ << 12;
        break;
    case MIRISDR_IF_2048KHZ:
        reg0 |= MIRISDR_IF_MODE_2048KHZ << 12;
        break;
    }

    /* šířka pásma - 8 MHz, nejvyšší možná */
    /* Bandwidth - 8 MHz, the highest possible */
    switch (p->bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        reg0 |= 0x00 << 14;
        break;
    case MIRISDR_BW_300KHZ:
        reg0 |= 0x01 << 14;
        break;
    case MIRISDR_BW_600KHZ:
        reg0 |= 0x02 << 14;
        break;
    case MIRISDR_BW_1536KHZ:
        reg0 |= 0x03 << 14;
        break;
    case MIRISDR_BW_5MHZ:
        reg0 |= 0x04 << 14;
        break;
    case MIRISDR_BW_6MHZ:
        reg0 |= 0x05 << 14;
        break;
    case MIRISDR_BW_7MHZ:
        reg0 |= 0x06 << 14;
        break;
    case MIRISDR_BW_8MHZ:
        reg0 |= 0x07 << 14;
        break;
    case MIRISDR_BW_MAX:
        reg0 |= 0x07 << 14;
        regd |= (1<<4);
        break;
    }

    /* xtal frekvence - nepodporujeme změnu */
    /* xtal frequency - we do not support change */
    switch (p->xtal)
    {
    case MIRISDR_XTAL_19_2M:
        reg0 |= 0x00 << 17;
        break;
    case MIRISDR_XTAL_22M:
        reg0 |= 0x01 << 17;
        break;
    case MIRISDR_XTAL_24M:
    case MIRISDR_XTAL_24_576M:
        reg0 |= 0x02 << 17;
        break;
    case MIRISDR_XTAL_26M:
        reg0 |= 0x03 << 17;
        break;
    case MIRISDR_XTAL_38_4M:
        reg0 |= 0x04 << 17;
        break;
    }

    /* 4 bity pro režimy snížené spotřeby */
    /* 4 bits for power saving modes */
    reg0 |= MIRISDR_IF_LPMODE_NORMAL << 20;
    reg0 |= MIRISDR_VCO_LPMODE_NORMAL << 23;

    /* vco frekvence, je lepší použít 64bitový rozsah */
    /* VCO frequency is better to use a 64-bit range */
    fvco = (lo_freq + offset) * lo_div;

    /* posun po hlavní frekvenci */
    /* shift the main frequency */
    n = fvco / MIRISDR_FLO_MULT;

    /* hlavní registr, hrubé ladění */
    /* major registry, coarse tuning */
    thresh = MIRISDR_FLO_MULT / lo_div;

    /* vedlejší registr, jemné ladění */
    /* side register, fine tuning */
    frac = (fvco % MIRISDR_FLO_MULT) / lo_div;

    /* najdeme největší společný dělitel pro thresh a frac */
    /* We find the greatest common divisor for thresh and frac */
    for (a = thresh, b = frac; a != 0;)
    {
        c = a;
        a = b % a;
        b = c;
    }

    /* dělíme */
    /* divided */
    thresh /= b;
    frac /= b;

    /* v této části musíme rozlišení snížit na maximální rozsah registru */
    /* In this section we reduce the resolution to the maximum extent registry */
    a = (thresh + 4094) / 4095;
    thresh = (thresh + (a / 2)) / a;
    frac = (frac + (a / 2)) / a;

    rfvco=(MIRISDR_FLO_MULT * (n * thresh * 4096UL + (frac * 4096UL))) / (thresh * 4096UL * lo_div);
    if(lo_freq + offset < rfvco)
        frac --;
    rfvco=(MIRISDR_FLO_MULT * (n * thresh * 4096UL + (frac * 4096UL + afc))) / (thresh * 4096UL * lo_div);
    afc = ((lo_freq + offset - rfvco) * thresh * 4096UL * lo_div) / MIRISDR_FLO_MULT;

    reg3 |= (afc & 4095) << 4;
    reg5 |= (0xFFF & thresh) << 4;
    /* rezervováno, musí být 0x28 */
    /* Reserved, must be 0x28 */
    reg5 |= MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING << 16;

    reg2 |= (0xFFF & frac) << 4;
    reg2 |= (0x3F & n) << 16;
    reg2 |= MIRISDR_LBAND_LNA_CALIBRATION_OFF << 22;

    /* kernel driver nastavuje až při změně frekvence */
    /* kernel driver adjusts to changing frequencies  */

//    i = 0;
//
//    while (p->freq >= 1000000 * band_limits[i + 1]) {
//        i++;
//    }
//
//    if (band_select[i] != 0)
//    {
//        mirisdr_write_reg(p, 0x08, 0xf380);
//        mirisdr_write_reg(p, 0x08, 0x6280);
//        mirisdr_write_reg(p, 0x08, band_select[i]);
//    }

    //mirisdr_write_reg(p, 0x08, switch_plan.band_select_word);
    p->reg8=switch_plan.band_select_word;
    update_reg_8(p);

    mirisdr_write_reg(p, 0x09, 0x0e);
    mirisdr_write_reg(p, 0x09, reg3);

    mirisdr_write_reg(p, 0x09, reg0);
    mirisdr_write_reg(p, 0x09, reg5);
    mirisdr_write_reg(p, 0x09, reg2);
    mirisdr_write_reg(p, 0x09, regd);

//    if (band_select[i] != 0)
//    {
//        mirisdr_write_reg(p, 0x08, 0xf380);
//        mirisdr_write_reg(p, 0x08, 0x6280);
//        mirisdr_write_reg(p, 0x08, band_select[i]);
//        mirisdr_write_reg(p, 0x09, 0x0e);
//        mirisdr_write_reg(p, 0x09, 0x03);
//
//        mirisdr_write_reg(p, 0x09, reg0);
//        mirisdr_write_reg(p, 0x09, reg5);
//        mirisdr_write_reg(p, 0x09, reg2);
//    }

#if MIRISDR_DEBUG >= 1
    fprintf( stderr,"mirisdr sel:%d %x ",i,switch_plan.band_select_word);
    fprintf( stderr,"freq: %.3f MHz (offset: %.3f MHz), n: %lu, fraction: %lu/%lu\n",
            ((double) n + (double) frac / (double) thresh) * (MIRISDR_FLO_MULT / 1.0e6) / (double) lo_div,
            (double) offset / 1.0e6, (long unsigned)n,
            (long unsigned)frac, (long unsigned)thresh);
    char* if_str = "undefined";
    switch (p->if_freq)
    {
    case MIRISDR_IF_ZERO:
        if_str = "0 Hz";
        break;
    case MIRISDR_IF_450KHZ:
        if_str = "450 kHz";
        break;
    case MIRISDR_IF_1620KHZ:
        if_str = "1620 kHz";
        break;
    case MIRISDR_IF_2048KHZ:
        if_str = "2048 kHz";
        break;
    }
    char* bw_str = "undefined";
    switch (p->bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        bw_str = "200 kHz";
        break;
    case MIRISDR_BW_300KHZ:
        bw_str = "300 kHz";
        break;
    case MIRISDR_BW_600KHZ:
        bw_str = "600 kHz";
        break;
    case MIRISDR_BW_1536KHZ:
        bw_str = "1536 kHz";
        break;
    case MIRISDR_BW_5MHZ:
        bw_str = "5 MHz";
        break;
    case MIRISDR_BW_6MHZ:
        bw_str = "6 MHz";
        break;
    case MIRISDR_BW_7MHZ:
        bw_str = "7 MHz";
        break;
    case MIRISDR_BW_8MHZ:
        bw_str = "8 MHz";
        break;
    case MIRISDR_BW_MAX:
        bw_str = "maximal";
        break;
    }
    fprintf(stderr, "mirisdr IF: %s, BW: %s\n", if_str, bw_str);
#endif

    return 0;
}

/*
 * Reaching above the fundamental limit.
 *
 * The VCO locks over roughly 1560 - 4230 MHz and the top band divides it by 2,
 * so the highest ordinary tune is about 2115 MHz. Above that the synthesiser
 * simply cannot get there.
 *
 * A switching mixer is a square wave, so its Fourier series has odd terms only
 * and the mixer answers at odd multiples of its own LO as well as at the LO
 * itself. Put the oscillator at f/n and the n-th harmonic downconverts f.
 * Everything downstream is unchanged, because from the synthesiser's point of
 * view a harmonic tune is an ordinary one at a lower frequency.
 *
 * This is chosen automatically. mirisdr_set_center_freq() takes the frequency
 * you want and the library finds a way to reach it, falling back to a harmonic
 * only when the fundamental will not lock -- so ordinary tuning is untouched
 * and nothing below 2115 MHz behaves differently.
 *
 * Two costs are unavoidable and neither is compensated here. The plan row that
 * supplies the divider also selects an RF input path, and for a harmonic tune
 * that path is the one around f/n, so the signal has to arrive through a front
 * end chosen for a fraction of its frequency. And the mixer still answers at
 * its fundamental and at every other odd multiple, so an external bandpass at
 * the wanted frequency is required rather than optional.
 */

/* the divider the band plan would use for this LO; 0 if it is off the plan */
static uint64_t mirisdr_plan_lo_div(mirisdr_dev_t *p, uint64_t lo, uint64_t *offset)
{
    const hw_switch_freq_plan_t *plan = hw_switch_freq_plan[(int) p->hw_flavour];
    int i = 0;

    *offset = 0;

    if (lo < 1000000ULL * plan[0].low_cut)
        return 0;

    while (lo >= 1000000ULL * plan[i].low_cut) {
        if (plan[i].mode < 0)
            return 0;                   /* past the plan's end marker */
        i++;
    }

    if (plan[i - 1].mode == MIRISDR_MODE_AM) {
        /* the AM rows fix the divider themselves and shift by the upconvert */
        if (plan[i - 1].upconvert_mixer_on)
            *offset = MIRISDR_FIF1_HZ;
        return 16;
    }

    return (uint64_t) plan[i - 1].lo_div;
}

/* can the synthesiser actually produce this LO? */
static int mirisdr_lo_lockable(mirisdr_dev_t *p, uint64_t lo)
{
    uint64_t offset, div = mirisdr_plan_lo_div(p, lo, &offset);
    uint64_t vco;

    if (!div)
        return 0;

    vco = (lo + offset) * div;

    return vco >= MIRISDR_VCO_MIN && vco <= MIRISDR_VCO_MAX;
}

/*
 * The lowest odd harmonic that can reach this frequency, or 1 if none can, so
 * an impossible request still fails as an ordinary tune rather than silently
 * landing somewhere unrelated.
 *
 * Lowest rather than the conventional 5th: harmonic amplitude falls with n, so
 * the 3rd costs the least conversion loss, and the reason the 5th is usually
 * preferred -- it alone preserves sideband sense -- no longer applies now that
 * the bridge corrects the sideband in hardware.
 */
static unsigned int mirisdr_pick_harmonic(mirisdr_dev_t *p, uint64_t freq)
{
    unsigned int n;

    for (n = 1; n <= MIRISDR_HARMONIC_MAX; n += 2)
        if (mirisdr_lo_lockable(p, freq / n))
            return n;

    return 1;
}

/*
 * Force a harmonic, or pass 0 to go back to choosing automatically.
 *
 * Only odd n is accepted: a square-wave mixer has no even-harmonic response to
 * speak of, so offering one would be offering a tune that cannot work.
 */
int mirisdr_set_harmonic(mirisdr_dev_t *p, unsigned int n)
{
    if (!p)
        return -1;

    if (n > 1 && (n & 1) == 0) {
        fprintf(stderr, "mirisdr_set_harmonic: %u is not usable, a switching"
                        " mixer responds on odd harmonics only\n", n);
        return -1;
    }

    p->harmonic_auto = (n == 0);
    if (n)
        p->harmonic = n;

    /* reg 0x07 carries the inversion that cancels this harmonic's own */
    if (mirisdr_set_hard(p) < 0)
        return -1;

    return mirisdr_set_soft(p);
}

/* the harmonic actually in use, resolved */
unsigned int mirisdr_get_harmonic(mirisdr_dev_t *p)
{
    return p ? p->harmonic : 0;
}

int mirisdr_set_center_freq(mirisdr_dev_t *p, uint32_t freq)
{
    int r;

    if (!p)
        return -1;

    p->freq = freq;

    if (p->harmonic_auto) {
        unsigned int n = mirisdr_pick_harmonic(p, freq);

        if (n != p->harmonic) {
            p->harmonic = n;
            /* the sideband correction lives in reg 0x07 and follows n */
            if (mirisdr_set_hard(p) < 0)
                return -1;
        }
    }

    r = mirisdr_set_soft(p);
    r += mirisdr_set_gain(p); // restore gain

    return r;
}

uint32_t mirisdr_get_center_freq(mirisdr_dev_t *p)
{
    return p->freq;
}

/*
 * Deliver a deliberately mirrored spectrum.
 *
 * The MSi2500 can exchange I and Q itself -- register 0x07 bit 9, see
 * mirisdr_reg07() -- so this costs nothing per sample.
 *
 * It also means a harmonic tune no longer hands back a mirrored spectrum. Each
 * harmonic rotates the quadrature LO by n * 90 degrees and the sideband inverts
 * whenever that lands on 270, i.e. n mod 4 == 3; the library now sets the bit
 * on exactly those harmonics to cancel it. So the spectrum arrives the right
 * way round whatever n is, and this call is what it says it is: an explicit
 * request for the mirror image, which on such a harmonic is granted by leaving
 * the hardware bit clear.
 */
int mirisdr_set_spectrum_inversion(mirisdr_dev_t *p, int on)
{
    if (!p)
        return -1;

    p->invert_spectrum = on ? 1 : 0;

    return mirisdr_set_hard(p);
}

/* whether the samples being delivered are mirrored, after any correction */
int mirisdr_get_spectrum_inverted(mirisdr_dev_t *p)
{
    return p ? (p->invert_spectrum != 0) : 0;
}

int mirisdr_set_if_freq(mirisdr_dev_t *p, uint32_t freq)
{
#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_if_freq: %u Hz\n", freq);
#endif
    if (!p)
        goto failed;

    switch (freq)
    {
    case 0:
        p->if_freq = MIRISDR_IF_ZERO;
        break;
    case 450000:
        p->if_freq = MIRISDR_IF_450KHZ;
        break;
    case 1620000:
        p->if_freq = MIRISDR_IF_1620KHZ;
        break;
    case 2048000:
        p->if_freq = MIRISDR_IF_2048KHZ;
        break;
    default:
        fprintf(stderr, "unsupported if frequency: %u Hz\n", freq);
        goto failed;
    }

    int r = mirisdr_set_soft(p);
    r += mirisdr_set_gain(p); // restore gain
    return r;

    failed: return -1;
}

uint32_t mirisdr_get_if_freq(mirisdr_dev_t *p)
{
    if (!p)
        goto failed;

    switch (p->if_freq)
    {
    case MIRISDR_IF_ZERO:
        return 0;
    case MIRISDR_IF_450KHZ:
        return 450000;
    case MIRISDR_IF_1620KHZ:
        return 1620000;
    case MIRISDR_IF_2048KHZ:
        return 2048000;
    }

    failed: return -1;
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
        /* realně 24 MHz ??? */
        return 24000000;
    case MIRISDR_XTAL_26M:
        return 26000000;
    case MIRISDR_XTAL_38_4M:
        return 38400000;
    }

    failed: return -1;
}

int mirisdr_set_bandwidth(mirisdr_dev_t *p, uint32_t bw)
{
#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_bandwidth: %u Hz\n", bw);
#endif
    if (!p)
        return -1;

    p->bandwidth = MIRISDR_BW_MAX;

    if(bw <= 8000000)
        p->bandwidth = MIRISDR_BW_8MHZ;
    if(bw <= 7000000)
        p->bandwidth = MIRISDR_BW_7MHZ;
    if(bw <= 6000000)
        p->bandwidth = MIRISDR_BW_6MHZ;
    if(bw <= 5000000)
        p->bandwidth = MIRISDR_BW_5MHZ;
    if(bw <= 1536000)
    {
        p->bandwidth = MIRISDR_BW_1536KHZ;
        if(p->rate >= 5000000)
            p->if_freq = MIRISDR_IF_1620KHZ;
    }
    if(bw <= 600000)
    {
        p->bandwidth = MIRISDR_BW_600KHZ;
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    if(bw <= 300000)
    {
        p->bandwidth = MIRISDR_BW_300KHZ;
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    if(bw <= 200000)
    {
        p->bandwidth = MIRISDR_BW_200KHZ;
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    int r = mirisdr_set_soft(p);
    r += mirisdr_set_gain(p); // restore gain
    return r;
}

uint32_t mirisdr_get_bandwidth(mirisdr_dev_t *p)
{
    if (!p)
        goto failed;

    switch (p->bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        return 200000;
    case MIRISDR_BW_300KHZ:
        return 300000;
    case MIRISDR_BW_600KHZ:
        return 600000;
    case MIRISDR_BW_1536KHZ:
        return 1536000;
    case MIRISDR_BW_5MHZ:
        return 5000000;
    case MIRISDR_BW_6MHZ:
        return 6000000;
    case MIRISDR_BW_7MHZ:
        return 7000000;
    case MIRISDR_BW_8MHZ:
        return 8000000;
    case MIRISDR_BW_MAX:
        return 14000000;
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

int mirisdr_set_direct_sampling(mirisdr_dev_t *p, int on)
{
    (void) p;
    (void) on;
    fprintf(stderr, "direct sampling not implemented yet\n");
    return -1;
}

int mirisdr_set_offset_tuning(mirisdr_dev_t *p, int on)
{
#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_offset_tuning: %d\n", on);
#endif
    if (!p)
        goto failed;

    if (on)
    {
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    else
    {
        p->if_freq = MIRISDR_IF_ZERO;
    }

    return mirisdr_set_soft(p);

    failed: return -1;
}

int mirisdr_set_transfer(mirisdr_dev_t *p, const char *v)
{
    if (!p)
        goto failed;

    if (!strcmp(v, "BULK"))
    {
        p->transfer = MIRISDR_TRANSFER_BULK;
    }
    else if (!strcmp(v, "ISOC"))
    {
        p->transfer = MIRISDR_TRANSFER_ISOC;
    }
    else
    {
        fprintf(stderr, "unsupported transfer type: %s\n", v);
        goto failed;
    }

    return 0;

    failed: return -1;
}

const char *mirisdr_get_transfer(mirisdr_dev_t *p)
{
    switch (p->transfer)
    {
    case MIRISDR_TRANSFER_BULK:
        return "BULK";
    case MIRISDR_TRANSFER_ISOC:
        return "ISOC";
    }

    return "";
}

mirisdr_band_t mirisdr_get_band (mirisdr_dev_t *p)
{
    return p->band;
}

int mirisdr_set_bias (mirisdr_dev_t *p, int bias)
{
	p->bias=bias;
	update_reg_8(p);
	return 0;
}

int mirisdr_get_bias (mirisdr_dev_t *p)
{
	return p->bias;
}
