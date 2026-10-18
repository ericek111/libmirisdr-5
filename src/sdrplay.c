/*
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
 * SDRplay front ends: the filters, notches and bias-T sit on SPI I/O expanders on the
 * MSi2500's SPI master (0x0B-0x0D), and some models carry two. The RSP1B, RSPdx-R2 and
 * RSPduo are mapped, and the RSP1B and RSPdx-R2 share the expander's wiring and most of its pins.
 *
 * One MCP23S18, chip select GPIO_3, MISO GPIO_0. GPIO_1 is the EEPROM's chip select.
 * The tuner takes words only with GPIO_1 and GPIO_3 high. The master pulls GPIO_1 low
 * itself, which gates the tuner of, and GPIO_3 low gates the EEPROM off, so the
 * expander can be written.
 *
 * The outputs are open drain: a pin is pulled low by clearing its IODIR bit, with
 * OLAT left at 0. IODIR bits, 0 = pulled low. RSP1B measured 2026-10-07 with a signal
 * generator and an FM antenna, the notches and HF 2026-10-08 with tone sweeps:
 *
 *   B7 B6 A7: path, 010 (B6 low) the VHF and B45 inputs, 100 (B7 low) the B3 and L
 *     inputs, each blocking the other's bands by 16-45 dB. 000 passes both
 *     2-13 dB down. 110 the HF bank, 111 the 250-420 MHz bank
 *   B1 B0: filter in the bank. HF, 2 MHz low pass both released, 2-12 both low,
 *     12-30 B0 low. 250-420 bank: 250-300 both released, 300-380 both low,
 *     380-420 B0 low, 30-60 B1 low
 *   A4: broadcast notch, MW up to 44 dB at 1 MHz, FM 37-42 dB at 80-115 MHz
 *   A6: DAB notch, 13-30 dB at 165-235 MHz
 *   A5: gain with a high pass, +13-16 dB at 40 MHz and above, -35..-40 below
 *     3 MHz
 *   A1: bias-T
 *   B2-B4: step attenuator, 6, 12.5 and 18.5 dB measured (6, 13 and 19 as the steps)
 *   A0, A2, A3, B5 are not connected on the board.
 *
 * RSPdx-R2, measured 2026-10-15 with tone sweeps (a MAX2870 above 24 MHz, an Si5351
 * below), the same as the RSP1B except:
 *
 *   B1 B0: HF bank (A7 released), 500 kHz low pass both released, 2 MHz low pass
 *     B1 low, 2-12 both low, 12-30 B0 low. A5 low with 12-30 takes 11-20 dB
 *     more off below 12 MHz and raises the floor 3-5 dB in band. The plan keeps
 *     it released at HF
 *   A3 A2: antenna, A2 low A, A3 low B, both low C. Both released connects A and B
 *     together. The other ports are 26-45 dB down
 *   A1: bias-T, on port B
 *   B2-B5: step attenuator, about 3, 6, 12 and 15 dB
 *   A4: FM notch 51-53 dB at 85-90 MHz, MW notch 19-28 dB at 0.55-1.6 MHz
 *   A6: DAB notch 50-72 dB at 175-210 MHz
 *   A0: direct sampling amplifier, on when released, +37.5 dB flat from 0.1 to 2
 *     MHz, the I converter's floor 4-5 dB up. No effect with GPIO_2 low
 *
 * RSPdx-R2 direct sampling: GPIO_2 high (register 8, the MSi2500's own pin) switches the
 * I converter from the tuner's I output to the antenna through the HF bank, the tuner
 * bypassed (its gain changes nothing there). Q stays on the tuner. It follows the bank
 * code: the 2 MHz low pass flat from 30 kHz to 1.5 MHz, -15 dB at 2.5 MHz, the 500 kHz
 * low pass flat to 500 kHz and ~60 dB down from 700 kHz. Below 30 kHz it rolls off
 * (-8 dB at 10 kHz, -21 at 5 kHz). It follows the antenna bits.
 *
 * RSPduo, 2026-10-17: two MCP23S18, the chip selects decoded from GPIO_3 and GPIO_2 with
 * the master's own: GPIO_3 low the first, GPIO_2 high (GPIO_3 high) the second, both with
 * GPIO_3 low and GPIO_2 high, and GPIO_3 high and GPIO_2 low the EEPROM. Every SPI read needs
 * GPIO_2 low except the second chip's. The library's tuner 0 is the one behind the 50 ohm
 * port marked tuner 1 (and the Hi-Z), its tuner 1 the one behind the port marked tuner 2
 * (with the bias-T): found 2026-10-17 with a generator on each port in turn, the tie on
 * the second chip open. Each port has a bank of filters of its own, the two decoders
 * alike pin for pin. Of the first chip's pins:
 *
 *   A7 B0: which tuner the converters get. Both released I and Q from tuner 0, both
 *     low I and Q from tuner 1, one low a tuner each (low IF from both): B0 low
 *     tuner 0 on I and tuner 1 on Q, A7 low the other way round
 *   B7 B6: which tuner takes the tuner words. Both released tuner 1, B7 low tuner 0,
 *     B6 low both, both low neither. The readback reads the one addressed
 *   B1-B5: tuner 1's filter, a decoder. B4 low with B3 released enables the first
 *     bank, B1 B4 110-240 MHz and L band, B1 B4 B5 60-100 and 420-900 MHz (the
 *     tuner's band picks one), B2 the amplifier's path (18 dB down with its supply off,
 *     chip 2). B3 low the second: B3 B4 B5 30-60,
 *     B1 B3 B4 250-300, B1 B3 B5 300-380, B1 B3 380-420 MHz. B4 B5 low the HF
 *     bank: B1-B5 the 2 MHz low pass, B1 B3 B4 B5 2-12, B2 B3 B4 B5 12-30 (from 5
 *     to 45 MHz, ~10 dB weaker from 15 to 20 MHz), B3 B4 B5 30-60. B3 released a
 *     flat path ~30 dB down
 *   A0-A4: tuner 0's filter, the same decoder, A0..A4 as B1..B5
 *   A5: the reference output, on when low (31 dB or more over the board's own leak
 *     at 24 MHz). No effect on reception (the first map's 20 dB on tuner 1 at
 *     1 MHz does not reproduce)
 *   A6: not connected, it floats (reads high with its pull-up on, keeps it after)
 *
 * Of the second's:
 *
 *   A2: ties the two 50 ohm ports. Low, each bank hears both (5 dB split, 10 dB at
 *     250 MHz). Released, 30-54 dB between them at 1-10 MHz
 *   A4 A6: tuner 0's notches: FM 39 dB at 90 MHz with MW (37 dB at 1 MHz), DAB 31 dB
 *     at 200 MHz
 *   A7 B1: tuner 1's: FM 40 dB at 80 MHz, 61 at 95, with MW 30-55 dB at 0.3-1.2 MHz,
 *     DAB 28-30 dB at 180-200 MHz
 *   B2-B4: tuner 0's attenuator, B5-B7 tuner 1's, three PIN diode pads in parallel,
 *     about 6, 13 and 19 dB, one at a time (two low give less than either sum)
 *   A1: the bias-T, on the port marked tuner 2 (tuner 1's, by meter)
 *   A0 A3: the Hi-Z, A0 low a 2 MHz low pass (-40 dB from 3.5 MHz), A3 low a MW notch
 *     (13-31 dB at 0.28-1.4 MHz). The Hi-Z feeds tuner 0's AM input 1 past the
 *     bank: flat -6..-8 dBFS from 0.1 to 12 MHz, ~8 dB less above
 *   A5 B0: the amplifiers' supplies, tuner 0's and tuner 1's, on when low (MOSFET
 *     switches, traced by the user). Released, the amplifier path loses ~20 dB
 */

#define EXP_WRITE               0x40    /* MCP23S18 opcode, no address pins */
#define EXP_IODIRA              0x00
#define EXP_IODIRB              0x01
#define EXP_IOCON_BANK1         0x05    /* IOCON's address if BANK was set */
#define EXP_OLATA               0x14
#define EXP_OLATB               0x15

/* register 8: GPIO_2 and GPIO_3 as outputs, and their levels */
#define SDRPLAY_G2_OUT          (1u << 14)
#define SDRPLAY_G3_OUT          (1u << 15)
#define SDRPLAY_G2              (1u << 10)
#define SDRPLAY_G3              (1u << 11)

#define SDRPLAY_A6              (1u << 6)
#define SDRPLAY_A5              (1u << 5)
#define DXR2_PAD                10      /* B2-B5: the RSPdx-R2's pads, about 3, 6, 12 and 15 dB */
#define RSP1B_PAD               10      /* B2-B4: the RSP1B's, about 6, 13 and 19 dB, one at a time */

static int mirisdr_duo_pad (int db);
#define SDRPLAY_A4              (1u << 4)
#define SDRPLAY_A3              (1u << 3)
#define SDRPLAY_A2              (1u << 2)
#define SDRPLAY_A1              (1u << 1)
#define SDRPLAY_A0              (1u << 0)

/* The RSPdx-R2's pads for an attenuation, dB: 0 to 36 in 3 dB steps, the pads in series
   (bit k: B2+k low), the combination nearest each step as measured, -1 for any other */
static int mirisdr_dxr2_pads (int db)
{
    static const uint8_t pads[13] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF };

    if ((db < 0) || (db > 36) || (db % 3)) return -1;

    return pads[db / 3];
}

/* The plan's expander word with the notches, bias-T, antenna, direct sampling, amplifier
   bypass and pads asked for: the pins they pull low (the bypass releases A5) */
static uint16_t mirisdr_sdrplay_word (int flavour, uint16_t word, int notch, int bias, int antenna,
                                      int ds, int ds_lna, int amp_off, int att)
{
    uint16_t w = word;

    if (!w || (flavour == MIRISDR_HW_RSPDUO)) return w;

    if (notch & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) w &= (uint16_t) ~SDRPLAY_A4;
    if (notch & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~SDRPLAY_A6;

    if (bias) w &= (uint16_t) ~SDRPLAY_A1;

    if (flavour == MIRISDR_HW_RSPDXR2)
    {
        if (antenna != MIRISDR_ANTENNA_B) w &= (uint16_t) ~SDRPLAY_A2;
        if (antenna != MIRISDR_ANTENNA_A) w &= (uint16_t) ~SDRPLAY_A3;
        if (ds && !ds_lna) w &= (uint16_t) ~SDRPLAY_A0;
        if (amp_off) w |= SDRPLAY_A5;
        if (mirisdr_dxr2_pads(att) > 0) w &= (uint16_t) ~(mirisdr_dxr2_pads(att) << DXR2_PAD);
    }
    if (flavour == MIRISDR_HW_RSP1B)
    {
        if (amp_off) w |= SDRPLAY_A5;
        if (mirisdr_duo_pad(att) > 0) w &= (uint16_t) ~(1u << (RSP1B_PAD + mirisdr_duo_pad(att) - 1));
    }

    return w;
}

/* The RSPduo's second expander: the tie between the ports (MIRISDR_FRONTEND_PORT_TIE), each tuner's
   pad, notches and amplifier supply, tuner 1's bias-T. The FM and MW notches share a switch, as
   mirisdr_notch_in_force() has it. On the Hi-Z the MW notch is its own (the Hi-Z's 2 MHz
   low pass, A0, is left out: it costs 2-4 dB from 1.1 to 2 MHz) */
#define DUO2_BIAS       (1u << 1)       /* A1, tuner 1's port */
#define DUO2_HIZ_MW     (1u << 3)       /* A3, on tuner 0's Hi-Z input */
#define DUO2_TIE        (1u << 2)       /* A2 */
#define DUO2_NOTCH0_FM  (1u << 4)       /* A4 */
#define DUO2_NOTCH0_DAB (1u << 6)       /* A6 */
#define DUO2_NOTCH1_FM  (1u << 7)       /* A7 */
#define DUO2_NOTCH1_DAB (1u << 9)       /* B1 */
#define DUO2_PAD0       10              /* B2-B4: tuner 0's 6, 13 and 19 dB pads */
#define DUO2_PAD1       13              /* B5-B7: tuner 1's */
#define DUO2_AMP0       (1u << 5)       /* A5: tuner 0's amplifier powered, low (a MOSFET switch) */
#define DUO2_AMP1       (1u << 8)       /* B0: tuner 1's */

/* the RSPduo's (and RSP1B's) pad for an attenuation, dB: 0, 6, 13 or 19, -1 for any other */
static int mirisdr_duo_pad (int db)
{
    switch (db)
    {
    case 0:  return 0;
    case 6:  return 1;
    case 13: return 2;
    case 19: return 3;
    default: return -1;
    }
}

static uint16_t mirisdr_duo_word2 (const int notch[2], int bias, int hiz, const int att[2], int tie,
                                   const int amp[2])
{
    uint16_t w = 0xFFFF;
    int k, pad;

    if (tie) w &= (uint16_t) ~DUO2_TIE;
    if (amp[0]) w &= (uint16_t) ~DUO2_AMP0;
    if (amp[1]) w &= (uint16_t) ~DUO2_AMP1;
    if (notch[0] & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) w &= (uint16_t) ~(hiz ? DUO2_HIZ_MW | DUO2_NOTCH0_FM : DUO2_NOTCH0_FM);
    if (notch[0] & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~DUO2_NOTCH0_DAB;
    if (notch[1] & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) w &= (uint16_t) ~DUO2_NOTCH1_FM;
    if (notch[1] & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~DUO2_NOTCH1_DAB;
    if (bias) w &= (uint16_t) ~DUO2_BIAS;
    for (k = 0; k < 2; k++)
    {
        pad = mirisdr_duo_pad(att[k]);
        if (pad > 0) w &= (uint16_t) ~(1u << ((k ? DUO2_PAD1 : DUO2_PAD0) + pad - 1));
    }

    return w;
}

/* The RSPduo's first expander: which tuner the converters get (A7 B0) for the tuners
   streamed (bit n tuner n), which takes the tuner words (B7 B6), each tuner's filter and
   the reference output (A5). The plan rows hold tuner 1's filter (B1-B5), tuner 0's is the
   same code on A0-A4 */
#define DUO1_SW_A7      (1u << 7)
#define DUO1_SW_B0      (1u << 8)
#define DUO1_ROUTE_B6   (1u << 14)
#define DUO1_ROUTE_B7   (1u << 15)
#define DUO1_FILTER0    0x001Fu         /* A0-A4 */
#define DUO1_FILTER1    0x3E00u         /* B1-B5 */
#define DUO1_REF_OUT    (1u << 5)       /* A5: the board's 24 MHz out of the reference connector, low */
#define DUO1_AMP1       (1u << 10)      /* B2, with B1 low from 55 MHz: the amplifier's path instead of
                                           the direct one. The amplifier's supply is on chip 2 */

/* w with tuner n's filter from a plan row's word, the other tuner's left as it is */
static uint16_t mirisdr_duo_filter (uint16_t w, int n, uint16_t row)
{
    if (n) return (uint16_t) ((w & ~DUO1_FILTER1) | (row & DUO1_FILTER1));

    return (uint16_t) ((w & ~DUO1_FILTER0) | ((row >> 9) & DUO1_FILTER0));
}

static uint16_t mirisdr_duo_route (uint16_t w, int tuners, int tuner)
{
    w |= DUO1_SW_A7 | DUO1_SW_B0 | DUO1_ROUTE_B6 | DUO1_ROUTE_B7;

    if (tuners == 2) w &= (uint16_t) ~(DUO1_SW_A7 | DUO1_SW_B0);       /* tuner 1 on both */
    else if (tuners == 3) w &= (uint16_t) ~DUO1_SW_B0;                 /* tuner 0 on I, 1 on Q */
                                                                       /* (1: tuner 0 on both) */
    if (tuner == 0) w &= (uint16_t) ~DUO1_ROUTE_B7;                    /* the words to tuner 0 alone */

    return w;
}

/* The one tuner word w routes the tuner words to, -1 both or none */
static int mirisdr_duo_routed (uint16_t w)
{
    switch (w & (DUO1_ROUTE_B6 | DUO1_ROUTE_B7))
    {
    case DUO1_ROUTE_B6 | DUO1_ROUTE_B7: return 1;
    case DUO1_ROUTE_B6:                 return 0;
    default:                            return -1;
    }
}

/* One write to expander chip (0 the first), register 8 selecting it for the frame and
   then back to idle, which keeps GPIO_3 high. The RSPduo decodes the selects: GPIO_3 low
   is the first, GPIO_2 high the second, so the first needs GPIO_2 low with it */
static int mirisdr_sdrplay_xfer (mirisdr_dev_t *p, uint32_t idle, int chip, uint8_t reg, uint8_t val)
{
    uint32_t r8 = idle;

    if (chip) r8 |= SDRPLAY_G2_OUT | SDRPLAY_G2;
    else
    {
        r8 &= ~SDRPLAY_G3;
        if (p->hw_flavour == MIRISDR_HW_RSPDUO) r8 = (r8 | SDRPLAY_G2_OUT) & ~SDRPLAY_G2;
    }

    if (mirisdr_write_reg(p, 0x08, r8) < 0) return -1;
    if (mirisdr_spi(p, 3, EXP_WRITE, reg, val, 0, NULL) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0B, 0) < 0) return -1;

    return mirisdr_write_reg(p, 0x08, idle);
}

/* Bring expander chip to word w: set up the first time, after that only the bytes that
   changed */
static int mirisdr_sdrplay_send (mirisdr_dev_t *p, int chip, uint16_t w)
{
    uint32_t idle = (p->reg8_valid ? p->reg8_sent : 0xEA80) | SDRPLAY_G3_OUT | SDRPLAY_G3;
    int bit = 1 << chip, first = !(p->exp_valid & bit), stale = p->exp_stale & bit, r = 0;

    if (!first && !stale && (p->exp_iodir[chip] == w)) return 0;

    if (first)
    {
        uint8_t a;

        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_IOCON_BANK1, 0);
        for (a = 0x02; a <= 0x0D; a++) r |= mirisdr_sdrplay_xfer(p, idle, chip, a, 0);
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_OLATA, 0);
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_OLATB, 0);
    }

    if (first || stale || ((p->exp_iodir[chip] ^ w) & 0x00FF))
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_IODIRA, (uint8_t) w);
    if (first || stale || ((p->exp_iodir[chip] ^ w) & 0xFF00))
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_IODIRB, (uint8_t) (w >> 8));

    p->exp_iodir[chip] = w;
    if (r) p->exp_valid &= ~bit;
    else p->exp_valid |= bit;
    p->exp_stale &= ~bit;

    return r;
}
