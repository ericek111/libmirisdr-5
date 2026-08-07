
/*** Register 0: IC Mode / Power Control ***/

/* reg0: 4:8 (AM_MODE, VHF_MODE, B3_MODE, B45_MODE, BL_MODE) */
#define MIRISDR_MODE_AM                                 0x01
#define MIRISDR_MODE_VHF                                0x02
#define MIRISDR_MODE_B3                                 0x04
#define MIRISDR_MODE_B45                                0x08
#define MIRISDR_MODE_BL                                 0x10

/* reg0: 9 (AM_MODE2) */
#define MIRISDR_UPCONVERT_MIXER_OFF                     0
#define MIRISDR_UPCONVERT_MIXER_ON                      1

/* reg0: 10 (RF_SYNTH) */
#define MIRISDR_RF_SYNTHESIZER_OFF                      0
#define MIRISDR_RF_SYNTHESIZER_ON                       1

/* reg0: 11 (AM_PORT_SEL) */
#define MIRISDR_AM_PORT1                                0
#define MIRISDR_AM_PORT2                                1

/* reg0: 12:13 (FIL_MODE_SEL0, FIL_MODE_SEL1) */
#define MIRISDR_IF_MODE_2048KHZ                         0
#define MIRISDR_IF_MODE_1620KHZ                         1
#define MIRISDR_IF_MODE_450KHZ                          2
#define MIRISDR_IF_MODE_ZERO                            3

/* reg0: 14:16 (FIL_BW_SEL0 - FIL_BW_SEL2) */

/* reg0: 17:19 (XTAL_SEL0 - XTAL_SEL2) */

/* reg0: 20:22 (IF_LPMODE0 - IF_LPMODE2) */
#define MIRISDR_IF_LPMODE_NORMAL                        0
#define MIRISDR_IF_LPMODE_ONLY_Q                        1
#define MIRISDR_IF_LPMODE_ONLY_I                        2
#define MIRISDR_IF_LPMODE_LOW_POWER                     4

/* reg0: 23 (VCO_LPMODE) */
#define MIRISDR_VCO_LPMODE_NORMAL                       0
#define MIRISDR_VCO_LPMODE_LOW_POWER                    1

/*** Register 2: Synthesizer Programming ***/

/* reg2: 4:15 (FRAC0 - FRAC11) */

/* reg2: 16:21 (INT0 - INT5) */

/* reg2: 22 (LNACAL_EN) */
#define MIRISDR_LBAND_LNA_CALIBRATION_OFF               0
#define MIRISDR_LBAND_LNA_CALIBRATION_ON                1

/*** Register 6: RF Synthesizer Configuration ***/

/* reg5: 4:15 (THRESH0 - THRESH11) */

/* reg5: 16:21 (reserved) */
#define MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING     0x28

typedef struct
{
    uint32_t low_cut;
    int mode;
    int upconvert_mixer_on;
    int am_port;
    int lo_div;
    uint32_t band_select_word;
} hw_switch_freq_plan_t;

/*
 * Synthesiser reference, and the quantities the datasheet derives from it
 * (MSi001 R3v2.0 section 5.5 and table 23).
 *
 * F_LO = (4 * F_REF1 / LO_DIV) * [INT + (FRAC * 2^12 + AFC) / (THRESH * 2^12)]
 *
 * so the 4 * F_REF1 below is the constant the synthesiser arithmetic is built
 * on. F_IF1, the first IF the AM path is up-converted to, is specified as an
 * integer multiple of F_REF1: the table gives 122.88 MHz for a 24.576 MHz
 * reference and 130 MHz for 26 MHz, both five times it, and five times this
 * board's 24 MHz is the 120 MHz that has been hardcoded here all along.
 *
 * Table 23 has no 24.000 MHz row -- it predates this hardware -- and the
 * crystal field is sent code 2, which the table calls 24.576. That is not a
 * mismatch. Read the table as ratios rather than frequencies and the field
 * selects a reference divider and an F_IF1 multiplier, nothing absolute:
 *
 *      code   crystal      F_REF1     F_IF1
 *      000    19.2 MHz     /1         x7
 *      001    22           /1         x6
 *      010    24.576       /1         x5
 *      011    26           /1         x5
 *      100    38.4         /2         x7
 *
 * Code 2 therefore means divide by one and multiply by five, which on this
 * board's 24.000 MHz crystal gives F_REF1 = 24.000 and F_IF1 = 120 MHz -- the
 * two constants below. The label in the table is only the crystal that row was
 * written for; the ratios are what the part is being told, and for a 24 MHz
 * part they are the right ones.
 *
 * Measurement agrees: tuning computed from 24.000 lands where it is asked to,
 * where 24.576 would be 2.4% out, which at 2.4 GHz is 60 MHz.
 */
#define MIRISDR_FREF1_HZ                24000000UL
#define MIRISDR_FLO_MULT                (4UL * MIRISDR_FREF1_HZ)    /* 96 MHz */
#define MIRISDR_FIF1_HZ                 (5UL * MIRISDR_FREF1_HZ)    /* 120 MHz */
