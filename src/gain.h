
/*** Register 1: Receiver Gain Control ***/

/* reg1: 4:9 (BBGAIN0 - BBGAIN5) */
#define MIRISDR_BASEBAND_GAIN_REDUCTION_MIN             0
#define MIRISDR_BASEBAND_GAIN_REDUCTION_MAX             0x3B

/* reg1: 10:11 (MIXBU0, MIXBU1) - AM port 1 */
#define MIRISDR_AM_PORT1_BLOCKUP_CONVERT_GAIN_REDUCTION_0DB  0
#define MIRISDR_AM_PORT1_BLOCKUP_CONVERT_GAIN_REDUCTION_6DB  1
#define MIRISDR_AM_PORT1_BLOCKUP_CONVERT_GAIN_REDUCTION_12DB 2
#define MIRISDR_AM_PORT1_BLOCKUP_CONVERT_GAIN_REDUCTION_18DB 3

/* reg1: 10:11 (MIXBU0, MIXBU1) - AM port 2 */
#define MIRISDR_AM_PORT2_BLOCKUP_CONVERT_GAIN_REDUCTION_0DB  0
#define MIRISDR_AM_PORT2_BLOCKUP_CONVERT_GAIN_REDUCTION_24DB 3

/* reg1: 12 (MIXL) */
#define MIRISDR_LNA_GAIN_REDUCTION_OFF                  0
#define MIRISDR_LNA_GAIN_REDUCTION_ON                   1

/* reg1: 13 (LNAGR) */
#define MIRISDR_MIXER_GAIN_REDUCTION_OFF                0
#define MIRISDR_MIXER_GAIN_REDUCTION_ON                 1

/* reg1: 14:16 (DCCAL0 - DCCAL2) */
#define MIRISDR_DC_OFFSET_CALIBRATION_STATIC            0
#define MIRISDR_DC_OFFSET_CALIBRATION_PERIODIC1         1
#define MIRISDR_DC_OFFSET_CALIBRATION_PERIODIC2         2
#define MIRISDR_DC_OFFSET_CALIBRATION_PERIODIC3         3
#define MIRISDR_DC_OFFSET_CALIBRATION_ONE_SHOT          4
#define MIRISDR_DC_OFFSET_CALIBRATION_CONTINUOUS        5

/* reg1: 17 (DCCAL_SPEEDUP) */
#define MIRISDR_DC_OFFSET_CALIBRATION_SPEEDUP_OFF       0
#define MIRISDR_DC_OFFSET_CALIBRATION_SPEEDUP_ON        1

/*** Register 6: DC Offset Calibration setup ***/

/*
 * Field positions per the MSi001 datasheet (R3v2.0, table 33), which lists
 * DCTRK_TIM0..DCTRK_TIM5 at bits 4..9 and DCRATE_TIM0..DCRATE_TIM11 at bits
 * 10..21. The track field is six bits, not four: its documented range is 1..63,
 * which does not fit in four. mirisdr_set_gain() already shifts DCRATE_TIM by
 * 10, so only these comments were wrong.
 *
 * Durations are (72 * 3 * n) / crystal seconds, so with a 24 MHz reference one
 * DCTRK_TIM step is 9 us and one DCRATE_TIM step is likewise 9 us.
 *
 * DCRATE_TIM must be greater than DCTRK_TIM: the refresh period has to be
 * longer than the tracking window it contains. Both are settable at runtime
 * through mirisdr_set_dc_raw(), which does not enforce that; these are only
 * the power-on defaults.
 */

/* reg6: 4:9 (DCTRK_TIM0 - DCTRK_TIM5), range 1..63 */
#define MIRISDR_DC_OFFSET_TRACK_TIME_DEFAULT            31
/* The vendor writes 1 here, which is what sdrplay_api reports as
 * DcOffsetTunerT.trackTime; this library has always used 31. */
#define MIRISDR_DC_OFFSET_TRACK_TIME_VENDOR             1

/* reg6: 10:21 (DCRATE_TIM0 - DCRATE_TIM11), range 2..4095 */
#define MIRISDR_DC_OFFSET_REFRESH_TIME_DEFAULT          2048
