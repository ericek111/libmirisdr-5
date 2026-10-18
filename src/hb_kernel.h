/*
 * The half-band kernels of the decimators in baseband.c and of the real to I/Q
 * converter (iqconverter_float.c), by where the halving stands. Each is the minimax
 * half-band of its length (a linear program over the taps at odd distances from the
 * centre, which is 0.5): its passband error mirrors its stopband, so a kernel down
 * D dB from fs/2 - p is flat within as much to p. Made for libmirisdr.
 *
 * Only the last halving shapes the output's band: flat to 0.2 of its input rate (0.4
 * of its output) and down from 0.3. One before it need only keep aliases out of that
 * band, at most 0.1 of its rate, so its stopband starts at 0.4; the ones before that,
 * half as wide again, at 0.45.
 */

#ifndef MIRISDR_HB_KERNEL_H
#define MIRISDR_HB_KERNEL_H

/* the last halving: 35 taps, 63.4 dB down from 0.3 fs */
#define MIRISDR_HB_LAST_LEN 35

static const float mirisdr_hb_last[MIRISDR_HB_LAST_LEN] =
{
    1.1562087049e-03f,
    0.0000000000e+00f,
    -2.7484906053e-03f,
    0.0000000000e+00f,
    5.7683078099e-03f,
    0.0000000000e+00f,
    -1.0736466712e-02f,
    0.0000000000e+00f,
    1.8592400972e-02f,
    0.0000000000e+00f,
    -3.1093896603e-02f,
    0.0000000000e+00f,
    5.2603967184e-02f,
    0.0000000000e+00f,
    -9.9130593262e-02f,
    0.0000000000e+00f,
    3.1592691055e-01f,
    5.0000000000e-01f,
    3.1592691055e-01f,
    0.0000000000e+00f,
    -9.9130593262e-02f,
    0.0000000000e+00f,
    5.2603967184e-02f,
    0.0000000000e+00f,
    -3.1093896603e-02f,
    0.0000000000e+00f,
    1.8592400972e-02f,
    0.0000000000e+00f,
    -1.0736466712e-02f,
    0.0000000000e+00f,
    5.7683078099e-03f,
    0.0000000000e+00f,
    -2.7484906053e-03f,
    0.0000000000e+00f,
    1.1562087049e-03f
};

/* the one before it: 11 taps, 68.7 dB down from 0.4 fs */
#define MIRISDR_HB_MID_LEN 11

static const float mirisdr_hb_mid[MIRISDR_HB_MID_LEN] =
{
    9.7119660469e-03f,
    0.0000000000e+00f,
    -5.8120811681e-02f,
    0.0000000000e+00f,
    2.9859337686e-01f,
    5.0000000000e-01f,
    2.9859337686e-01f,
    0.0000000000e+00f,
    -5.8120811681e-02f,
    0.0000000000e+00f,
    9.7119660469e-03f
};

/* the ones before those: 7 taps, 72.5 dB down from 0.45 fs */
#define MIRISDR_HB_EARLY_LEN 7

static const float mirisdr_hb_early[MIRISDR_HB_EARLY_LEN] =
{
    -3.3666563683e-02f,
    0.0000000000e+00f,
    2.8354855446e-01f,
    5.0000000000e-01f,
    2.8354855446e-01f,
    0.0000000000e+00f,
    -3.3666563683e-02f
};

#endif
