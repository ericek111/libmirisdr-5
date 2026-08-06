/*
 * miri_test - transport benchmark for libmirisdr
 *
 * Copyright (C) 2026 by Benjamin Larsson <banan@ludd.ltu.se>
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
 * Sweeps sample rates against both USB transfer modes and reports, for each
 * combination, the rate actually delivered and the samples lost getting there.
 *
 * The point of measuring both together is that throughput on its own does not
 * say whether a mode is keeping up. Isochronous has a reserved slot of
 * 3 x 1024 bytes per microframe -- 24.576 MB/s and not a byte more, however
 * idle the bus is -- and when the device produces more than that the surplus is
 * simply dropped, at no cost to the measured byte rate. Bulk reserves nothing
 * but is not capped either, so on a quiet bus it can carry more. Which of the
 * two wins therefore depends on the sample rate, and the loss column is what
 * distinguishes "fast" from "fast and complete".
 *
 * Samples arrive as 1024 byte blocks holding a 32-bit sequence counter, so a
 * gap is exact rather than inferred; the library accumulates it and this tool
 * reads it back with mirisdr_get_lost_samples().
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#else
#include <windows.h>
#include "getopt/getopt.h"
#endif

#include "mirisdr.h"

#define DEFAULT_FREQUENCY       100000000
#define DEFAULT_BANDWIDTH       8000000
#define DEFAULT_GAIN            40
#define DEFAULT_DURATION        1.0
#define DEFAULT_SETTLE          0.4
#define MAX_RATES               64

/* the range hard.c clamps to */
#define MIRISDR_RATE_MIN        1010000
#define MIRISDR_RATE_MAX        15500000

/*
 * How far the delivered rate may stray from the requested one, either way.
 *
 * Well inside the range the two match to about a tenth of a percent -- 14.0
 * MS/s repeats as 14.001 across runs -- and they only part near the ceiling,
 * reaching about 0.75% at 15.5. So a percent separates ordinary measurement
 * scatter from a real fault, and anything looser lets faults through: at 3% a
 * run delivering 15.16 MS/s against a requested 15.5 was reported as ok.
 *
 * The check has to be two-sided. Coming up short means the transport is not
 * keeping up, which is the common case and the one this tool was written for.
 * But a rate can also come back *high*, and when it does it is the only symptom
 * there is: if the rate PLL's VCO will not lock as low as the requested rate
 * needs, it rails at its floor and the part streams faster than it was asked
 * to, with every loss counter clean. A one-sided check calls that a pass. It
 * did: a board whose VCO floor is near 218 MHz answered a request for 1.010
 * MS/s with 1.139, thirteen percent high, and the row read ok.
 */
#define RATE_TOL 0.01

/* async buffering, matching miri_sdr so the numbers are comparable */
#define ASYNC_BUF_NUMBER        32
#define ASYNC_BUF_LENGTH        (16 * 16384)

static volatile int do_exit = 0;
static mirisdr_dev_t *dev = NULL;

typedef struct {
    mirisdr_dev_t   *dev;
    double          settle;         /* seconds to discard before measuring */
    double          duration;       /* seconds to measure over */
    double          t_first;        /* first callback, start of the settle */
    double          t_begin;        /* start of the measured window */
    double          t_last;         /* most recent callback */
    unsigned long long bytes;       /* bytes in the measured window */
    int             measuring;
} run_t;

static void sighandler(int signum)
{
    (void) signum;
    do_exit = 1;
    if (dev) mirisdr_cancel_async(dev);
}

static double now_seconds(void)
{
#ifndef _WIN32
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double) tv.tv_sec + (double) tv.tv_usec / 1.0e6;
#else
    return (double) GetTickCount64() / 1.0e3;
#endif
}

/*
 * The converters print a line per dropped block. At the loss rates this tool is
 * built to find that is tens of thousands of lines, enough to slow the very
 * pipeline being measured, so stderr is parked on the null device for the
 * duration of a run unless -v asks to see them.
 */
#ifndef _WIN32
static int saved_stderr = -1;

static void stderr_mute(void)
{
    int devnull;

    fflush(stderr);
    if ((saved_stderr = dup(STDERR_FILENO)) < 0) return;
    if ((devnull = open("/dev/null", O_WRONLY)) < 0) {
        close(saved_stderr);
        saved_stderr = -1;
        return;
    }
    dup2(devnull, STDERR_FILENO);
    close(devnull);
}

static void stderr_restore(void)
{
    if (saved_stderr < 0) return;

    fflush(stderr);
    dup2(saved_stderr, STDERR_FILENO);
    close(saved_stderr);
    saved_stderr = -1;
}
#else
static void stderr_mute(void) {}
static void stderr_restore(void) {}
#endif

static void callback(unsigned char *buf, uint32_t len, void *ctx)
{
    run_t *r = ctx;
    double now = now_seconds();

    (void) buf;

    if (r->t_first == 0.0) {
        /* first buffer only says the device came up; the clock starts after
         * the settle window, so USB start-up latency is not charged to the
         * throughput and start-up drops are not charged to the loss count */
        r->t_first = now;
        return;
    }

    if (!r->measuring) {
        if (now - r->t_first < r->settle) return;
        r->measuring = 1;
        r->t_begin = now;
        r->bytes = 0;
        mirisdr_reset_lost(r->dev);
        return;
    }

    r->bytes += len;
    r->t_last = now;

    if (now - r->t_begin >= r->duration) mirisdr_cancel_async(r->dev);
}

/*
 * The five packet formats. `pairs` is the I/Q pairs in each 1024 byte block,
 * which is what the numeric part of the name counts, and `pair_bytes` is what
 * the converter emits per pair -- the two differ because the s16 converters
 * expand: 336_S16 turns 1024 bytes on the wire into 1344 bytes of output.
 *
 * The per-format rate ceilings quoted in hard.c (6.048, 8.064, 9.216, 12.096
 * MS/s) are 24.576 MB/s divided by the block geometry, i.e. properties of the
 * isochronous slot rather than of the silicon. Sweeping the formats against
 * both transports is how you see that.
 */
static const struct {
    const char *name;       /* what mirisdr_set_sample_format() wants */
    const char *alias;      /* miri_sdr style numeric shorthand */
    int         pairs;      /* I/Q pairs per 1024 byte block */
    int         pair_bytes; /* bytes emitted per pair */
    int         bits;       /* sample depth the wire format actually carries */
} formats[] = {
    { "504_S8",  "504", 504, 2,  8 },
    { "504_S16", NULL,  504, 4,  8 },   /* 8-bit samples in 16-bit containers */
    { "384_S16", "384", 384, 4, 10 },
    { "336_S16", "336", 336, 4, 12 },
    { "252_S16", "252", 252, 4, 14 },
};
#define N_FORMATS ((int)(sizeof(formats) / sizeof(formats[0])))

/*
 * What a bare run tests: the two corners of the operating envelope and one
 * ordinary point between them.
 *
 * Sweeping every rate against every format takes minutes and mostly re-measures
 * the uninteresting middle. The corners are where the limits are. Give -r or -m
 * and the full cross product comes back.
 *
 * The largest sample depth cannot be had at the largest rate: a block is always
 * 1024 bytes but holds only 252 pairs at 14 bits against 504 at 8, so 14-bit at
 * 15.5 MS/s would need 63 MB/s against a host bulk limit near 45. Measured, the
 * ceiling rate carries 10-bit and no more, and 14-bit runs to 11 MS/s. So the
 * two extremes are covered where each is reachable -- the largest depth at the
 * midrange point, the largest rate with the deepest sample it can actually
 * take -- rather than pairing them into a combination that can only fail.
 */
static const struct {
    uint32_t    rate;
    const char *fmt;
    int         modes;      /* 1 isoc, 2 bulk, 3 both */
    const char *what;
} default_plan[] = {
    /* both transports should carry these */
    { MIRISDR_RATE_MIN, "504_S8",  3, "floor, smallest sample"   },
    { 2000000,          "252_S16", 3, "midrange, largest sample" },
    { 6000000,          "336_S16", 3, "mid, 12-bit"              },
    /*
     * And these are past what the isochronous slot can carry, so bulk should
     * take them and isochronous should not. 24.576 MB/s over 1024 bytes per
     * block puts the isochronous ceiling at 6.05 MS/s for 14-bit, 9.22 for
     * 10-bit and 12.10 for 8-bit, so each of the three fails differently and
     * a run that shows them all passing is a run to distrust.
     */
    { 9000000,          "252_S16", 1, "past the isoc slot at 14-bit" },
    { MIRISDR_RATE_MAX, "504_S8",  1, "past the isoc slot at 8-bit"  },
    { MIRISDR_RATE_MAX, "384_S16", 3, "ceiling, deepest sample the"
                                      " wire carries there"          },
    /*
     * Bulk at 8-bit gets nowhere near a transport limit at any rate the part
     * will do, so the pairing above says nothing over bulk -- what it measures
     * there is the synthesiser drifting off the requested rate at the ceiling,
     * which the row below it already reports. This one asks a harder question
     * of the transport instead, at a rate the synthesiser holds cleanly.
     */
    { 15000000,         "384_S16", 2, "bulk, 10-bit just under the ceiling" },
    /*
     * And a walk across one of those edges rather than just past it. The 8-bit
     * isochronous slot works out at 12.096 MS/s, so these straddle it and
     * show the transport going from carrying everything to not, which a single
     * point either side cannot. Isochronous only: over bulk all four are a
     * long way inside the envelope and would say nothing.
     */
    { 12000000,         "504_S8",  1, "isoc, just below the 8-bit slot" },
    { 12500000,         "504_S8",  1, "isoc, just above"                },
    /*
     * The same walk for the other transport. Bulk has no reserved slot, so what
     * stops it is whatever the host controller will carry -- about 45 MB/s
     * here, which at 14 bits works out near 11 MS/s. That figure is a property
     * of the machine rather than of the part, so it is walked rather than
     * asserted, and on a different host these three will cross somewhere else.
     */
    { 11000000,         "252_S16", 2, "bulk, close to it"               },
    { 11500000,         "252_S16", 2, "bulk, past it"                   },
};
#define N_PLAN ((int)(sizeof(default_plan) / sizeof(default_plan[0])))

/* accepts either the full name or the numeric shorthand; -1 if unknown */
static int format_lookup(const char *s)
{
    int i;

    for (i = 0; i < N_FORMATS; i++) {
        if (!strcasecmp(s, formats[i].name)) return i;
        if (formats[i].alias && !strcasecmp(s, formats[i].alias)) return i;
    }

    return -1;
}

/*
 * Gain linearity.
 *
 * The transport benchmark asks whether samples arrive; this asks whether the
 * gain they arrive with is the gain that was requested. Those are different
 * questions and only the second one can catch a wrong gain map: an API
 * round-trip test, comparing mirisdr_set_tuner_gain() against
 * mirisdr_get_tuner_gain(), checks the library against itself and agrees
 * perfectly while the dB never reach the ADC.
 *
 * So point the receiver at something steady -- a DVB-T mux is ideal, being
 * constant envelope and always there -- step the commanded gain, and watch the
 * power. Correct behaviour is a straight line of slope 1. What a wrong LNA step
 * looks like is a cliff: the aggregate setter switches the LNA in believing it
 * is worth 24 dB, takes 24 dB back out of the baseband stage to compensate, and
 * if the LNA is really worth 7 dB in this band the receiver loses the
 * difference. Asking for one more dB then costs you sixteen.
 *
 * Absolute calibration is irrelevant here; slope and continuity are the whole
 * measurement, so dBFS straight off the ADC is the right unit.
 */
static double pow_acc_i, pow_acc_q, pow_acc_ii, pow_acc_qq;
static long   pow_acc_n;
static int    pow_bufs;

#define POWER_SETTLE_BUFS 6
#define POWER_USE_BUFS    10

/*
 * How far the power must fall, against a rising command, to be called a fault.
 * Close to the noise floor successive captures wander by a couple of dB, while
 * a mis-sized stage step costs fifteen or more, so the gap between the two is
 * wide and this sits in it.
 */
#define GAIN_REVERSAL_DB  (-6.0)

static void power_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    (void) ctx;

    if (++pow_bufs > POWER_SETTLE_BUFS) {
        /* 504_S8 only: 1024 byte blocks, 16 byte header then 1008 signed bytes */
        for (uint32_t b = 0; b + 1024 <= len; b += 1024) {
            const signed char *s = (const signed char *) (buf + b + 16);
            int k;
            for (k = 0; k < 1008; k += 2) {
                double i = s[k], q = s[k + 1];
                pow_acc_i += i; pow_acc_q += q;
                pow_acc_ii += i * i; pow_acc_qq += q * q;
                pow_acc_n++;
            }
        }
    }

    if (pow_bufs >= POWER_SETTLE_BUFS + POWER_USE_BUFS) mirisdr_cancel_async(dev);
}

/* mean power in dBFS with DC removed; full scale for a signed byte is 127 */
static double measure_power(void)
{
    double mi, mq, var;

    pow_bufs = 0;
    pow_acc_i = pow_acc_q = pow_acc_ii = pow_acc_qq = 0.0;
    pow_acc_n = 0;

    mirisdr_reset_buffer(dev);
    mirisdr_read_async(dev, power_callback, NULL, 16, 65536);

    if (!pow_acc_n) return -999.0;

    mi = pow_acc_i / pow_acc_n;
    mq = pow_acc_q / pow_acc_n;
    var = (pow_acc_ii / pow_acc_n - mi * mi) + (pow_acc_qq / pow_acc_n - mq * mq);
    if (var <= 0.0) return -999.0;

    return 10.0 * log10(var / (127.0 * 127.0));
}

static int gain_sweep(int hw_flavour, uint32_t frequency,
                      uint32_t rate, uint32_t bandwidth,
                      int lo, int hi, int step, int verbose)
{
    double prev = -999.0;
    int worst_g = -1;
    double worst_drop = 0.0;
    int g, n_gains, band_max;

    mirisdr_set_hw_flavour(dev, hw_flavour ? MIRISDR_HW_SDRPLAY : MIRISDR_HW_DEFAULT);
    mirisdr_set_sample_rate(dev, rate);
    mirisdr_set_center_freq(dev, frequency);
    mirisdr_set_sample_format(dev, "504_S8");
    mirisdr_set_transfer(dev, "ISOC");
    mirisdr_set_if_freq(dev, 0);
    mirisdr_set_bandwidth(dev, bandwidth);
    mirisdr_set_tuner_gain_mode(dev, 1);

    /* the advertised ceiling depends on the band we just tuned into */
    n_gains = mirisdr_get_tuner_gains(dev, NULL);
    band_max = n_gains > 0 ? n_gains - 1 : 0;
    if (hi < 0) hi = band_max;

    fprintf(stderr, "band %d, library offers 0 - %d dB\n\n",
            (int) mirisdr_get_band(dev), band_max);

    printf("commanded  reported   dBFS    step\n");
    printf("---------  --------  -------  ------\n");

    for (g = lo; g <= hi && !do_exit; g += step) {
        double p, d;

        mirisdr_set_tuner_gain(dev, g);
        /* each capture restarts the stream, which re-seeds the block counter
         * and trips the desync message; that is expected, not a finding */
        if (!verbose) stderr_mute();
        p = measure_power();
        if (!verbose) stderr_restore();
        d = (prev > -998.0) ? p - prev : 0.0;

        printf("%9d  %8d  %7.2f  %+6.2f%s\n",
               g, mirisdr_get_tuner_gain(dev), p, d,
               (prev > -998.0 && d < GAIN_REVERSAL_DB)
                   ? "   <-- power fell as gain rose" : "");
        fflush(stdout);

        if (prev > -998.0 && d < worst_drop && d < GAIN_REVERSAL_DB) {
            worst_drop = d;
            worst_g = g;
        }
        prev = p;
    }

    if (worst_g >= 0) {
        printf("\nWorst reversal: asking for %d dB instead of %d dB cost %.2f dB.\n",
               worst_g, worst_g - step, -worst_drop);
        printf("A correct gain map is monotonic; a cliff here means the aggregate\n"
               "setter and the hardware disagree about what a stage is worth.\n");
    } else {
        printf("\nMonotonic across %d - %d dB: no stage handover loses ground.\n", lo, hi);
    }

    mirisdr_close(dev);
    dev = NULL;

    return 0;
}

/* printed once, when the device cannot be opened at all */
static void no_device_hint(int index)
{
    fprintf(stderr,
        "\nCould not open device %d.\n"
        "Usually the kernel's msi2500 driver has claimed the interface: either"
        " configure with -DDETACH_KERNEL_DRIVER=ON so the library detaches it"
        " itself, or unbind the driver once.\n", index);
}


/*
 * What this particular host actually reaches at the top.
 *
 * The limits up here are not properties of the chip alone: the sample depth
 * that fits at a given rate depends on how much the host's bulk endpoint can
 * carry, which varies with the controller and with what else is on the bus.
 * Rather than bake in the numbers measured on one machine, find them.
 *
 * Two searches, both over bulk since that is the faster transport, and both
 * with short runs because a failure declares itself immediately -- a stream
 * that cannot keep up desynchronises in the first buffers, it does not
 * degrade slowly.
 */
#define PROBE_SECS 0.6

static int measure(int hw_flavour, uint32_t rate, int mode, int fmt,
                   uint32_t frequency, uint32_t bandwidth, int gain,
                   double duration, double settle, int verbose, int quiet);

static void probe_corner(int index, int hw_flavour, uint32_t frequency,
                         uint32_t bandwidth, int gain, double settle,
                         int verbose)
{
    int f, deepest = -1;
    (void) settle;

    printf("\ncorner probe on this host, over bulk:\n");

    /*
     * The envelope points finish on isochronous and this probes over bulk, and
     * that transition is the one the device will not make on a held handle.
     * Reopen so the probe starts from bulk cleanly.
     */
    mirisdr_close(dev);
    dev = NULL;
    if (mirisdr_open(&dev, index) < 0 || !dev) {
        printf("  could not reopen for the probe\n");
        return;
    }

    /* deepest sample the ceiling rate can carry: try deep first, take the
     * first that survives */
    for (f = N_FORMATS - 1; f >= 0; f--) {
        if (f && formats[f].bits == formats[f - 1].bits) continue;  /* skip the
                                                    duplicate 8-bit container */
        if (measure(hw_flavour, MIRISDR_RATE_MAX, 2, f, frequency, bandwidth,
                    gain, PROBE_SECS, 0.25, verbose, 1) == 0) {
            deepest = f;
            break;
        }
    }

    if (deepest < 0)
        printf("  deepest sample at %.3f MS/s : none of them\n",
               MIRISDR_RATE_MAX / 1e6);
    else
        printf("  deepest sample at %.3f MS/s : %d-bit (%s)\n",
               MIRISDR_RATE_MAX / 1e6, formats[deepest].bits,
               formats[deepest].name);


}

void usage(void)
{
    fprintf(stderr,
        "miri_test, a transport benchmark for libmirisdr\n\n"
        "Measures delivered sample rate and dropped samples for each\n"
        "combination of sample rate and USB transfer mode.\n\n"
        "Usage:\n"
        "\t[-r sample rates to test, comma separated in Hz]\n"
        "\t    without -r or -m only the envelope is checked: the floor with\n"
        "\t    the smallest sample, the ceiling with the largest, and 2 MS/s\n"
        "\t    naming either sweeps 1.010 MS/s then 1.5 - 15.5 in 0.5 steps\n"
        "\t[-e transfer modes to test (default: 3)]\n"
        "\t    1:      isochronous only\n"
        "\t    2:      bulk only\n"
        "\t    3:      both\n"
        "\t[-t seconds to measure per run (default: 1)]\n"
        "\t[-p seconds to settle before measuring (default: 0.4)]\n"
        "\t[-f frequency to tune to in Hz (default: 100000000)]\n"
        "\t[-m sample formats, comma separated, or \"all\" (default: 504)]\n"
        "\t    504_S8  (or 504):  8-bit,  504 pairs per 1024 B block\n"
        "\t    504_S16:          10-bit,  504 pairs\n"
        "\t    384_S16 (or 384): 10-bit,  384 pairs, per-group exponent\n"
        "\t    336_S16 (or 336): 12-bit,  336 pairs\n"
        "\t    252_S16 (or 252): 14-bit,  252 pairs\n"
        "\t[-w bandwidth in Hz (default: 8000000)]\n"
        "\t[-g gain in dB (default: 40)]\n"
        "\t[-d device index (default: 0)]\n"
        "\t[-T device variant: 0 default, 1 SDRplay (default: 0)]\n"
        "\t[-v show the per-block loss messages while measuring]\n"
        "\n"
        "Gain linearity mode:\n"
        "\t[-G sweep commanded gain and measure the power that results]\n"
        "\t    needs a steady carrier -- a DVB-T mux is ideal\n"
        "\t    -g accepts lo,hi,step here (default 0,band max,1)\n"
        "\t    correct behaviour is a straight line of slope 1; a cliff means\n"
        "\t    the gain map and the hardware disagree\n"
        "\t    e.g. miri_test -G -T 1 -f 488000000\n");
    exit(1);
}

/*
 * One format/rate/mode combination.
 *
 * Returns 0 if the stream both arrived intact and ran at the rate asked for,
 * and non-zero otherwise, so the corner probe can use the same code path it
 * reports from. `quiet` suppresses the table row for probe runs.
 */
static int measure(int hw_flavour, uint32_t rate, int mode, int fmt,
                   uint32_t frequency, uint32_t bandwidth, int gain,
                   double duration, double settle, int verbose, int quiet)
{
    run_t run;
    double elapsed;
    uint32_t lost, events, desync;
    double mbs, msps, loss_pct, asked;
    const char *verdict;
    unsigned long long pairs;
    int r;

    memset(&run, 0, sizeof(run));
    run.settle = settle;
    run.duration = duration;
    asked = (double) rate / 1.0e6;

    run.dev = dev;

    mirisdr_set_hw_flavour(dev, hw_flavour ? MIRISDR_HW_SDRPLAY : MIRISDR_HW_DEFAULT);
    mirisdr_set_sample_rate(dev, rate);
    mirisdr_set_center_freq(dev, frequency);
    mirisdr_set_sample_format(dev, formats[fmt].name);
    mirisdr_set_transfer(dev, (mode == 2) ? "BULK" : "ISOC");
    mirisdr_set_if_freq(dev, 0);
    mirisdr_set_bandwidth(dev, bandwidth);
    mirisdr_set_tuner_gain_mode(dev, 1);
    mirisdr_set_tuner_gain(dev, gain);

    if (mirisdr_reset_buffer(dev) < 0)
        fprintf(stderr, "warning: failed to reset buffers\n");

    if (!verbose) stderr_mute();
    r = mirisdr_read_async(dev, callback, (void *) &run,
                           ASYNC_BUF_NUMBER, ASYNC_BUF_LENGTH);
    if (!verbose) stderr_restore();

    lost = mirisdr_get_lost_samples(dev);
    events = mirisdr_get_lost_events(dev);
    desync = mirisdr_get_desync_events(dev);

    if (r < 0 || !run.measuring || run.t_last <= run.t_begin) {
        if (!quiet) {
            printf("%-8s %3db  %-5s %8.3f %8s  %9s  %7s %7s  %7s  %s\n",
                   formats[fmt].name, formats[fmt].bits,
                   (mode == 2) ? "bulk" : "isoc", asked,
                   "-", "-", "-", "-", "-", (r < 0) ? "ERROR" : "NO DATA");
            fflush(stdout);
        }
        return -1;
    }

    elapsed = run.t_last - run.t_begin;
    pairs = run.bytes / formats[fmt].pair_bytes;
    /*
     * Report bytes on the wire, not converted output, since the s16 converters
     * expand and only the wire figure is comparable between formats and against
     * the 24.576 MB/s isochronous slot (3 x 1024 B per microframe x 8000
     * microframes/s, decimal MB).
     */
    mbs = ((double) pairs / (double) formats[fmt].pairs) * 1024.0 / elapsed / 1.0e6;
    msps = (double) pairs / elapsed / 1.0e6;
    /* loss is against what should have arrived: delivered plus dropped */
    loss_pct = (pairs + lost) ? 100.0 * (double) lost / (double) (pairs + lost) : 0.0;

    /*
     * Once the framing has slipped the sequence counter stops meaning anything,
     * so a sample count taken across that stretch is not a smaller truth, it is
     * not a truth at all. Print the fact instead of a number.
     */
    /*
     * Judge it on the rate delivered as well as on the loss counters, in both
     * directions. A stream can arrive with no gap in the block counter at all
     * and still be nowhere near the rate that was asked for -- 10-bit at 15.5
     * MS/s over isochronous returns 9.216 with a spotless loss column, because
     * that is the whole isochronous slot in this format's geometry and the
     * transport can carry no more; and a request below the rate PLL's VCO floor
     * returns something faster than asked, equally clean. Reporting only the
     * counters would call either a pass.
     */
    if (desync)              verdict = "DESYNC";
    else if (lost)           verdict = "LOSS";
    else if (msps < asked * (1.0 - RATE_TOL)) verdict = "SHORT";
    else if (msps > asked * (1.0 + RATE_TOL)) verdict = "OVER";
    else                     verdict = "ok";

    if (!quiet) {
        printf("%-8s %3db  %-5s %8.3f %8.3f  %9.2f  %7u %7u  %6.3f%%  %s\n",
               formats[fmt].name, formats[fmt].bits,
               (mode == 2) ? "bulk" : "isoc",
               asked, msps, mbs, events, desync, loss_pct, verdict);
        fflush(stdout);
    }

    return strcmp(verdict, "ok") ? 1 : 0;
}

int main(int argc, char **argv)
{
#if !defined(_WIN32) || defined(__MINGW32__)
    struct sigaction sigact;
#endif
    uint32_t rates[MAX_RATES];
    int n_rates = 0;
    char *rate_arg = NULL;
    char *rate_str, *tok;
    uint32_t frequency = DEFAULT_FREQUENCY;
    uint32_t bandwidth = DEFAULT_BANDWIDTH;
    int bw_set = 0;
    int gain = DEFAULT_GAIN;
    int gain_mode = 0;
    int gain_lo = 0, gain_hi = -1, gain_step = 1;   /* hi < 0 means band max */
    char *fmt_arg = "504";
    int fmt_given = 0;
    char *fmt_str;
    int fmts[N_FORMATS];
    int n_fmts = 0;
    int modes = 3;
    int index = 0;
    int hw_flavour = 0;
    int verbose = 0;
    double duration = DEFAULT_DURATION;
    double settle = DEFAULT_SETTLE;
    int opt, i, f, m;
    int use_plan;
    uint32_t device_count;

    while ((opt = getopt(argc, argv, "r:e:t:p:f:m:w:g:d:T:vG")) != -1) {
        switch (opt) {
        case 'r': rate_arg = optarg; break;
        case 'e': modes = atoi(optarg); break;
        case 't': duration = atof(optarg); break;
        case 'p': settle = atof(optarg); break;
        case 'f': frequency = (uint32_t) atof(optarg); break;
        case 'm': fmt_arg = optarg; fmt_given = 1; break;
        case 'w': bandwidth = (uint32_t) atof(optarg); bw_set = 1; break;
        case 'g':
            /* a single value for the benchmark, lo,hi,step for -G */
            if (sscanf(optarg, "%d,%d,%d", &gain_lo, &gain_hi, &gain_step) != 3) {
                gain_lo = 0; gain_hi = -1; gain_step = 1;
                gain = atoi(optarg);
            }
            break;
        case 'G': gain_mode = 1; break;
        case 'd': index = atoi(optarg); break;
        case 'T': hw_flavour = atoi(optarg); break;
        case 'v': verbose = 1; break;
        default: usage(); break;
        }
    }

    if (modes < 1 || modes > 3) {
        fprintf(stderr, "transfer mode selection must be 1, 2 or 3\n");
        usage();
    }
    if (duration <= 0.0 || settle < 0.0) {
        fprintf(stderr, "measurement and settle times must be positive\n");
        usage();
    }

    /* a bare run walks the envelope; naming rates or formats sweeps them */
    use_plan = (!rate_arg && !fmt_given);

    if (!strcasecmp(fmt_arg, "all")) {
        for (n_fmts = 0; n_fmts < N_FORMATS; n_fmts++) fmts[n_fmts] = n_fmts;
    } else {
        if (!(fmt_str = strdup(fmt_arg))) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        for (tok = strtok(fmt_str, ","); tok; tok = strtok(NULL, ",")) {
            if ((f = format_lookup(tok)) < 0) {
                fprintf(stderr, "unknown sample format '%s'\n", tok);
                free(fmt_str);
                usage();
            }
            if (n_fmts == N_FORMATS) {
                fprintf(stderr, "at most %d sample formats\n", N_FORMATS);
                free(fmt_str);
                exit(1);
            }
            fmts[n_fmts++] = f;
        }
        free(fmt_str);
    }

    if (!n_fmts) {
        fprintf(stderr, "no sample formats given\n");
        usage();
    }

    if (!rate_arg) {
        /*
         * The rate PLL is a genuine fractional-N -- hard.c writes the low 20
         * bits of the fraction to reg 0x04 and bit 20 to reg 0x03 -- so its
         * resolution is well under a hertz and there is no grid to enumerate.
         * Sweep the range evenly instead, from the lowest rate the driver will
         * accept to the highest.
         */
        rates[n_rates++] = MIRISDR_RATE_MIN;
        for (uint32_t r = 1500000; r <= MIRISDR_RATE_MAX && n_rates < MAX_RATES;
             r += 500000)
            rates[n_rates++] = r;
    } else {
        /* strtok writes to its argument, so give it a copy */
        if (!(rate_str = strdup(rate_arg))) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        for (tok = strtok(rate_str, ","); tok; tok = strtok(NULL, ",")) {
            uint32_t asked = (uint32_t) atof(tok);

            if (n_rates == MAX_RATES) {
                fprintf(stderr, "at most %d sample rates\n", MAX_RATES);
                free(rate_str);
                exit(1);
            }
            if (asked < MIRISDR_RATE_MIN)
                fprintf(stderr, "note: %u S/s is below the driver's %u S/s"
                                " minimum and will be clamped\n",
                        asked, MIRISDR_RATE_MIN);
            rates[n_rates++] = asked;
        }
        free(rate_str);
    }

    if (!n_rates) {
        fprintf(stderr, "no sample rates given\n");
        usage();
    }

    device_count = mirisdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported devices found.\n");
        exit(1);
    }
    if (index < 0 || (uint32_t) index >= device_count) {
        fprintf(stderr, "Device %d out of range, %u present.\n", index, device_count);
        exit(1);
    }

#if !defined(_WIN32) || defined(__MINGW32__)
    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
#else
    SetConsoleCtrlHandler((PHANDLER_ROUTINE) sighandler, TRUE);
#endif

    fprintf(stderr, "Using device %d: %s\n", index, mirisdr_get_device_name(index));

    /*
     * Open once and hold it for the whole sweep. Reopening per run made the
     * library detach and reattach the kernel driver each time, which is both
     * noisy and slow, and it meant a device that could not be claimed produced
     * one failure per run instead of one failure. Enumerating it proves
     * nothing: the kernel claims the interface rather than the device, so it is
     * visible and unopenable at the same time.
     */
    if (mirisdr_open(&dev, index) < 0 || !dev) {
        no_device_hint(index);
        return 1;
    }

    if (gain_mode) {
        /*
         * Narrow and slow by default: the point is a stable power reading, not
         * throughput, and a wide filter would let the neighbouring muxes move
         * the number around. -r and -w still override.
         */
        uint32_t g_rate = rate_arg ? rates[0] : 2000000;
        uint32_t g_bw   = bw_set   ? bandwidth : 1536000;

        if (gain_step < 1) gain_step = 1;
        fprintf(stderr, "gain linearity at %u Hz, bw %u Hz, %u S/s, 504_S8\n",
                frequency, g_bw, g_rate);
        gain_sweep(hw_flavour, frequency, g_rate, g_bw,
                   gain_lo, gain_hi, gain_step, verbose);
        mirisdr_close(dev);
        return 0;
    }

    fprintf(stderr, "%u Hz, bw %u Hz, gain %d dB, %.1f s per run"
                    " after %.1f s settling\n\n",
            frequency, bandwidth, gain, duration, settle);

    {
        int n_modes = ((modes & 1) ? 1 : 0) + ((modes & 2) ? 1 : 0);
        int runs = 0;

        if (use_plan) {
            for (i = 0; i < N_PLAN; i++)
                runs += ((default_plan[i].modes & 1) && (modes & 1) ? 1 : 0)
                      + ((default_plan[i].modes & 2) && (modes & 2) ? 1 : 0);
        } else {
            runs = n_rates * n_fmts * n_modes;
        }
        if (use_plan)
            fprintf(stderr, "envelope check: %d points, %d runs, about %.0f s"
                            "  (-r or -m sweeps instead)\n\n",
                    N_PLAN, runs, runs * (duration + settle + 1.3));
        else
            fprintf(stderr, "%d rates x %d formats x %d modes = %d runs,"
                            " about %.0f s\n\n",
                    n_rates, n_fmts, n_modes, runs,
                    runs * (duration + settle + 1.0));
    }

    printf("format   bits  mode     asked      got   USB MB/s   events  desync     loss  verdict\n");
    printf("-------- ----  ----- -------- --------  ---------  ------- -------  -------  --------\n");

    /*
     * Mode-major, bulk before isochronous. Both orderings matter on a device
     * that is held open rather than reopened per run: switching from the
     * isochronous alternate setting to the bulk one leaves the endpoint failing
     * every transfer, while the reverse is fine, and grouping by mode means the
     * sweep changes mode once rather than once per rate.
     */
    for (m = 0; m < 2 && !do_exit; m++) {
        int mode = m ? 1 : 2;               /* bulk first, then isochronous */

        if (!(modes & mode)) continue;

        if (use_plan) {
            for (i = 0; i < N_PLAN && !do_exit; i++) {
                if (!(default_plan[i].modes & mode)) continue;
                measure(hw_flavour, default_plan[i].rate, mode,
                        format_lookup(default_plan[i].fmt), frequency,
                        bandwidth, gain, duration, settle, verbose, 0);
            }
            continue;
        }

        for (f = 0; f < n_fmts && !do_exit; f++)
            for (i = 0; i < n_rates && !do_exit; i++)
                measure(hw_flavour, rates[i], mode, fmts[f], frequency,
                        bandwidth, gain, duration, settle, verbose, 0);
    }

    if (use_plan && !do_exit)
        probe_corner(index, hw_flavour, frequency, bandwidth, gain, settle,
                     verbose);

    if (do_exit) fprintf(stderr, "\nUser cancel, exiting...\n");

    if (dev) mirisdr_close(dev);

    return 0;
}
