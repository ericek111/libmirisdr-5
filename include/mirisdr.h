/*
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
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

#ifndef __MIRISDR_H
#define __MIRISDR_H

#ifdef __cplusplus
extern "C" {
#endif

// Set debug level
// 0=no debug
// 1=gain and frequency info.
// 2=extended debug (register writes)
// 3=more extended debug (function calls)
#define MIRISDR_DEBUG 0

#include <stdint.h>
#include <mirisdr_export.h>

typedef enum
{
    MIRISDR_HW_AUTO = -1,       /* open config only: by VID:PID, else MIRISDR_HW_DEFAULT; under
                                   our own ids (16d0:158c) a serial starting "B-" is an RSP1B,
                                   "C-" an RSPdx-R2, "D-" an RSPduo */
    MIRISDR_HW_DEFAULT,
    MIRISDR_HW_SDRPLAY,
    MIRISDR_HW_RSP1B,           /* SDRplay RSP1B */
    MIRISDR_HW_RSPDXR2,         /* SDRplay RSPdx-R2 */
    MIRISDR_HW_RSPDUO,          /* SDRplay RSPduo */
} mirisdr_hw_flavour_t;

typedef enum
{
    MIRISDR_BAND_AM1,
    MIRISDR_BAND_AM2,
    MIRISDR_BAND_VHF,
    MIRISDR_BAND_3,
    MIRISDR_BAND_45,
    MIRISDR_BAND_L,
} mirisdr_band_t;

typedef struct mirisdr_dev mirisdr_dev_t;

/* Some padding in the external structs */
#define MIRISDR_RESERVED        32
#define MIRISDR_RESERVED_NESTED 8

/* devices */
MIRISDR_API uint32_t mirisdr_get_device_count (void);
MIRISDR_API const char *mirisdr_get_device_name (uint32_t index);
/* The serial is the system's copy (libusb >= 1.0.30, or sysfs on Linux), otherwise read by opening
   the device for a moment. If serial is NULL, the device is never opened. */
MIRISDR_API int mirisdr_get_device_usb_strings (uint32_t index, char *manufact, char *product, char *serial);
/* This function finds the index of a receiver by the (possibly synthetic) serial. Warning:
 * these functions have a race condition: the index can change if devices are added or removed.
 * To avoid the race condition, use mirisdr_open_ex() with a serial */
MIRISDR_API int mirisdr_get_index_by_serial (const char *serial);
/* The same with a USB port too, either may be NULL: see usb_port in mirisdr_open_config_t */
MIRISDR_API int mirisdr_get_index_by_serial_port (const char *serial, const char *usb_port); /* extra */
/* The USB port the device is in, "<bus>:<port>[.<port>...]" e.g. "1:3" (the synthetic serial of one
   without a serial of its own), which stays the same while the device stays in that port.
   Serial (may be NULL) the serial it declares, empty if none. */
MIRISDR_API int mirisdr_get_device_port (uint32_t index, char *port, int port_len, char *serial, int serial_len); /* extra */

/* main */
MIRISDR_API int mirisdr_open (mirisdr_dev_t **p, uint32_t index);
MIRISDR_API int mirisdr_open_fd (mirisdr_dev_t **p, int fd);
MIRISDR_API int mirisdr_close (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_reset (mirisdr_dev_t *p);                       /* extra */
MIRISDR_API int mirisdr_reset_buffer (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_get_usb_strings (mirisdr_dev_t *dev, char *manufact, char *product, char *serial);
MIRISDR_API int mirisdr_get_usb_ids (mirisdr_dev_t *p, uint16_t *vid, uint16_t *pid); /* extra */
/* The serial the device itself declares, returns 0 if none declared */
MIRISDR_API int mirisdr_get_serial (mirisdr_dev_t *p, char *out, int len); /* extra */
/* Where the device sits on the USB: bus number and device address (as lsusb shows them, either may
   be NULL) and in port (len bytes, may be NULL) the port path as Linux sysfs names the device,
   "<bus>-<port>[.<port>...]", e.g. "1-1.2"; 32 bytes hold any path. They find the device's host
   controller (sysfs, or a service listing controllers with their devices, for USB SOF timing).
   Returns 0, or -1 (no device, or len too small for the path) */
MIRISDR_API int mirisdr_get_usb_position (mirisdr_dev_t *p, uint8_t *busnum, uint8_t *devnum, char *port, int len); /* extra */
MIRISDR_API int mirisdr_set_hw_flavour (mirisdr_dev_t *p, mirisdr_hw_flavour_t hw_flavour);
/* Get detected/selected flavour */
MIRISDR_API int mirisdr_get_hw_flavour (mirisdr_dev_t *p);

/* sync */
MIRISDR_API int mirisdr_read_sync (mirisdr_dev_t *p, void *buf, int len, int *n_read);

/* async
 *
 * Threads: one thread at a time makes the control calls (settings, tuning, scans,
 * GPIO and the other I/O); the library has no lock around them. The callback runs on
 * the thread inside mirisdr_read_async() and may only cancel and read the stream
 * (buffer info, stats, mirisdr_get_stream()). mirisdr_cancel_async() may come from any
 * thread, also during a setting that restarts the stream. During such a setting the
 * callback can run on the control thread, for the transfers that come back while it waits.
 * See doc/c-api.md, Threads. */
typedef void(*mirisdr_read_async_cb_t) (unsigned char *buf, uint32_t len, void *ctx);
MIRISDR_API int mirisdr_read_async (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len);
/* A cancel is kept from the moment mirisdr_read_async() is entered, also before its
 * transfers run. One made before that call is not: check a flag of your own in the callback.
 * Once a cancel is pending, another returns 0; without a stream it returns -1 */
MIRISDR_API int mirisdr_cancel_async (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_cancel_async_now (mirisdr_dev_t *p);            /* extra */
MIRISDR_API int mirisdr_start_async (mirisdr_dev_t *p);                 /* extra */
MIRISDR_API int mirisdr_stop_async (mirisdr_dev_t *p);                  /* extra */

/* adc */
MIRISDR_API int mirisdr_adc_init (mirisdr_dev_t *p);                    /* extra */

/* rate control */
MIRISDR_API int mirisdr_set_sample_rate (mirisdr_dev_t *p, uint32_t rate);
MIRISDR_API uint32_t mirisdr_get_sample_rate (mirisdr_dev_t *p);

/*
 * The PLL of the device is limited to ~15MHz, so higher sample rates require 
 * turning off the internal decimator. This results in a doubled output rate,
 * but requires the application to do some extra filtering to get useful results.
 *
 * Three modes are available:
 *   "AUTO" (default)  bypass only above 14.5 Msps
 *   "ON"              always bypass (the PLL runs at the sample rate)
 *   "OFF"             never bypass  (the PLL runs at twice the sample rate)
 */
MIRISDR_API int mirisdr_set_decimation_bypass (mirisdr_dev_t *p, const char *v);  /* extra */
MIRISDR_API const char *mirisdr_get_decimation_bypass (mirisdr_dev_t *p);         /* extra */

/* sample format control */
/*
 * Complex formats deliver interleaved I/Q, so n samples are 2n int16. The
 * REAL formats digitise a single converter. Every int16 is one real sample.
 * This means that n samples are n int16.
 *
 * The following rates are available:
 *   AUTO, 252_S16, 336_S16, 384_S16, 504_S16, 504_S8
 *   AUTO_REAL, 504_REAL_S16, 672_REAL_S16, 768_REAL_S16
 */
MIRISDR_API int mirisdr_set_sample_format (mirisdr_dev_t *p, const char *v);  /* extra */
MIRISDR_API const char *mirisdr_get_sample_format (mirisdr_dev_t *p);   /* extra */

/*
 * This function returns the selected sample format, eg:
 * set: AUTO get: 336_S16
 */
MIRISDR_API const char *mirisdr_get_sample_format_selected (mirisdr_dev_t *p);  /* extra */

/*
 * For complex formats this swaps I and Q. For real formats, this selects which
 * converter is used (swap=Q, no-swap=I).
 */
MIRISDR_API int mirisdr_set_swap_iq (mirisdr_dev_t *p, int swap);       /* extra */
MIRISDR_API int mirisdr_get_swap_iq (mirisdr_dev_t *p);                 /* extra */

/*
 * Counts are in the unit the hardware counter uses:
 * I/Q pairs in the complex formats, single samples in the real ones.
 */
typedef struct mirisdr_stream_stats
{
	uint64_t samples;   /* total number delivered to the application */
	uint64_t lost;      /* total number of samples lost */
	uint64_t gaps;      /* number of gaps in the stream */
	uint64_t jitter;    /* number of jitter events. These happen at very high rates and are harmless. */
	uint64_t resyncs;   /* byte alignment recoveries */
	uint64_t index;     /* absolute index of the first sample in the buffer */
	uint64_t filled;    /* samples gap fill put in the stream, counted in samples, not in lost */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_stream_stats_t;

MIRISDR_API int mirisdr_get_stream_stats (mirisdr_dev_t *p, mirisdr_stream_stats_t *s); /* extra */

/*
 * Where samples are missing in the buffer a mirisdr_read_async() callback is being
 * called with. The buffer's sample at position p has the absolute index:
 * index + p + the sum of (samples - filled)
 * over the gaps with 0 < offset <= p.
 * Note: A gap at offset 0 lies before its first sample.
 *
 * Each 1 kB USB block has at most one gap before it, so a buffer of automatic
 * length (one transfer, up to 24 blocks) never overflows gaps. A larger fixed
 * length can, and the gaps past MIRISDR_GAPS_MAX count in gap_samples only.
 */
#define MIRISDR_GAPS_MAX 24

typedef struct mirisdr_gap
{
	uint32_t offset;    /* samples of the buffer before the gap, filled ones of earlier gaps included */
	uint32_t filled;    /* of the missing samples, how many gap fill put in, from offset on */
	uint64_t samples;   /* samples missing here */
} mirisdr_gap_t;

typedef struct mirisdr_buffer_info
{
	uint64_t sample;        /* samples delivered before this buffer, filled ones included */
	uint64_t index;         /* absolute index of its first sample */
	uint64_t gap_samples;   /* samples missing inside it, all gaps added up */
	uint32_t gaps_len;
	mirisdr_gap_t gaps[MIRISDR_GAPS_MAX];
	int      adc;           /* MIRISDR_IQ_BOTH for complex (I/Q pairs),
	                           MIRISDR_IQ_ONLY_I or _ONLY_Q for real */
	uint32_t rate;          /* samples per second */
	int      type;          /* MIRISDR_SAMPLE_* */
	int      tuner;         /* the tuner the samples are from: 0, or on an RSPduo streaming
	                           tuner 1 or both, 1 too (see the stream config's tuners) */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_buffer_info_t;

#define MIRISDR_SAMPLE_S16      0       /* int16, full scale 32768 */
#define MIRISDR_SAMPLE_S8       1       /* int8, full scale 128 (504_S8) */
#define MIRISDR_SAMPLE_F32      2       /* float, full scale 1.0 (baseband) */

/* Call from inside the read_async callback only */
MIRISDR_API int mirisdr_get_buffer_info (mirisdr_dev_t *p, mirisdr_buffer_info_t *info); /* extra */

/* Gap fill, read_async only: missing samples are replaced by zeros right before the
 * samples that follow the gap. Off by default. */
MIRISDR_API int mirisdr_set_gap_fill (mirisdr_dev_t *p, int on); /* extra */

/*
 * The stream set up in one call. Start from mirisdr_get_stream() (or
 * mirisdr_stream_config_default()), change what is needed and apply. The strings take
 * the values of the single setters above, NULL for their default. A rate out of reach
 * is taken to the nearest there is, unless strict. An invalid or conflicting combination
 * is refused as a whole, rather than adjusted. While streaming
 * the stream restarts with a short gap, as with the single setters. The buffer being
 * filled is dropped, so no buffer mixes two formats, and mirisdr_get_buffer_info()
 * says whether each one is real or complex. mirisdr_stream_check() says what a config
 * would give without applying it.
 *
 * With follow_tune the stream is real or complex as the tune needs it: format while
 * the tune runs both tuner outputs, format_single on the converter of the one it
 * runs (see mirisdr_tune()). A tune then switches the stream itself, in one restart.
 *
 * With baseband the callback gets complex float samples (MIRISDR_SAMPLE_F32) with the
 * tune's frequency at 0 Hz, and rate is that output's rate. At zero IF they are the
 * samples as they come. With a low IF the LO goes the IF above the frequency (as
 * low_if_auto). Both tuner outputs are shifted by a quarter of the converters' rate,
 * which is a sign swap a sample. A single output (iq) goes through a real to I/Q
 * converter instead, at half the rate: half the USB data, with more bits at high rates,
 * for more CPU. With the converters at 4 x IF that puts the band at 0 Hz; at any other
 * rate an NCO takes it the rest of the way. The image rejection is dominated by the
 * tuner (about 37 dB measured). Half-band stages (the last flat to 0.4 of its output
 * rate, 63 dB down from 0.6) then halve the rate, and a resampler (the same, flat to 0.4 of
 * the output rate, 63 dB down from 0.6) makes what halving leaves, so any rate works from just
 * over the converter rate / 512. The converters run at 4 x IF up to 7 Msps (above it
 * the receiver's analog performance falls off), else at the least the IF takes,
 * 2.4 x (IF + half its widest filter), or at the output rate if higher: 1.8, 6.48 or
 * 6.7584 Msps at 450, 1620 or 2048 kHz. 4 x IF divided by a power of 2 needs neither
 * the resampler nor the NCO. adc_rate, nco and resampled in the result say what runs.
 * A baseband stream follows the tune as with follow_tune, and a tune that changes the
 * IF keeps the rate where the new IF has it, else takes the nearest it has. Buffer infos count in output
 * samples, their gaps placed to within the filters delay. The stream stats, events,
 * PPS and scan reports still count the converters' samples, decimation (in the stream
 * result) of them to an output sample. Gap fill is always on, so the filters see time
 * pass. swap_iq still mirrors the spectrum with both outputs, as without baseband. With one
 * output it picks the converter, and two tuners refuse it.
 * mirisdr_read_sync() does not support baseband output, and needs the BULK transfer.
 *
 * Bulk transfers max bandwidth is very host dependent, so only the receiver's own limit
 * (~56 MB/s) is enforced there. The automatic choice takes the format with the most
 * bits that fits a figure: usb_capacity, 32 MB/s by default in bulk, or an isochronous
 * mode's reservation. In bulk, if above 32MB/s, it takes the densest (8-bit samples,
 * 768_REAL for a single converter).
 */
typedef struct mirisdr_stream_config
{
	uint32_t    rate;               /* samples per second */
	const char *format;             /* as mirisdr_set_sample_format(), NULL for "AUTO" */
	const char *transfer;           /* as mirisdr_set_transfer(), NULL for the platform's default */
	const char *decimation_bypass;  /* as mirisdr_set_decimation_bypass(), NULL for "AUTO" */
	int         swap_iq;            /* as mirisdr_set_swap_iq(): a real format captures Q when set */
	int         gap_fill;           /* as mirisdr_set_gap_fill() */
	int         follow_tune;        /* real or complex as the tune needs it, see above */
	const char *format_single;      /* with follow_tune: the real format for one tuner output,
	                                   NULL for "AUTO_REAL" */
	uint32_t    usb_capacity;       /* B/s the automatic choice may plan on, 0 for the default */
	int         baseband;           /* complex float centred on the tune's frequency, see above */
	int         tuners;             /* the tuners streamed, bit n for tuner n, 0 is tuner 0 alone.
	                                   The RSPduo also takes 2 (tuner 1 alone) and 3 (both, a low IF
	                                   each on a converter of its own: see "two tuners" below) */
	int         strict;             /* 0, the default: a rate out of reach (outside the converters'
	                                   range for the decimation setting, past the engine's limit
	                                   for the format, outside the span a baseband IF takes) is
	                                   taken to the nearest there is, and the result says what is in
	                                   force. 1: such a rate is refused. Either way a combination
	                                   the receiver cannot do (a rate past an isochronous mode's
	                                   reservation, an unknown string, two tuners without baseband)
	                                   is refused. A tune that changes the IF of a baseband stream
	                                   still takes the rate to the nearest the new IF has */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_stream_config_t;

typedef struct mirisdr_stream_result
{
	uint32_t    rate;               /* samples per second, as the sample clock reaches it */
	const char *format;             /* the one running, never an AUTO one */
	int         adc;                /* what is digitised: MIRISDR_IQ_BOTH, _ONLY_I or _ONLY_Q */
	int         decimation_bypassed;
	uint32_t    usb_bytes;          /* per second */
	uint32_t    usb_capacity;       /* B/s the automatic choice planned on */
	int         baseband;           /* MIRISDR_BASEBAND_*: baseband output type */
	uint32_t    adc_rate;           /* the converters' samples per second */
	double      decimation;         /* converter samples per output sample, 1 without baseband;
	                                   a fraction where the resampler runs */
	double      group_delay;        /* the baseband filters' delay in converter samples: output
	                                   sample m is centred on converter sample
	                                   m x decimation - group_delay (the count the stream stats,
	                                   events, PPS and scan reports use), counted from the
	                                   start or the last restart. 0 without filters */
	int         gap_fill;           /* in effect: as asked, and always under baseband */
	int         nco;                /* a low IF baseband stream runs the NCO: the converters
	                                   are not at 4 x IF, which costs CPU (see above) */
	int         resampled;          /* ... and the resampler: the rate is not the converters'
	                                   divided by a power of 2, which costs CPU too */
	uint32_t    adc_clock;          /* Hz the converters sample at: twice adc_rate, or adc_rate with the
	                                   decimation bypassed. See MIRISDR_ADC_CLOCK_BEST */
	int         bits;               /* bits per sample the format carries: 14, 12, 10 or 8 */
	int         tuners;             /* the tuners streamed, as the config's tuners (never 0) */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_stream_result_t;

/* The converters are at their best up to a 15 MHz clock (7.5 Msps, twice that with the
   decimation bypassed). Above it their effective resolution drops */
#define MIRISDR_ADC_CLOCK_BEST   15000000

#define MIRISDR_BASEBAND_OFF     0      /* raw samples */
#define MIRISDR_BASEBAND_ZERO_IF 1      /* zero IF, as float */
#define MIRISDR_BASEBAND_COMPLEX 2      /* low IF, both outputs shifted */
#define MIRISDR_BASEBAND_REAL    3      /* low IF, one output through the real to I/Q converter */
#define MIRISDR_BASEBAND_DUAL    4      /* two tuners, each a low IF on its own converter, each as
                                           MIRISDR_BASEBAND_REAL: buffers labelled with their tuner */

MIRISDR_API void mirisdr_stream_config_default (mirisdr_stream_config_t *cfg); /* extra */
MIRISDR_API int mirisdr_set_stream (mirisdr_dev_t *p, const mirisdr_stream_config_t *cfg,
                                    mirisdr_stream_result_t *res); /* extra */
MIRISDR_API int mirisdr_stream_check (mirisdr_dev_t *p, const mirisdr_stream_config_t *cfg,
                                      mirisdr_stream_result_t *res); /* extra */
MIRISDR_API int mirisdr_get_stream (mirisdr_dev_t *p, mirisdr_stream_config_t *cfg,
                                    mirisdr_stream_result_t *res); /* extra */

/* A fake device for unit tests */
MIRISDR_API int mirisdr_open_null (mirisdr_dev_t **p, const char *format); /* extra */
MIRISDR_API int mirisdr_feed_bulk (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t buf_len,
                                   const uint8_t *data, uint32_t n); /* extra */

/* streaming control */
MIRISDR_API int mirisdr_streaming_start (mirisdr_dev_t *p);             /* extra */
MIRISDR_API int mirisdr_streaming_stop (mirisdr_dev_t *p);              /* extra */

/* Stall the streaming endpoint or clear it, when in bulk mode.
 * This is mostly for testing. */
MIRISDR_API int mirisdr_set_endpoint_halt (mirisdr_dev_t *p, int on);   /* extra */

/* frequency */
MIRISDR_API int mirisdr_set_center_freq (mirisdr_dev_t *p, uint32_t freq);
MIRISDR_API uint32_t mirisdr_get_center_freq (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_set_if_freq (mirisdr_dev_t *p, uint32_t freq);  /* extra */
MIRISDR_API uint32_t mirisdr_get_if_freq (mirisdr_dev_t *p);            /* extra */
MIRISDR_API int mirisdr_set_xtal_freq (mirisdr_dev_t *p, uint32_t freq);/* extra */
MIRISDR_API uint32_t mirisdr_get_xtal_freq (mirisdr_dev_t *p);          /* extra */
MIRISDR_API int mirisdr_set_bandwidth (mirisdr_dev_t *p, uint32_t bw);  /* extra */
MIRISDR_API uint32_t mirisdr_get_bandwidth (mirisdr_dev_t *p);          /* extra */
MIRISDR_API int mirisdr_set_offset_tuning (mirisdr_dev_t *p, int on);   /* extra */
MIRISDR_API mirisdr_band_t mirisdr_get_band (mirisdr_dev_t *p);         /* extra */

/*
 * Tuning in one call. Start from mirisdr_get_tune() (or mirisdr_tune_config_default()),
 * change what is needed and apply: everything goes out as one request. An invalid
 * combination is refused as a whole, rather than adjusted as the single setters do.
 * mirisdr_tune_check() says what a config would give without applying it.
 *
 * In low IF modes the tuner passes the band if_freq below its LO and rejects the
 * other side (by ~40 dB). With low_if_auto the LO is put if_freq above `frequency`,
 * so that is what is received. The I and Q outputs then each carry it as a real signal
 * at if_freq, so either can be switched off (iq) and captured with a real sample
 * format on the matching converter (see mirisdr_set_swap_iq()). A tune that switches
 * off the output the stream captures is refused, unless the stream follows the tune
 * (see mirisdr_set_stream()): the tune then switches it, in one restart.
 *
 * mirisdr_get_center_freq() returns the LO.
 * mirisdr_set_center_freq() sets the LO: it clears low_if_auto. Under a
 * baseband stream at a low IF it sets the frequency put at 0 Hz instead, the LO going
 * the IF above it as with any tune there.
 *
 * tuner is which tuner of the receiver: 0, or on the RSPduo 0 or 1 (see
 * mirisdr_get_tuner_count()).
 */
#define MIRISDR_IQ_BOTH         0
#define MIRISDR_IQ_ONLY_I       1
#define MIRISDR_IQ_ONLY_Q       2

/* The gain with a tune. TOTAL is split as mirisdr_set_tuner_gain() does, the front end
 * first so noise figure and IIP3 stay low, again for each band: the board's amplifier
 * before the tuner's stages. Above the band's maximum is clamped. STAGES sets each
 * stage and keeps them through retunes, as the single stage setters do. A tune result
 * reports both. The board's pads are apart, frontend.attenuator: the total does not
 * count them. */
#define MIRISDR_GAIN_KEEP       0       /* do not change */
#define MIRISDR_GAIN_TOTAL      1
#define MIRISDR_GAIN_STAGES     2

typedef struct mirisdr_gain_config
{
	int      mode;          /* MIRISDR_GAIN_* */
	int      total;         /* dB */
	int      lna;           /* on or off: 24 dB, 7 in band IV/V, ~4 in L; the AM ports don't have it */
	int      mixer;         /* on or off: 19 dB */
	int      mixbuffer;     /* dB, the AM ports only: 0, 6, 12 or 18 on AM1, 0 or 24 on AM2 */
	int      baseband;      /* dB, 0-59 */
	int      amp_bypass;    /* the board's amplifier before the tuner bypassed (RSP1B, RSPdx-R2
	                           and RSPduo, where it is, see mirisdr_get_frontend_caps()). Its gain
	                           in the band is in the total, up to about 25 dB. 0 where there is
	                           none */
	uint32_t reserved[MIRISDR_RESERVED_NESTED];
} mirisdr_gain_config_t;

/* The words the board's switches get, raw: register 8's GPIO bits and the SDRplay I/O
 * expanders. A tune works them out from its band plan and the frontend fields. A part
 * named in fields replaces that word whole, and the fields no longer reach it (a raw
 * GPIO word decides the RSPdx-R2's direct sampling switch, GPIO_2, too). Pins held with
 * mirisdr_set_gpio_direction() and friends stay held. For boards not mapped yet and paths
 * the plan does not choose. A part the board does not have is refused. The tune result
 * reports the words in force, with fields naming every part the board has */
#define MIRISDR_FRONTEND_RAW_GPIO   (1 << 0)    /* gpio_dir and gpio_level */
#define MIRISDR_FRONTEND_RAW_EXP1   (1 << 1)    /* expander[0]: RSP1B, RSPdx-R2, the RSPduo's first */
#define MIRISDR_FRONTEND_RAW_EXP2   (1 << 2)    /* expander[1]: the RSPduo's second */

typedef struct mirisdr_frontend_raw
{
	uint32_t fields;        /* MIRISDR_FRONTEND_RAW_*: the parts replaced, 0 none */
	uint8_t  gpio_dir;      /* bit n: GPIO_n an output */
	uint8_t  gpio_level;    /* bit n: GPIO_n high, as an output */
	uint16_t expander[2];   /* MCP23S18 IODIR, port B in the high byte: a 0 bit pulls the pin low */
	uint32_t reserved[MIRISDR_RESERVED_NESTED];
} mirisdr_frontend_raw_t;

/* frontend.flags: the board's switches, a bit set on. A flag the board does not have is
 * refused. They are the board's, not a tuner's: a tune of either tuner sets them for both,
 * and both tuners' tune configs and results then have them, the last tune's winning */
#define MIRISDR_FRONTEND_PORT_TIE   (1 << 0)    /* RSPduo: ties its two 50 ohm ports, so each
                                                   tuner hears both, as from one antenna. The
                                                   port's own chain loses 0-5 dB at HF, 3-10 dB
                                                   at 50-600 MHz, and the other tuner hears it within
                                                   5-10 dB at 50-200 and 600-900 MHz, much less at
                                                   300-433 MHz and in L band */
#define MIRISDR_FRONTEND_REF_OUT    (1 << 1)    /* RSPduo: the board's 24 MHz out of its
                                                   reference connector, for another board */

/* The front end before the tuner, part of a tune so it goes out with it. The tune
 * result reports what is in force. A part the board does not have (a notch, an antenna,
 * pads, a flag, direct sampling) is refused */
typedef struct mirisdr_frontend_config
{
	int      bias;          /* bias-T: power on the antenna input (port B on the RSPdx-R2, on
	                           the RSPduo the port marked tuner 2, tuner 1's) */
	int      notch;         /* MIRISDR_NOTCH_*, RSP1B, RSPdx-R2 and RSPduo only */
	int      antenna;       /* MIRISDR_ANTENNA_*, B and C on the RSPdx-R2, and the RSPduo's tuner 0
	                           takes B, its Hi-Z input */
	int      direct_sampling; /* RSPdx-R2 only, MIRISDR_DS_*: when to sample the antenna directly,
	                           see below. The result says whether it is in force (0 or 1) */
	int      attenuator;    /* dB, pads before the tuner, apart from the gain's total. RSPduo
	                           and RSP1B: 0, 6, 13 or 19, one pad at a time (the RSPduo's not on
	                           the Hi-Z). RSPdx-R2: 0 to 36 in
	                           3 dB steps, its pads in series (31 at most at 1.5 GHz).
	                           mirisdr_get_frontend_caps() lists them */
	uint32_t flags;         /* MIRISDR_FRONTEND_*, the board's switches, see above */
	mirisdr_frontend_raw_t raw; /* words replacing the plan's, see above */
	uint32_t reserved[MIRISDR_RESERVED]; /* Not using _NESTED here as I think this could grow a lot */
} mirisdr_frontend_config_t;

/* Force specific tuner calibration codes instead of autodetection.
 * A held code is kept across retunes, so clear the holds when adjusting the frequency a lot.
 * mirisdr_get_tune() returns the holds in force. */
typedef struct mirisdr_tuner_override
{
	uint8_t  hold_vco;
	uint8_t  coarse;      /* VCO range 0-2 */
	uint8_t  fine;        /* VCO capacitor band, 0-31 */
	uint8_t  hold_upconv;
	uint8_t  upconv;      /* up-converter LO code, 0-15 */
	uint8_t  hold_lna;
	uint8_t  lna_cal;     /* L-band LNA code, 0-15 */
	uint8_t  hold_unknown; /* the status reads the search register instead */
	uint8_t  unknown;     /* 0-31, higher is a lower frequency. With the VCO held, above
	                         about 20 the loop nears the end of its range and ~29 loses lock */
	uint8_t  hold_filter; /* the status still reads the calibrated code */
	uint8_t  filter;      /* IF filter code, 0-31, 0 the widest. With the
	                         bandwidth at MIRISDR_BW_MAX is this held at 0 */
	uint32_t reserved[MIRISDR_RESERVED_NESTED];
} mirisdr_tuner_override_t;

typedef struct mirisdr_tune_config
{
	uint32_t frequency;     /* Hz to receive */
	uint32_t bandwidth;     /* Hz: 200000, 300000 or 600000 with a 450 kHz IF, 600000 or 1536000
	                           with 1620 and 2048 kHz, 1536000, 5000000 to 8000000 (1 MHz steps)
	                           or 14000000 (no filter) with zero IF. 0: the library's, at a low IF
	                           the widest, at zero IF the narrowest at least as wide as the rate */
	uint32_t if_freq;       /* 0 (zero IF), 450000, 1620000 or 2048000 */
	int      low_if_auto;   /* low IF: put the LO if_freq above frequency */
	int      iq;            /* MIRISDR_IQ_*: low IF only, the output not requested is switched off */
	mirisdr_gain_config_t gain;
	mirisdr_frontend_config_t frontend;
	mirisdr_tuner_override_t override; /* all holds off by default */
	uint32_t synth_thresh;  /* 0: the LO set to within 3 Hz. 1-4095: the synthesizer's fraction denominator
	                           fixed at this and no AFC, so the LO lands on a grid of 3 MHz (VHF),
	                           6 MHz (AM, band III), 24 MHz (band IV/V) or 48 MHz (L) divided by it
	                           and tunes in one band differ in register 2 only. For fast hopping or
                               better control of fractional spurious tones. */ 
	int      dc_removal;    /* MIRISDR_DC_*: under a baseband stream, the DC offset taken off
	                           in software (an estimate over 50 ms a converter, from each
	                           buffer's mean) */
	int      strict;        /* 0, the default: a value the hardware does not have exactly is taken
	                           to the nearest it has, and the result says what is in force: the
	                           bandwidth to the narrowest filter as wide (the widest if none is),
	                           the pads to the nearest step, a gain stage or total into its range
	                           and onto its steps, synth_thresh to 4095. A pad on the RSPduo's
	                           Hi-Z, which has none, is 0. 1: such a value is refused. Either way
	                           a part or mode the board does not have (a notch, an antenna, a
	                           flag, pads on a board without them, an IF, a combination it cannot
	                           do) is refused */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_tune_config_t;

#define MIRISDR_DC_AUTO         0       /* at a low IF, not at zero IF, where it is the band's centre */
#define MIRISDR_DC_ON           1       /* at zero IF too: a signal right at the LO goes with it */
#define MIRISDR_DC_OFF          2

typedef struct mirisdr_tune_result
{
	uint32_t lo;            /* Hz, the LO, what the VCO if outputting */
	int32_t  offset;        /* in a complex stream its signed offset from the LO,
                               in a single output its real IF */
	int      inverted;      /* single output: frequency up is IF down */
	int      iq;            /* the outputs running, MIRISDR_IQ_* */
	uint32_t bandwidth;     /* Hz in force */
	mirisdr_gain_config_t gain; /* in force: each stage and the total */
	mirisdr_frontend_config_t frontend; /* in force */
	mirisdr_band_t band;
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_tune_result_t;

MIRISDR_API void mirisdr_tune_config_default (mirisdr_tune_config_t *cfg); /* extra */

/* The tuners a receiver has: 2 on the RSPduo, 1 on the others. The tune calls take the
 * tuner's number, from 0, and each tuner keeps its own tune. The single setters
 * (mirisdr_set_center_freq() and the rest) act on the first tuner the stream takes */
MIRISDR_API int mirisdr_get_tuner_count (mirisdr_dev_t *p); /* extra */
MIRISDR_API int mirisdr_tune (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *cfg,
                              mirisdr_tune_result_t *res); /* extra */
MIRISDR_API int mirisdr_tune_check (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *cfg,
                                    mirisdr_tune_result_t *res); /* extra */
MIRISDR_API int mirisdr_get_tune (mirisdr_dev_t *p, int tuner, mirisdr_tune_config_t *cfg,
                                  mirisdr_tune_result_t *res); /* extra */

/* The stored tune with these overrides, NULL clears them all */
MIRISDR_API int mirisdr_set_tuner_override (mirisdr_dev_t *p, int tuner,
                                            const mirisdr_tuner_override_t *ov); /* extra */

/* not implemented yet */
MIRISDR_API int mirisdr_set_freq_correction (mirisdr_dev_t *p, int ppm);

/* Direct sampling, RSPdx-R2 only (refused elsewhere). frontend.direct_sampling says when a
 * tune takes the antenna past the tuner, through the 2 MHz low pass, straight to the I
 * converter: MIRISDR_DS_ON at MIRISDR_DS_MAX_HZ and below, MIRISDR_DS_AUTO when a baseband
 * stream's band (the frequency + rate / 2) fits under it (without baseband the frequency
 * alone), otherwise the tuner as usual. Under AUTO a new rate whose band does not fit
 * moves the tune to the tuner, and back. The tune result's frontend.direct_sampling says
 * which. In force it is like a single output (I)
 * whose IF is the frequency, the right way round: a stream following the tune switches
 * to I, and a baseband one puts the frequency at 0 Hz, its converters at 2.4 x the low
 * pass's edge whatever the frequency, so a retune moves only the NCO, no restart. The
 * 500 kHz low pass is taken when a baseband stream's band fits under it. The gain is
 * then the low pass's amplifier alone, about 37 dB: gain.lna in STAGES, a total from
 * 19 dB. The tuner's IF, bandwidth and iq are kept for above. Not in scans.
 * mirisdr_set_direct_sampling(): 0 off, 1 on (the I converter), as librtlsdr's, or
 * MIRISDR_DS_AUTO. The 2 of librtlsdr, the Q converter, is refused. */
#define MIRISDR_DS_OFF          0
#define MIRISDR_DS_ON           1
#define MIRISDR_DS_AUTO         3
#define MIRISDR_DS_MAX_HZ       2000000
#define MIRISDR_DS_LNA_DB       37
MIRISDR_API int mirisdr_set_direct_sampling (mirisdr_dev_t *p, int on);
MIRISDR_API int mirisdr_get_direct_sampling (mirisdr_dev_t *p);  /* extra: MIRISDR_DS_* as set, -1 on error */

/* transfer */
MIRISDR_API int mirisdr_set_transfer (mirisdr_dev_t *p, const char *v);       /* extra */
MIRISDR_API const char *mirisdr_get_transfer (mirisdr_dev_t *p);        /* extra */

/* gain */
MIRISDR_API int mirisdr_set_gain (mirisdr_dev_t *p);                    /* extra */
MIRISDR_API int mirisdr_get_tuner_gains (mirisdr_dev_t *dev, int *gains);
MIRISDR_API int mirisdr_set_tuner_gain (mirisdr_dev_t *p, int gain);
MIRISDR_API int mirisdr_get_tuner_gain (mirisdr_dev_t *p);              /* extra */
MIRISDR_API int mirisdr_set_tuner_gain_mode (mirisdr_dev_t *p, int mode);
MIRISDR_API int mirisdr_get_tuner_gain_mode (mirisdr_dev_t *p);         /* extra */
MIRISDR_API int mirisdr_set_mixer_gain (mirisdr_dev_t *p, int gain);    /* extra */
MIRISDR_API int mirisdr_set_mixbuffer_gain (mirisdr_dev_t *p, int gain);/* extra */
MIRISDR_API int mirisdr_set_lna_gain (mirisdr_dev_t *p, int gain);      /* extra */
MIRISDR_API int mirisdr_set_baseband_gain (mirisdr_dev_t *p, int gain); /* extra */
MIRISDR_API int mirisdr_get_mixer_gain (mirisdr_dev_t *p);              /* extra */
MIRISDR_API int mirisdr_get_mixbuffer_gain (mirisdr_dev_t *p);          /* extra */
MIRISDR_API int mirisdr_get_lna_gain (mirisdr_dev_t *p);                /* extra */
MIRISDR_API int mirisdr_get_baseband_gain (mirisdr_dev_t *p);           /* extra */
/* the stream's first tuner's, as the other setters: on the RSPduo tuner 0's unless tuner 1
   streams alone, and only tuner 1 has the bias-T */
MIRISDR_API int mirisdr_set_bias (mirisdr_dev_t *p, int bias);          /* extra */
MIRISDR_API int mirisdr_get_bias (mirisdr_dev_t *p);                    /* extra */
/* The tune's frontend.attenuator, dB: RSP1B, RSPdx-R2 and RSPduo, see above */
MIRISDR_API int mirisdr_set_attenuator (mirisdr_dev_t *p, int db);      /* extra */
MIRISDR_API int mirisdr_get_attenuator (mirisdr_dev_t *p);              /* extra */
/* The tune's frontend.flags, MIRISDR_FRONTEND_*: the board's, see above */
MIRISDR_API int mirisdr_set_frontend_flags (mirisdr_dev_t *p, uint32_t flags); /* extra */
MIRISDR_API int mirisdr_get_frontend_flags (mirisdr_dev_t *p);          /* extra */

/* What a tuner's front end has, for a program to offer: a field 0 (or none listed) the
 * board does not have. It does not change while the device is open. What depends on the
 * frequency comes with a tune: the amplifier's gain in the band (in the result's total,
 * and gain.amp_bypass whether it is in, and a tune_check with it bypassed gives its dB),
 * the Hi-Z only with the LO below 30 MHz. The pads attenuate less in L band (the
 * RSPdx-R2's 31 dB at most at 1.5 GHz), and the result reports their steps' dB.
 * mirisdr_tune_check() says what a tune would get. -1 for a tuner the board does not have */
#define MIRISDR_ATTENUATOR_MAX  16

typedef struct mirisdr_frontend_caps
{
	int      notches;           /* MIRISDR_NOTCH_* the board has */
	uint32_t antennas;          /* bit n: frontend.antenna n (MIRISDR_ANTENNA_*) taken */
	int      antenna_hiz;       /* the antenna that is a Hi-Z input, -1 none */
	int      bias;              /* a bias-T on this tuner's input (plain boards take frontend.bias
	                               as libmirisdr did, on GPIO_3, but have none known) */
	int      direct_sampling;   /* frontend.direct_sampling taken */
	uint32_t amp_bypass_hz;     /* gain.amp_bypass from this frequency up, 0 no amplifier */
	int      amp_max_db;        /* the amplifier's gain in its best band */
	int      attenuator_count;  /* the values frontend.attenuator takes, 1 (0 dB) without pads */
	int      attenuator[MIRISDR_ATTENUATOR_MAX]; /* dB, from 0 up */
	uint32_t flags;             /* MIRISDR_FRONTEND_* the board has */
	uint32_t raw_parts;         /* MIRISDR_FRONTEND_RAW_* the board has */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_frontend_caps_t;

MIRISDR_API int mirisdr_get_frontend_caps (mirisdr_dev_t *p, int tuner, mirisdr_frontend_caps_t *caps); /* extra */

/* The single setters for the tune's frontend. Switchable notch filters, RSP1B, RSPdx-R2 and RSPduo only,
 * enabling these filters can improve or reduce signal quality depending on your
 * specific scenario, so probably just have to try. */
#define MIRISDR_NOTCH_FM        0x01    /* 85-100 MHz */
#define MIRISDR_NOTCH_DAB       0x02    /* 165-230 MHz */
#define MIRISDR_NOTCH_MW        0x04    /* 0.4-1.6 MHz.*/
MIRISDR_API int mirisdr_set_notch (mirisdr_dev_t *p, int notches);      /* extra */
MIRISDR_API int mirisdr_get_notch (mirisdr_dev_t *p);                   /* extra */

/* The antenna input. Other boards have only A, but for the RSPduo's Hi-Z input on tuner 0
   (with the LO below 30 MHz) */
#define MIRISDR_ANTENNA_A       0
#define MIRISDR_ANTENNA_B       1
#define MIRISDR_ANTENNA_C       2
#define MIRISDR_ANTENNA_HIZ     MIRISDR_ANTENNA_B
MIRISDR_API int mirisdr_set_antenna (mirisdr_dev_t *p, int antenna);    /* extra */
MIRISDR_API int mirisdr_get_antenna (mirisdr_dev_t *p);                 /* extra */

/* GPIO control */
#define MIRISDR_GPIO_COUNT      4

MIRISDR_API int mirisdr_set_gpio_direction (mirisdr_dev_t *p, unsigned int pin, int output); /* extra */
MIRISDR_API int mirisdr_get_gpio_direction (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_set_gpio_output (mirisdr_dev_t *p, unsigned int pin, int high); /* extra */
MIRISDR_API int mirisdr_get_gpio_output (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_get_gpio_input (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_release_gpio (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_set_gpio_outputs (mirisdr_dev_t *p, unsigned int mask, unsigned int levels); /* extra */
MIRISDR_API int mirisdr_get_gpio_inputs (mirisdr_dev_t *p); /* extra */

#define MIRISDR_FW_SERIAL_MAX   12

/* mirisdr_fw_get() sets fields to what the image has, mirisdr_fw_patch() applies only
 * what is set. A serial is at most MIRISDR_FW_SERIAL_MAX characters. An empty one leaves
 * the image declaring none */
#define MIRISDR_FW_PATCH_IDS    (1u << 0)
#define MIRISDR_FW_PATCH_SERIAL (1u << 1)

typedef struct mirisdr_fw_patch
{
	unsigned int   fields;
	uint16_t       vid;
	uint16_t       pid;
	char           serial[MIRISDR_FW_SERIAL_MAX + 1];
	uint32_t reserved[MIRISDR_RESERVED_NESTED];
} mirisdr_fw_patch_t;

MIRISDR_API int mirisdr_fw_get (const uint8_t *image, uint32_t size, mirisdr_fw_patch_t *out); /* extra */
MIRISDR_API int mirisdr_fw_patch (uint8_t *image, uint32_t size, const mirisdr_fw_patch_t *p); /* extra */

/* Open a device with options, including the firmware it should be running.
 * The image comes from a buffer or from a path. The image can be patched
 * at runtime to ensure the VID/PID/Serial do not change. MIRISDR_FW_IDS_DEVICE
 * and _SET patch the image's BVDB block, so an image without one (another
 * firmware) is refused there: load it with MIRISDR_FW_IDS_IMAGE. */
#define MIRISDR_FW_IDS_IMAGE    0
#define MIRISDR_FW_IDS_DEVICE   1
#define MIRISDR_FW_IDS_SET      2

typedef struct mirisdr_open_config
{
	uint32_t       index;           /* when opening by index */
	const char    *serial;          /* or by serial or port ("1:3"), which wins over the index,
	                                   matched in the device list it is opened from */
	const char    *usb_port;        /* or by port ("1:3"). If serial also set, the device in that
	                                   port only when none has the serial, and only if it
	                                   declares no serial (as from ROM) */
	int            fd;              /* or an already open descriptor, -1 for none */

	const uint8_t *firmware;        /* the image, or NULL */
	uint32_t       firmware_size;
	const char    *firmware_path;   /* a file to read it from, which wins over
	                                 * the buffer above. */
	int            firmware_ids;    /* one of MIRISDR_FW_IDS_* */
	mirisdr_fw_patch_t firmware_patch; /* what MIRISDR_FW_IDS_SET writes in */
	int            keep_running;    /* use running fw */
	int            external_tuner; /* the board's tuner is not an MSi001, the library
                                    * will not try to control it when set */
	uint8_t        gpio_input_mask; /* pins to hold as inputs from before the first
                                     * register 8 write, one bit per pin. The EEPROM
                                     * drives GPIO 1 and 2 (3 too on the SDRplay boards):
                                     * holding one, EEPROM access and the reboot past it
                                     * are refused */
	int            hw_flavour;      /* the board, a mirisdr_hw_flavour_t. Settings this wrong
                                     * could in theory cause damage (eg bias-T on).
	                                 * MIRISDR_HW_AUTO by default */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_open_config_t;

/* Please fill in the open_config struct using mirisdr_open_config_default(&cfg) and only
 * the edit the options you need. You will likely only need to change index or fd. */

/* The firmware built into this library, which the default config loads */
MIRISDR_API const uint8_t *mirisdr_default_firmware (uint32_t *size); /* extra */
MIRISDR_API void mirisdr_open_config_default (mirisdr_open_config_t *cfg); /* extra */
MIRISDR_API int mirisdr_open_ex (mirisdr_dev_t **p, const mirisdr_open_config_t *cfg); /* extra */

/* If a function returns this value, the USB handle is invalid, and the device needs to be reopened.
 * Currently this can only happens when opening by fd or calling mirisdr_reboot(). */
#define MIRISDR_REOPEN          (-2)

#define MIRISDR_FW_BLOCK        0x0040
#define MIRISDR_FW_ID_LEN       8

MIRISDR_API int mirisdr_get_fw_id (mirisdr_dev_t *p, uint8_t *buf, int len); /* extra */

MIRISDR_API int mirisdr_running_from_rom (mirisdr_dev_t *p);            /* extra */

/* Read and write the device registers. Reading is safe, but writing the wrong
 * bits can cause the device to hang and require a power cycle. */
MIRISDR_API int mirisdr_write_reg (mirisdr_dev_t *p, uint8_t reg, uint32_t val); /* extra */
MIRISDR_API int mirisdr_read_reg (mirisdr_dev_t *p, uint8_t index, uint8_t *buf, int len); /* extra */

/* These functions allow the host to read and write device memory. The source (read) and
 * remap (write) arguments allow access to certain internal DSP and USB memories, assuming a
 * compatible firmware is loaded. Remap is ignored on the standard firmware and can cause
 * data corruption. With our firmware a read of up to 64 bytes is one snapshot. */
#define MIRISDR_MEM_XDATA       0 /* xdata */
#define MIRISDR_MEM_REMAP       1 /* xdata with the DSP and USB memories mapped in */
#define MIRISDR_MEM_IRAM        2 /* internal RAM, 0x00-0xFF */
MIRISDR_API int mirisdr_read_mem (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len, int source); /* extra */
MIRISDR_API int mirisdr_write_mem (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len, int remap); /* extra */

#define MIRISDR_BOOT_ROM        0
#define MIRISDR_BOOT_RAM        1
#define MIRISDR_BOOT_IGNORE_EEPROM 2 /* attempt to skip loading firmware or configuration from EEPROM */
MIRISDR_API int mirisdr_reboot (mirisdr_dev_t *p, int mode); /* extra, returns MIRISDR_REOPEN */

/* Call code on the device. The registers go in and the ones the callee left come
 * back, in the same struct. Write the code somewhere unused with mirisdr_write_mem()
 * first. Needs a firmware that supports it */
#define MIRISDR_CALL_CTX        0x1FF8
typedef struct mirisdr_call_regs
{
	uint8_t  a;
	uint8_t  b;
	uint16_t dptr;
	uint8_t  r0;
	uint8_t  r1;
} mirisdr_call_regs_t;

MIRISDR_API int mirisdr_call (mirisdr_dev_t *p, uint16_t addr, mirisdr_call_regs_t *regs); /* extra */

/* Read and write the eeprom */
MIRISDR_API int mirisdr_read_eeprom (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len); /* extra */
MIRISDR_API int mirisdr_write_eeprom (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len); /* extra */

/* Returns the maximum EEPROM size for the strapping:
 * 0=no response, 512=9-bit, 65536=16-bit */
MIRISDR_API int mirisdr_eeprom_size (mirisdr_dev_t *p); /* extra */

/* A pulse per second on GPIO_0, timestamped in the stream's own samples. Needs
 * our firmware and a running stream: enabling takes an anchor, which costs one
 * damaged packet. `edges` advances once per pulse and wraps at 256 */
typedef struct mirisdr_pps
{
	uint64_t sample;    /* count at low->high edge, in mirisdr_stream_stats_t.index units */
	uint8_t  edges;
	int      trusted;   /* nothing was disturbing this capture, ignore sample if 0.
	                       Self repairs.  What was, if anything, is in guard below */
	uint8_t  guard;     /* why, if trusted is 0: see MIRISDR_PPS_GUARD_* below */
	int      gapless;   /* nothing lost since the anchor, ignore sample if 0. Self repairs */
	uint16_t frame;     /* USB frame the edge fell in, 11 bits, wraps every 2.048 s */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_pps_t;

/* This field explains the reason why a sample is untrustworthy. Mostly for debug. */
#define MIRISDR_PPS_GUARD_NONE      0
#define MIRISDR_PPS_GUARD_USB       2       /* a USB interrupt ran within the
                                               two packet periods before the
                                               capture, or between the
                                               streaming interrupt and the
                                               interval's first SOF */
#define MIRISDR_PPS_GUARD_BASE      0xFB    /* the interval opened on the poll
                                               loop's carry */
#define MIRISDR_PPS_GUARD_TAIL      0xFC    /* the interval's first SOF fell
                                               inside the streaming interrupt */
#define MIRISDR_PPS_GUARD_STRADDLE  0xFD    /* a streaming interrupt opened
                                               this interval while a SOF was
                                               between its edge and its latch */
#define MIRISDR_PPS_GUARD_CARRY     0xFE    /* the device's counter was mid
                                               carry */
#define MIRISDR_PPS_GUARD_HELD      0xFF    /* a bit banged UART or I2C
                                               transfer was running */

MIRISDR_API int mirisdr_enable_pps (mirisdr_dev_t *p, int run); /* extra */
/* Take the edges from the USB start of frame instead of GPIO_0, syncing
 * to the USB host clock. The divider divides the 2ms pulse rate (1-255).
 * Applies on mirisdr_enable_pps. */
MIRISDR_API int mirisdr_set_pps_source (mirisdr_dev_t *p, int sof, int divider); /* extra */
MIRISDR_API int mirisdr_get_pps (mirisdr_dev_t *p, mirisdr_pps_t *out); /* extra */

/* I2C master on GPIO_1 (SDA) and GPIO_2 (SCL). A read or write is at most 64 bytes
 * (the firmware's buffer). The result is 0, MIRISDR_I2C_NACK, or -1 for a USB failure */
#define MIRISDR_I2C_NO_STOP     0x01
#define MIRISDR_I2C_REPEAT      0x02
#define MIRISDR_I2C_NACK        1
MIRISDR_API int mirisdr_set_i2c_rate (mirisdr_dev_t *p, uint32_t hz); /* extra, 50 kHz until set */
MIRISDR_API int mirisdr_i2c_write (mirisdr_dev_t *p, uint8_t addr, unsigned int flags, const uint8_t *buf, int len); /* extra */
MIRISDR_API int mirisdr_i2c_read (mirisdr_dev_t *p, uint8_t addr, unsigned int flags, uint8_t *buf, int len); /* extra */
MIRISDR_API int mirisdr_i2c_transfer (mirisdr_dev_t *p, uint8_t addr, const uint8_t *out, int outlen, uint8_t *in, int inlen); /* extra */
MIRISDR_API int mirisdr_i2c_recover (mirisdr_dev_t *p); /* extra */

/* Send UART data via GPIO_2 */
MIRISDR_API int mirisdr_uart_write (mirisdr_dev_t *p, uint32_t baud, const uint8_t *buf, int len); /* extra */

/* The MSI2500 has a remote control receiver on GPIO3, this function allows to configure a
 * callback that receives IR events. If you want to sample a non-pulse based protocol, 
 * such as UART, it makes sense to set tick_ns to 1 and use only the level field */
#define MIRISDR_IR_GPIO         3
#define MIRISDR_IR_KEEPS_GOING  127

typedef struct mirisdr_ir_pulse
{
	uint32_t index;     /* the sample counter for this packet */
	uint32_t duration;  /* the run's length if it ended here, or its current duration if not (microseconds) */
	uint8_t  level;     /* high=1 */
	uint8_t  ended;     /* set when this is the end of the pulse */
	uint8_t  partial;   /* duration is a floor due to missing packets */
	uint8_t  ticks;     /* raw count as received from the device (127=ongoing) */
} mirisdr_ir_pulse_t;

typedef void (*mirisdr_ir_cb_t) (const mirisdr_ir_pulse_t *pulse, void *ctx);

MIRISDR_API int mirisdr_set_ir (mirisdr_dev_t *p, uint32_t tick_ns, mirisdr_ir_cb_t cb, void *ctx); /* extra */
MIRISDR_API uint32_t mirisdr_get_ir (mirisdr_dev_t *p);                 /* extra */

/* This function allows you to read the current state of a tuner (numbered as for
 * mirisdr_tune(), 1 too on the RSPduo). It is a measurement, so it stays apart from the
 * tune config: the codes held there read back as held. Afaik there is no
 * way to know if the PLL is locked, the HIGH and LOW limit flags only indicate
 * you are tuning near the edge of your chip, not that you try to tune beyond it. */
#define MIRISDR_TUNER_SYNTH_OFF      0x01   /* no VCO range selected */
#define MIRISDR_TUNER_AT_LOW_LIMIT   0x02   /* coarse 0, fine 31 */
#define MIRISDR_TUNER_AT_HIGH_LIMIT  0x04   /* coarse 2, fine 0 */

typedef struct mirisdr_tuner_status
{
	uint32_t raw;
	int8_t   coarse;     /* VCO range 0-2 from the one-hot bits, -1 when none is set */
	uint8_t  fine;       /* VCO capacitor band within the range, 5 bits */
	uint8_t  unknown;    /* a finer VCO frequency control the second search sets after the band,
	                        5 bits, higher is a lower frequency */
	uint8_t  upconv;     /* up-converter LO calibration, 4 bits */
	uint8_t  lna_cal;    /* L-band LNA calibration, 4 bits */
	uint8_t  filter;     /* IF filter calibration code, 5 bits: timed against the crystal
	                        XTALSEL sets. Higher is narrower */
	uint8_t  top;        /* bits 27:26, so far always 3 */
	uint8_t  flags;      /* MIRISDR_TUNER_* */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_tuner_status_t;

MIRISDR_API int mirisdr_get_tuner_status (mirisdr_dev_t *p, int tuner, mirisdr_tuner_status_t *st); /* extra */

/* Register lists, run by the firmware without host involvement: a tune, or a
 * scan that hops on stream interrupts. An entry with reg below 0x20
 * writes that register as mirisdr_write_reg() would. The commands below take a 16 bit val.
 * Up to 63 entries a bank. A list replaces the running one unless queued, n = 0 stops.
 * PPS pauses while a list runs, and register writes are refused: stop it first. */
#define MIRISDR_LIST_WAIT_US    0x80    /* busy wait, val microseconds */
#define MIRISDR_LIST_WAIT_IRQ   0x81    /* val stream interrupts, counted from the previous wait */
#define MIRISDR_LIST_REPEAT     0x82    /* back to the start, val more passes, 0 = forever */
#define MIRISDR_LIST_SWITCH     0x83    /* to the other bank, if it is queued */
#define MIRISDR_LIST_WAIT_SPI   0x84    /* the SPI master (registers 0x0B-0x0D) done, at most val
                                           polls of ~0.5 us, 0 = 65536; a timeout stops the list */
#define MIRISDR_LIST_QUEUE      0x01    /* flag: load behind the running list */

typedef struct mirisdr_list_entry {
    uint8_t reg;
    uint32_t val;
} mirisdr_list_entry_t;

typedef struct mirisdr_list_status {
    uint8_t running, waiting, queued, bank, pps_paused, spi_timeout, entry;
    uint16_t passes;
    uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_list_status_t;

MIRISDR_API int mirisdr_load_list (mirisdr_dev_t *p, int bank, int flags, const mirisdr_list_entry_t *e, int n); /* extra */
MIRISDR_API int mirisdr_get_list_status (mirisdr_dev_t *p, mirisdr_list_status_t *st); /* extra */

/*
 * Scanning. The firmware hops the tuner through a list of tunes on its own, one hop
 * every `dwell` stream interrupts, while the host only keeps the two list banks fed.
 *
 * mirisdr_scan_compile() turns the tunes into list entries, as the library would send
 * them, against the device's state (flavour, crystal, filter calibration, notches): the
 * receiver itself (or possibly the null device to work offline). Each hop is register 2
 * plus what changed since the hop before, then the wait. The first hop of a pass is written
 * in full, so the list can wrap. synth_thresh keeps registers 3 and 5 the same within a
 * band, so a hop there is two entries. A hop that is not held searches the VCO in the
 * tuner after its register 2, which only takes longer to settle. An L band hop must
 * hold its LNA code (see mirisdr_tune_learn()), as the calibration requires a second tune,
 * which would confuse the tune event output from the chip stream.
 *
 * The entries are cut into chunks on hop boundaries. Each chunk starts by flipping the
 * stream marker, so the stream says which chunk runs, and every hop toggles the tune
 * event once. mirisdr_scan_start() loads the first chunks and calls cb from the stream
 * (read_async) for every hop as it shows. mirisdr_scan_feed() keeps the banks fed:
 * call it often from another thread, at least once per chunk's run. Register writes
 * are refused while the scan runs and PPS pauses. The scan takes the stream marker
 * (mirisdr_set_stream_mark()) and leaves it cleared. mirisdr_scan_stop() ends it and
 * puts the tune from before back.
 *
 * Timing: a hop's register 2 goes out just after the interrupt that ends the wait
 * before it, which is the start of the packet before the one that toggles. Each wait
 * counts its own dwell interrupts of 1 (isochronous 1x1024) to 4 (bulk) packets, so a
 * hop that started late still gets them and the hops after it move. Entries before
 * register 2 (a band change, the expander, held codes) delay it by up to ~3 packets, and
 * the host loading the next chunk now and then delays a hop more (up to ~0.5 ms seen in
 * isochronous mode). The reports come from each hop's own toggle, so they correctly identify
 * the samples regardless of delays.
 * Measured on an RSP1B at 8 Msps: the tone moves ~27 us in and is within 2 kHz after
 * ~0.13-0.15 ms and ~0.09-0.13 ms with all codes held.
 */
typedef struct mirisdr_scan mirisdr_scan_t;

typedef struct mirisdr_scan_hop
{
	uint32_t chunk;          /* the chunk it runs in */
	uint16_t entries;        /* list entries it takes, its wait included */
	uint16_t pre_entries;    /* entries before its register 2 */
	uint16_t pre_words;      /* of which tuner words, ~5.6 us each */
	mirisdr_tune_result_t res;
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_scan_hop_t;

#define MIRISDR_SCAN_MISSED     0x01    /* packets were lost since the hop before */
#define MIRISDR_SCAN_RESTART    0x02    /* hopping lits underflow, the hop before ran longer */
#define MIRISDR_SCAN_LOST       0x04    /* tune events and hops disagree: the count resyncs at
                                           the next chunk */

typedef struct mirisdr_scan_report
{
	uint32_t hop;            /* index into the hops compiled */
	uint32_t pass;
	uint64_t sample;         /* where its register 2 went out: the start of the packet before
	                            the one that toggled, in the delivered count of
	                            mirisdr_get_buffer_info() */
	uint64_t index;          /* same as sample, but now lost samples counted */
	uint32_t length;         /* the dwell in samples. */
	uint8_t  flags;          /* MIRISDR_SCAN_* */
} mirisdr_scan_report_t;

typedef void (*mirisdr_scan_cb_t) (const mirisdr_scan_report_t *r, void *ctx);

typedef struct mirisdr_scan_status
{
	int      running;
	uint32_t pass;           /* of the chunk last loaded */
	uint64_t loaded;         /* chunks loaded */
	uint32_t restarts;       /* amount of underflows */
	uint32_t reserved[MIRISDR_RESERVED];
} mirisdr_scan_status_t;

/* Tune cfg with nothing held, read back what the tuner found and fill cfg->override
   with all of it held.
   Leaves the receiver tuned there, does not apply the holds. */
MIRISDR_API int mirisdr_tune_learn (mirisdr_dev_t *p, int tuner, mirisdr_tune_config_t *cfg); /* extra */

MIRISDR_API int mirisdr_scan_compile (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *hops,
                                      uint32_t n, uint16_t dwell, mirisdr_scan_t **scan); /* extra */
MIRISDR_API void mirisdr_scan_free (mirisdr_scan_t *scan); /* extra */
MIRISDR_API uint32_t mirisdr_scan_chunks (const mirisdr_scan_t *scan); /* extra */
MIRISDR_API int mirisdr_scan_chunk (const mirisdr_scan_t *scan, uint32_t chunk,
                                    const mirisdr_list_entry_t **e); /* extra, returns the count */
MIRISDR_API int mirisdr_scan_hop (const mirisdr_scan_t *scan, uint32_t hop, mirisdr_scan_hop_t *info); /* extra */

MIRISDR_API int mirisdr_scan_start (mirisdr_dev_t *p, const mirisdr_scan_t *scan, uint32_t passes,
                                    mirisdr_scan_cb_t cb, void *ctx); /* extra, passes 0 = forever */
MIRISDR_API int mirisdr_scan_feed (mirisdr_dev_t *p); /* extra: 1 running, 0 done, -1 failed */
MIRISDR_API int mirisdr_get_scan_status (mirisdr_dev_t *p, mirisdr_scan_status_t *st); /* extra */
MIRISDR_API int mirisdr_scan_stop (mirisdr_dev_t *p); /* extra */

/* Every packet header carries the gain the tuner is running and three bits that
 * toggle when a gain word, a synthesizer word or a sample rate write takes effect.
 * The callback runs for each packet where something changed, before that packet's
 * samples are delivered. The gain/tune/rate/mark flags are set on the next buffer
 * that is encoded by the chip after the action. */
#define MIRISDR_EVENT_GAIN      0x01    /* a gain word took effect */
#define MIRISDR_EVENT_TUNE      0x02    /* a synthesizer word took effect */
#define MIRISDR_EVENT_RATE      0x04    /* the sample rate register was written */
#define MIRISDR_EVENT_MARK      0x08    /* the marker set by mirisdr_set_stream_mark changed */
#define MIRISDR_EVENT_MISSED    0x80    /* packets were lost before this one */

typedef struct mirisdr_stream_event
{
	uint64_t sample;     /* samples delivered before this packet's first sample */
	uint64_t index;      /* the same point as an absolute index, lost samples included */
	uint8_t  events;     /* MIRISDR_EVENT_* */
	uint8_t  bb_gr;      /* gain in effect: baseband gain reduction, 6 bits */
	uint8_t  mixbu;      /* up-converter mixer gain reduction, 2 bits */
	uint8_t  mixl;       /* IQ mixer gain reduction */
	uint8_t  lna;        /* LNA gain reduction */
	uint8_t  mark;       /* the marker's level */
	uint8_t  saturate;   /* the output clips rather than wraps (register 7 bit 2) */
	uint16_t raw;        /* header bytes 4-5 */
} mirisdr_stream_event_t;

typedef void (*mirisdr_stream_event_cb_t) (const mirisdr_stream_event_t *ev, void *ctx);

MIRISDR_API int mirisdr_set_stream_events (mirisdr_dev_t *p, mirisdr_stream_event_cb_t cb, void *ctx); /* extra */
MIRISDR_API int mirisdr_set_stream_mark (mirisdr_dev_t *p, int on);     /* extra */

/*
 * DC Calibration
 *
 * Only raw interface for testing purposes. No sanity checks done.
 * raw format with bit length in parenthesis:
 *   [ unused (4) | period (12) | unused (6) | track (6) | speedup (1) | mode (3) ]
 * The default dc setting is:
 *   0x080001f2 (period = 0x800, track = 0x1f, speedup = 0, mode = 2)
 */
MIRISDR_API int mirisdr_set_dc_raw (mirisdr_dev_t *p, uint32_t raw);
MIRISDR_API uint32_t mirisdr_get_dc_raw (mirisdr_dev_t *p);

#ifdef __cplusplus
}
#endif

#endif /* __MIRISDR_H */
