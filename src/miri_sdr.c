/*
 * MiriSDR
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

#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined (_WIN32) || defined(__MINGW32__)
#include <unistd.h>
#else
#include <Windows.h>
#endif

#include "mirisdr.h"

#define DEFAULT_SAMPLE_RATE		2048000
#define DEFAULT_ASYNC_BUF_NUMBER	32
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

static int do_exit = 0;
static int write_failed = 0;
static mirisdr_dev_t *dev = NULL;

void usage(void)
{
#if defined (_WIN32) && !defined(__MINGW32__)
	fprintf(stderr,
		"Usage:\t miri_sdr.exe [device_index] [samplerate in kHz] "
		"[gain] [frequency in Hz] [filename]\n");
	#else
	fprintf(stderr,
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-m sample format (default: auto]\n"
		"\t    auto:   widest format possible\n"
		"\t    504:    S8 (fastest)\n"
		"\t    384:    S10 +2bits \n"
		"\t    336:    S12\n"
		"\t    252:    S14\n"
		"\t  real modes, one converter, every sample real:\n"
		"\t    autor:  widest real format possible\n"
		"\t    768r:   S10 +2bits\n"
		"\t    672r:   S12\n"
		"\t    504r:   S16\n"

#if !defined (_WIN32) || defined(__MINGW32__)
		"\t[-e USB transfer mode (default: ISOC, BULK with -S)]\n"
#else
		"\t[-e USB transfer mode (default: BULK)]\n"
#endif
		"\t    ISOC:   Isochronous, 3 x 1024 per microframe (24.576 MB/s)\n"
		"\t    ISOC2:  Isochronous, 2 x 1024 per microframe (16.384 MB/s)\n"
		"\t    ISOC1:  Isochronous, 1 x 1024 per microframe (8.192 MB/s)\n"
		"\t    BULK:   Bulk, not microframe limited\n"
		"\t    1 and 2 are also accepted, for ISOC and BULK\n"
		"\t[-i IF mode (default: ZERO]\n"
		"\t    0:       ZERO\n"
		"\t    450000:  450 kHz\n"
		"\t    1620000: 1620kHz\n"
		"\t    2048000: 2048kHz\n"
		"\t[-w BW mode (default: the widest the IF allows)]\n"
		"\t    200000: 200kHz, 450 kHz IF\n"
		"\t    300000: 300kHz, 450 kHz IF\n"
		"\t    600000: 600kHz, any low IF\n"
		"\t   1536000: 1536kHz, zero, 1620 or 2048 kHz IF\n"
		"\t   5000000: 5MHz, zero IF, 6, 7 and 8 MHz too\n"
		"\t   8000000: 8MHz, zero IF\n"
		"\t  14000000: 14MHz, zero IF, no filter\n"
		"\t    a combination the tuner cannot do is refused\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-B baseband: complex float (cf32), -f at 0 Hz at any IF]\n"
		"\t    a low IF gives 4 x IF halved 0 to 8 times, for example\n"
		"\t    -i 2048000 -s 256000, and a real -m uses one converter\n"
		"\t[-D decimation bypass (default: auto)]\n"
		"\t    auto:   bypass above 14.5 Msps, where the PLL runs out\n"
		"\t    on:     always bypass, 2.6 - 30 Msps\n"
		"\t    off:    never bypass, 1.3 - 15 Msps\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-T board (default: from the USB IDs)]\n"
		"\t    0:       a plain MSi2500/MSi001 board\n"
		"\t    1:       SDRplay RSP1\n"
		"\t    2:       SDRplay RSP1B\n"
		"\t    3:       SDRplay RSPdx-R2\n"
		"\t    4:       SDRplay RSPduo\n"
		"\t[-g total gain in 10 dB units: 4.5 is 45 dB, taken to the nearest the band has\n"
		"\t    (default: keep the gain set, there is no automatic gain)]\n"
        "\t[-G individual gains separated by comma (mixer, lna, mixbuffer, baseband)]\n"
        "\t    mixer: 0, 1\n"
        "\t    LNA: 0, 1\n"
        "\t    mixbuffer: 0, 6, 12, 18, 24\n"
        "\t    baseband: 0 - 59\n"
		"\t[-b output_block_size (default: 16 * 16384, with -B as they come)]\n"
		"\t[-q swap I/Q (default: off)]\n"
		"\t    in the real modes this chooses which converter is digitised\n"
		"\t[-S force sync output (default: async), not with -B; needs BULK]\n"
		"\tfilename (a '-' dumps samples to stdout)\n\n");
#endif
	exit(1);
}

#if defined (_WIN32) && !defined(__MINGW32__)
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		mirisdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
/* No output here: with SIGPIPE from stderr it would raise the signal again for ever */
static void sighandler(int signum)
{
(void) signum;
	do_exit = 1;
	mirisdr_cancel_async(dev);
}
#endif

static void mirisdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	mirisdr_stream_stats_t st;
	mirisdr_stream_result_t sr;
	static uint64_t mark;

	/* a signal before the stream ran was not a cancel */
	if (do_exit) {
		mirisdr_cancel_async(dev);
		return;
	}

	if (ctx) {
		if (fwrite(buf, 1, len, (FILE*)ctx) != len) {
			fprintf(stderr, "Short write, samples lost, exiting!\n");
			write_failed = 1;
			mirisdr_cancel_async(dev);
		}
	}

	/* the stats count the converters' samples, at their rate */
	if (mirisdr_get_stream_stats(dev, &st) == 0 && mirisdr_get_stream(dev, NULL, &sr) == 0) {
		if (st.index < mark) mark = 0;
		if (st.index - mark >= sr.adc_rate) {
			mark = st.index;
			fprintf(stderr, "%.3f Ms delivered, %llu lost in %llu gaps, %llu jitter, %llu resync\n",
				st.samples / 1e6, (unsigned long long) st.lost,
				(unsigned long long) st.gaps, (unsigned long long) st.jitter,
				(unsigned long long) st.resyncs);
		}
	}
}

int main(int argc, char **argv)
{
#if !defined (_WIN32) || defined(__MINGW32__)
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
    int gain_mixer = 0, gain_lna = 0, gain_mb = 0, gain_bb = 0;
    int gain_one = 1;
	int sync_mode = 0;
	int baseband = 0;
	int block_given = 0;
	int transfer_given = 0;
	int swap_iq = 0;
	FILE *file;
	uint8_t *buffer;
	uint32_t format = 0;
	const char *decimation = "AUTO";
#if !defined (_WIN32) || defined(__MINGW32__)
	const char *transfer_name = "ISOC";
#else
	const char *transfer_name = "BULK";
#endif
	uint32_t if_mode = 0;
	uint32_t bw = 0;
	static const char *format_names[] = { "AUTO", "504_S8", "384_S16", "336_S16", "252_S16",
	                                      "768_REAL_S16", "672_REAL_S16", "504_REAL_S16", "AUTO_REAL" };
	mirisdr_stream_config_t sc;
	mirisdr_stream_result_t sr;
	mirisdr_tune_config_t tc;
	mirisdr_tune_result_t tr;
	uint32_t dev_index = 0;
	uint32_t frequency = 100000000;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    uint32_t device_count;
	char vendor[256] = { 0 }, product[256] = { 0 }, serial[256] = { 0 };
	mirisdr_hw_flavour_t hw_flavour = MIRISDR_HW_AUTO;
	int intval;

#if !defined (_WIN32) || defined(__MINGW32__)
	while ((opt = getopt(argc, argv, "b:Bd:D:T:e:f:g:G:i:m:qs:w:S::")) != -1) {
		switch (opt) {
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			block_given = 1;
			break;
		case 'B':
			baseband = 1;
			break;
		case 'd':
			dev_index = atoi(optarg);
			break;
		case 'D':
			for (decimation = optarg; *optarg; optarg++)
				*optarg = toupper((unsigned char) *optarg);
			break;
        case 'T':
            intval = atoi(optarg);
            if ((intval < 0) || (intval > MIRISDR_HW_RSPDUO))
            {
                fprintf(stderr, "-T takes 0 to %d.\n", MIRISDR_HW_RSPDUO);
                exit(2);
            }
            hw_flavour = (mirisdr_hw_flavour_t) intval;
            break;
		case 'e':
			transfer_given = 1;
			if ((strcmp("ISOC", optarg) == 0) ||
			    (strcmp("1", optarg) == 0)) {
				transfer_name = "ISOC";}
			else if ((strcmp("BULK", optarg) == 0) ||
			    (strcmp("2", optarg) == 0)) {
				transfer_name = "BULK";}
			else {
				transfer_name = optarg;}
			break;
		case 'f':
			frequency = (uint32_t)atof(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10 + 0.5); /* dB */
			break;
        case 'G':
            // mixer lna mb bb
            gain_one = 0;
            char* part;
            if (!(part = strtok(optarg, ","))) {
                goto invalid;
            }
            gain_mixer = atoi(part);
            if (!(part = strtok(NULL, ","))) {
                goto invalid;
            }
            gain_lna = atoi(part);
            if (!(part = strtok(NULL, ","))) {
                goto invalid;
            }
            gain_mb = atoi(part);
            if (!(part = strtok(NULL, ","))) {
                goto invalid;
            }
            gain_bb = atoi(part);
            break;
            invalid:
                fprintf(stderr, "-G needs 4 gains (mixer, lna, mb, bb)!\n");
                exit(2);
		case 'i':
			if_mode = atoi(optarg);
			break;
		case 'm':
			if (strcmp("504", optarg) == 0) {
				format = 1;}
			if (strcmp("384", optarg) == 0) {
				format = 2;}
			if (strcmp("336", optarg) == 0) {
				format = 3;}
			if (strcmp("252", optarg) == 0) {
				format = 4;}
			if (strcmp("768r", optarg) == 0) {
				format = 5;}
			if (strcmp("672r", optarg) == 0) {
				format = 6;}
			if (strcmp("504r", optarg) == 0) {
				format = 7;}
			if (strcmp("autor", optarg) == 0) {
				format = 8;}
			if ((strcmp("auto", optarg) != 0) && !format) {
				fprintf(stderr, "-m %s is not a format.\n", optarg);
				exit(2);
			}
			break;
		case 'q':
			swap_iq = 1;
			break;
		case 's':
			samp_rate = (uint32_t)atof(optarg);
			break;
		case 'w':
			bw = atoi(optarg);
			break;
		case 'S':
			sync_mode = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}
#else
	if(argc <6)
		usage();
	dev_index = atoi(argv[1]);
	samp_rate = atoi(argv[2])*1000;
	gain=(int)(atof(argv[3]) * 10 + 0.5); /* dB */
	frequency = atoi(argv[4]);
	filename = argv[5];
#endif
	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

	if (baseband && sync_mode) {
		fprintf(stderr, "Baseband is async only, -B and -S do not go together.\n");
		exit(1);
	}

	/* the synchronous reads are bulk only */
	if (sync_mode && !transfer_given)
		transfer_name = "BULK";
	if (sync_mode && strcmp(transfer_name, "BULK")) {
		fprintf(stderr, "-S needs the BULK transfer.\n");
		exit(1);
	}

	if (!(buffer = malloc(out_block_size * sizeof(uint8_t)))) {
		fprintf(stderr, "Out of memory.\n");
		exit(1);
	}

	device_count = mirisdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		exit(1);
	}

	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (uint32_t i = 0; i < device_count; i++) {
		mirisdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");

	fprintf(stderr, "Using device %d: %s\n",
		dev_index, mirisdr_get_device_name(dev_index));

	{
		mirisdr_open_config_t cfg;

		mirisdr_open_config_default(&cfg);
		cfg.index = dev_index;
		cfg.hw_flavour = hw_flavour;
		r = mirisdr_open_ex(&dev, &cfg);
	}
	if (r < 0) {
		fprintf(stderr, "Failed to open mirisdr device #%d.\n", dev_index);
		exit(1);
	}


#if !defined (_WIN32) || defined(__MINGW32__)
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	r = mirisdr_get_usb_strings(dev, vendor, product, serial);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to read usb strings.\n");
	else
		fprintf(stderr, "%s, %s: SN: %s\n", vendor, product, serial);

	/* The tune first, as its IF sets the rates a baseband stream can have. In one
	   call, the gain with it: -g as a total the library splits,
	   -G stage by stage; there is no automatic gain, so -g 0 keeps what is set */
	mirisdr_get_tune(dev, 0, &tc, NULL);
	tc.frequency = frequency;
	tc.if_freq = if_mode;
	tc.bandwidth = bw;
	/* baseband: the LO goes the IF above -f, as the stream will want. With a real
	   format one converter, -q picks Q */
	tc.low_if_auto = baseband;
	if (baseband && format >= 5)
		tc.iq = swap_iq ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I;
	if (!gain_one) {
		tc.gain.mode = MIRISDR_GAIN_STAGES;
		tc.gain.mixer = gain_mixer;
		tc.gain.lna = gain_lna;
		tc.gain.mixbuffer = gain_mb;
		tc.gain.baseband = gain_bb;
	} else if (gain) {
		tc.gain.mode = MIRISDR_GAIN_TOTAL;
		tc.gain.total = gain;
	} else {
		fprintf(stderr, "No automatic gain, keeping the gain set.\n");
	}
	if (mirisdr_tune(dev, 0, &tc, &tr) < 0) {
		fprintf(stderr, "Failed to tune.\n");
		exit(1);
	}
	fprintf(stderr, "Tuned to %u Hz (LO %u Hz).\n", frequency, tr.lo);
	fprintf(stderr, "Tuner gain %d dB: LNA %s, mixer %s, mixbuffer %d dB, baseband %d dB.\n", tr.gain.total,
	        tr.gain.lna ? "on" : "off", tr.gain.mixer ? "on" : "off", tr.gain.mixbuffer, tr.gain.baseband);

	/* The stream in one call: an invalid combination is refused, not adjusted. The
	   real modes digitise one converter, so swap_iq picks which one; in the complex
	   modes it swaps I and Q. */
	mirisdr_get_stream(dev, &sc, NULL);
	sc.rate = samp_rate;
	sc.format = format_names[format];
	/* baseband: the tune's converters decide, a real format is the one for a single
	   converter, and -q mirrors the spectrum with both */
	sc.baseband = baseband;
	if (baseband && format >= 5) {
		sc.format = NULL;
		sc.format_single = format_names[format];
		swap_iq = 0;
	}
	sc.transfer = transfer_name;
	sc.decimation_bypass = decimation;
	sc.swap_iq = swap_iq;
	if (mirisdr_set_stream(dev, &sc, &sr) < 0) {
		fprintf(stderr, "Failed to set up the stream.\n");
		exit(1);
	}
	samp_rate = sr.rate;
	fprintf(stderr, "Sample rate is set to %u Hz.\n", samp_rate);
	/* the filter the library picks follows the rate */
	mirisdr_get_tune(dev, 0, NULL, &tr);
	fprintf(stderr, "Tuner filter is %u Hz.\n", tr.bandwidth);
	if (baseband) {
		fprintf(stderr, "Baseband, complex float: %s, converters at %u Hz, LO %u Hz.\n",
		        sr.baseband == MIRISDR_BASEBAND_REAL ? "one converter" :
		        sr.baseband == MIRISDR_BASEBAND_COMPLEX ? "both converters" : "zero IF",
		        sr.adc_rate, tr.lo);
	}
	if (sr.adc_clock > MIRISDR_ADC_CLOCK_BEST)
		fprintf(stderr, "The converters clock at %.1f MHz, past their best %.0f MHz: their resolution drops.\n",
		        sr.adc_clock / 1e6, MIRISDR_ADC_CLOCK_BEST / 1e6);
	fprintf(stderr, "I/Q swap is %s.\n", sc.swap_iq ? "on" : "off");
	fprintf(stderr, "Transfer mode is %s.\n", mirisdr_get_transfer(dev));
	fprintf(stderr, "Sample format is %s", mirisdr_get_sample_format(dev));
	if (strncmp(mirisdr_get_sample_format(dev), "AUTO", 4) == 0)
		fprintf(stderr, " (%s)", sr.format);
	fprintf(stderr, ".\n");

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			r = -1;
			goto out;
		}
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	r = mirisdr_reset_buffer(dev);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");

	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		while (!do_exit) {
			r = mirisdr_read_sync(dev, buffer, (int) out_block_size, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if (fwrite(buffer, 1, n_read, file) != (size_t)n_read) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				r = -1;
				break;
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				r = -1;
				break;
			}
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		/* baseband rates can be low: without -b the buffers go out as they come */
		r = mirisdr_read_async(dev, mirisdr_callback, (void *)file, DEFAULT_ASYNC_BUF_NUMBER,
				      (baseband && !block_given) ? 0 : out_block_size);
	}

	if (do_exit && (r >= 0))
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else if (r >= 0)
		fprintf(stderr, "\nThe stream ended, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	/* a cancel in the callback after a short write ends the stream without an error */
	if (write_failed)
		r = -1;

	if (file != stdout)
		fclose(file);

out:
	mirisdr_close(dev);
	free (buffer);
	return (r < 0) ? 1 : 0;
}
