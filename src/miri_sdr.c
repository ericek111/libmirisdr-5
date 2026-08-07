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

#define DEFAULT_SAMPLE_RATE		500000
#define DEFAULT_ASYNC_BUF_NUMBER	32
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

static int do_exit = 0;
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
		"\t    504:    S8 (fastest)\n"
		"\t    384:    S10 +2bits \n"
		"\t    336:    S12\n"
		"\t    252:    S14\n"
#if !defined (_WIN32) || defined(__MINGW32__)
		"\t[-e USB transfer mode (default: 1)]\n"
#else
		"\t[-e USB transfer mode (default: 2)]\n"
#endif
		"\t    1:      Isochronous (maximum 196.608 Mbit/s) \n"
		"\t    2:      Bulk (maximum 333Mbit/s, depends on controller type)\n"
		"\t[-i IF mode (default: ZERO]\n"
		"\t    0:       ZERO\n"
		"\t    450000:  450 kHz\n"
		"\t    1620000: 1620kHz\n"
		"\t    2048000: 2048kHz\n"
		"\t[-w BW mode (default: 8MHz]\n"
		"\t    200000: 200kHz\n"
		"\t    300000: 300kHz\n"
		"\t    600000: 600kHz\n"
		"\t   1536000: 1536kHz\n"
		"\t   5000000: 5MHz\n"
		"\t   6000000: 6MHz\n"
		"\t   7000000: 7MHz\n"
		"\t   8000000: 8MHz\n"
        "\t  14000000: 14MHz\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-d device_index (default: 0)]\n"
	    "\t[-T device_type device variant (default: 0)]\n"
        "\t    0:       Default\n"
        "\t    1:       SDRPlay\n"
		"\t[-g gain in dB (0 for auto; max is band dependent, 102 in VHF/Band III)]\n"
        "\t[-G individual gains separated by comma (mixer, lna, mixbuffer, baseband)]\n"
        "\t    mixer: 0, 1\n"
        "\t    LNA: 0, 1\n"
        "\t    mixbuffer: 0, 6, 12, 18, 24\n"
        "\t    baseband: 0 - 59\n"
		"\t[-b output_block_size (default: 16 * 16384)]\n"
		"\t[-S force sync output (default: async)]\n"
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
static void sighandler(int signum)
{
(void) signum;
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	mirisdr_cancel_async(dev);
}
#endif

static void mirisdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	if (ctx) {
		if (fwrite(buf, 1, len, (FILE*)ctx) != len) {
			fprintf(stderr, "Short write, samples lost, exiting!\n");
			mirisdr_cancel_async(dev);
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
	FILE *file;
	uint8_t *buffer;
	uint32_t format = 0;
#if !defined (_WIN32) || defined(__MINGW32__)
	uint32_t transfer = 1;
#else
	uint32_t transfer = 2;
#endif
	uint32_t if_mode = 0;
	uint32_t bw = 8000000;
	uint32_t dev_index = 0;
	uint32_t frequency = 100000000;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    uint32_t device_count;
	char vendor[256] = { 0 }, product[256] = { 0 }, serial[256] = { 0 };
	mirisdr_hw_flavour_t hw_flavour = MIRISDR_HW_DEFAULT;
	int intval;

#if !defined (_WIN32) || defined(__MINGW32__)
	while ((opt = getopt(argc, argv, "b:d:T:e:f:g:G:i:m:s:w:S::")) != -1) {
		switch (opt) {
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			break;
		case 'd':
			dev_index = atoi(optarg);
			break;
        case 'T':
            intval = atoi(optarg);
            if ((intval >=0) && (intval <= 1))
            {
                hw_flavour = (mirisdr_hw_flavour_t) intval;
            }
            break;
		case 'e':
			if ((strcmp("ISOC", optarg) == 0) ||
			    (strcmp("1", optarg) == 0)) {
				transfer = 1;}
			if ((strcmp("BULK", optarg) == 0) ||
			    (strcmp("2", optarg) == 0)) {
				transfer = 2;}
			break;
		case 'f':
			frequency = (uint32_t)atof(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
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
	gain=(int)(atof(argv[3]) * 10); /* tenths of a dB */
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

	buffer = malloc(out_block_size * sizeof(uint8_t));

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

	r = mirisdr_open(&dev, dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open mirisdr device #%d.\n", dev_index);
		exit(1);
	}

	mirisdr_set_hw_flavour(dev, hw_flavour);

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

	/* Set the sample rate */
	r = mirisdr_set_sample_rate(dev, samp_rate);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
	else {
		samp_rate = mirisdr_get_sample_rate(dev);
		fprintf(stderr, "Sample rate is set to %u Hz.\n", samp_rate);
	}

	/* Set the frequency */
	r = mirisdr_set_center_freq(dev, frequency);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	else
		fprintf(stderr, "Tuned to %u Hz.\n", frequency);

	/* Set sample format */
	switch (format) {
	case 1:
		mirisdr_set_sample_format(dev, "504_S8");
		break;
	case 2:
		mirisdr_set_sample_format(dev, "384_S16");
		break;
	case 3:
		mirisdr_set_sample_format(dev, "336_S16");
		break;
	case 4:
		mirisdr_set_sample_format(dev, "252_S16");
		break;
	default:
		mirisdr_set_sample_format(dev, "AUTO");
		break;
	}

	/* Sync reading is bulk only, and the transfer mode defaults to
	   isochronous, so -S on its own would always fail. */
	if (sync_mode && transfer != 2) {
		fprintf(stderr, "Sync output requires bulk transfers, selecting -e 2.\n");
		transfer = 2;
	}

	/* Set USB transfer type */
	switch (transfer) {
	case 1:
		mirisdr_set_transfer(dev, "ISOC");
		break;
	case 2:
		mirisdr_set_transfer(dev, "BULK");
		break;
	}

	/* Set IF mode */
	mirisdr_set_if_freq(dev, if_mode);

	/* Set bandwidth */
	mirisdr_set_bandwidth(dev, bw);

	if (0 == gain && gain_one) {
		 /* Enable automatic gain */
		r = mirisdr_set_tuner_gain_mode(dev, 0);
		if (r < 0)
			fprintf(stderr, "WARNING: Failed to enable automatic gain.\n");
	} else {
		/* Enable manual gain */
		r = mirisdr_set_tuner_gain_mode(dev, 1);
		if (r < 0)
			fprintf(stderr, "WARNING: Failed to enable manual gain.\n");

        if (gain_one) {
            r = mirisdr_set_tuner_gain(dev, gain);
            if (r < 0)
                fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
            else
                fprintf(stderr, "Tuner gain set to %f dB.\n", gain/10.0);
        } else {
            // mixer lna mb bb
            r = mirisdr_set_mixer_gain(dev, gain_mixer);
            if (r < 0)
                fprintf(stderr, "WARNING: Failed to set mixer gain.\n");
            else
                fprintf(stderr, "Mixer amplifier set to %s.\n", gain_mixer ? "ON" : "OFF");

            r = mirisdr_set_lna_gain(dev, gain_lna);
            if (r < 0)
                fprintf(stderr, "WARNING: Failed to set LNA gain.\n");
            else
                fprintf(stderr, "LNA set to %s.\n", gain_mixer ? "ON" : "OFF");

            r = mirisdr_set_mixbuffer_gain(dev, gain_mb);
            if (r < 0)
                fprintf(stderr, "WARNING: Failed to set mixbuffer gain.\n");
            else
                fprintf(stderr, "Mixbuffer gain set to %d dB.\n", gain_mb);

            r = mirisdr_set_baseband_gain(dev, gain_bb);
            if (r < 0)
                fprintf(stderr, "WARNING: Failed to set baseband gain.\n");
            else
                fprintf(stderr, "Baseband gain set to %d dB.\n", gain_bb);
        }
	}

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
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
				break;
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		r = mirisdr_read_async(dev, mirisdr_callback, (void *)file,
				      DEFAULT_ASYNC_BUF_NUMBER, out_block_size);
	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);

	mirisdr_close(dev);
	free (buffer);
out:
	return r >= 0 ? r : -r;
}
