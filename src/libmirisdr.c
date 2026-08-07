/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
 * Copyright (C) 2025 by Peter Hackenberg <170885528+Peter3579@users.noreply.github.com>
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

/* potřebné funkce */
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if !defined (_WIN32) || defined(__MINGW32__)
#include <unistd.h>
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include "libusb.h"

#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

/* hlavní hlavičkový soubor */
#include "mirisdr.h"

/* interní definice */
#include "constants.h"
#include "structs.h"

/* interní funkce - inline */
#include "reg.c"
#include "adc.c"
#include "convert/base.c"
#include "async.c"
#include "devices.c"
#include "gain.c"
#include "hard.c"
#include "streaming.c"
#include "soft.c"
#include "sync.c"

int mirisdr_setup (mirisdr_dev_t **out_dev, mirisdr_dev_t *dev) {
    int r;

    if (libusb_kernel_driver_active(dev->dh, 0) == 1) {
        dev->driver_active = 1;

#ifdef DETACH_KERNEL_DRIVER
        if (!libusb_detach_kernel_driver(dev->dh, 0)) {
            fprintf(stderr, "Detached kernel driver\n");
        } else {
            fprintf(stderr, "Detaching kernel driver failed!");
            dev->driver_active = 0;
            goto failed;
        }
#else
        fprintf(stderr, "\nKernel driver is active, or device is "
                "claimed by second instance of libmirisdr."
                "\nIn the first case, please either detach"
                " or blacklist the kernel module\n"
                "(msi001 and msi2500), or enable automatic"
                " detaching at compile time.\n\n");
#endif
    } else {
        dev->driver_active = 0;
    }

    if ((r = libusb_claim_interface(dev->dh, 0)) < 0) {
        fprintf(stderr, "failed to claim miri usb device %u with code %d: %s\n", dev->index, r, libusb_error_name(r));
        if (r == LIBUSB_ERROR_BUSY) {
            fprintf(stderr, "Verify that the SDRplay background service is not running by `sudo systemctl stop sdrplay` and try again.\n");
        }

        goto failed;
    }

    /* reset je potřeba, jinak občas zařízení odmítá komunikovat */
    mirisdr_reset(dev);

    /* ještě je třeba vždy ukončit i streamování, které může být při otevření aktivní */
    mirisdr_streaming_stop(dev);
    mirisdr_adc_stop(dev);

    /* inicializace tuneru */
    dev->freq = DEFAULT_FREQ;
    dev->rate = DEFAULT_RATE;
    dev->gain = DEFAULT_GAIN;
    dev->band = MIRISDR_BAND_VHF; // matches always the default frequency of 90 MHz

    dev->gain_reduction_lna = 0;
    dev->gain_reduction_mixer = 0;
    dev->gain_reduction_baseband = 43;
    dev->if_freq = MIRISDR_IF_ZERO;
    dev->format_auto = MIRISDR_FORMAT_AUTO_ON;
    dev->bandwidth = MIRISDR_BW_8MHZ;
    dev->xtal = MIRISDR_XTAL_24M;
    dev->bias = 0;
    dev->dc_mode = MIRISDR_DC_PERIODIC2;
    dev->dc_speedup = 0;
    dev->dc_track = MIRISDR_DC_OFFSET_TRACK_TIME_DEFAULT;
    dev->dc_period = MIRISDR_DC_OFFSET_REFRESH_TIME_DEFAULT;
    dev->sync_started = 0;
    dev->harmonic = 1;
    dev->harmonic_auto = 1;
    dev->invert_spectrum = 0;
    dev->lost_samples = 0;
    dev->lost_events = 0;
    dev->desync_events = 0;

    /*
     * Ask the hardware which plan it wants before falling back to a guess. The
     * caller can still override with mirisdr_set_hw_flavour(); this only picks
     * a better default than a fixed one, and leaves the fixed one in place when
     * the probe cannot tell.
     */
    {
        int detected = mirisdr_detect_hw_flavour(dev);

        dev->hw_flavour = (detected < 0) ? MIRISDR_HW_DEFAULT
                                         : (mirisdr_hw_flavour_t) detected;
    }

    /* ISOC is more stable but works only on Unix systems */
#if !defined (_WIN32) || defined(__MINGW32__)
    dev->transfer = MIRISDR_TRANSFER_ISOC;
#else
    dev->transfer = MIRISDR_TRANSFER_BULK;
#endif

    mirisdr_adc_init(dev);
    mirisdr_set_hard(dev);
    mirisdr_set_soft(dev);
    mirisdr_set_gain(dev);

    *out_dev = dev;

    return 0;

failed:
    if (dev) {
        if (dev->dh) {
            libusb_release_interface(dev->dh, 0);
            libusb_close(dev->dh);
        }
        if (dev->ctx) libusb_exit(dev->ctx);
        free(dev);
    }

    return -1;
}

int mirisdr_open (mirisdr_dev_t **p, uint32_t index) {
    mirisdr_dev_t *dev = NULL;
    libusb_device **list, *device = NULL;
    struct libusb_device_descriptor dd;
    ssize_t i, i_max;
    size_t count = 0;
    int r;

    *p = NULL;

    if (!(dev = malloc(sizeof(*dev)))) return -ENOMEM;

    memset(dev, 0, sizeof(*dev));

    /* ostatní parametry */
    dev->index = index;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    libusb_init(&dev->ctx);
    i_max = libusb_get_device_list(dev->ctx, &list);

    for (i = 0; i < i_max; i++) {
        libusb_get_device_descriptor(list[i], &dd);

        if ((mirisdr_device_get(dd.idVendor, dd.idProduct)) &&
            (count++ == index)) {
            device = list[i];
            break;
        }
    }

    /* nenašli jsme zařízení */
    if (!device) {
        libusb_free_device_list(list, 1);
        fprintf( stderr, "no miri device %u found\n", dev->index);
        goto failed;
    }

    /* otevření zařízení */
    if ((r = libusb_open(device, &dev->dh)) < 0) {
        libusb_free_device_list(list, 1);
        fprintf( stderr, "failed to open miri usb device %u with code %d\n", dev->index, r);
        goto failed;
    }

    libusb_free_device_list(list, 1);

    return mirisdr_setup(p, dev);

failed:
    if (dev) {
        if (dev->dh) {
            libusb_close(dev->dh);
        }
        if (dev->ctx) libusb_exit(dev->ctx);
        free(dev);
    }

    return -1;
}

int mirisdr_open_fd (mirisdr_dev_t **p, int fd) {
    mirisdr_dev_t *dev = NULL;
    libusb_device **list, *device = NULL;
    struct libusb_device_descriptor dd;
    ssize_t i, i_max;
    size_t count = 0;
    int r;

    *p = NULL;

    if (!(dev = malloc(sizeof(*dev)))) return -ENOMEM;

    memset(dev, 0, sizeof(*dev));

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    r = libusb_init(&dev->ctx);
    if(r < 0){
        free(dev);
        return -1;
    }
    
    r = libusb_wrap_sys_device(dev->ctx, (intptr_t)fd, &dev->dh);
    if (r || dev->dh == NULL){
        free(dev);
        return -1;
    }

    return mirisdr_setup(p, dev);
}

int mirisdr_close (mirisdr_dev_t *p) {
    if (!p) goto failed;

    /* ukončení async čtení okamžitě */
    mirisdr_cancel_async_now(p);

    /* the sync path starts the stream itself, so it has to stop it itself too;
     * a device left streaming into a closed handle is what wedges the receiver */
    mirisdr_stop_sync(p);

    // similar to rtl-sdr
#if defined(_WIN32) && !defined(__MINGW32__)
            Sleep(1);
#else
            usleep(1000);
#endif

    /* deinicializace tuneru */
    if (p->dh)
    {
        libusb_release_interface(p->dh, 0);

#ifdef DETACH_KERNEL_DRIVER
        if (p->driver_active) {
            if (!libusb_attach_kernel_driver(p->dh, 0))
                fprintf(stderr, "Reattached kernel driver\n");
            else
                fprintf(stderr, "Reattaching kernel driver failed!\n");
        }
#endif
        /* Always close the handle. Skipping libusb_close() when the async side
         * had failed leaks the device handle, so the receiver stays claimed and
         * every later open fails with a stalled endpoint (transfer status 4)
         * until it is physically replugged -- i.e. exactly the case where
         * cleaning up matters most. */
        libusb_close(p->dh);
    }

    if (p->ctx) libusb_exit(p->ctx);

    if (p->samples) free(p->samples);

    free(p);

    return 0;

failed:
    return -1;
}

/*
 * Reset the USB link only.
 *
 * Note what this does not do: it leaves the MSi2500 itself exactly as it was.
 * For the chip, see mirisdr_reset_chip().
 */
int mirisdr_reset (mirisdr_dev_t *p) {
    int r;

    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* měli bychom uvolnit zařízení předem? */

    if ((r = libusb_reset_device(p->dh)) < 0) {
        fprintf( stderr, "failed to reset miri usb device %u with code %d\n", p->index, r);
        goto failed;
    }

    return 0;

failed:
    return -1;
}

/*
 * Restart the MSi2500 itself, with the CMD_RESET the firmware dispatches to
 * restart its 8051.
 *
 * The command has been defined in reg.c since the beginning and never sent by
 * anybody in this lineage, which is why one particular failure has always been
 * described as needing a power cycle. Programming a sample rate much above
 * 15.7 MS/s makes the part stop following the rate: it returns about
 * 15.69 MS/s whatever is asked, and reopening, re-initialising and resetting
 * the USB link all leave it there. This clears it immediately and repeatably.
 *
 * The part re-enumerates -- the USB device number changes -- so this is a soft
 * replug, not a register poke. The handle is dead the moment it returns and
 * the caller must close and reopen. That is also why it cannot go in
 * mirisdr_reset() or the open path: the device would vanish mid-setup, and a
 * libusb_reset_device() issued too soon afterwards interrupts the restart and
 * leaves the fault in place.
 */
int mirisdr_reset_chip (mirisdr_dev_t *p) {
    if (!p) return -1;
    if (!p->dh) return -1;

    /* the device disappears while this is in flight, so a failure here is not
     * necessarily one; the caller finds out by reopening */
    libusb_control_transfer(p->dh, 0x42, CMD_RESET, 0, 0, NULL, 0, CTRL_TIMEOUT);

    return 0;
}

int mirisdr_reset_buffer (mirisdr_dev_t *p) {
    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* zatím není jasné k čemu by bylo, proto pouze provedeme reset async části */
    mirisdr_stop_async(p);
    mirisdr_start_async(p);

    return 0;

failed:
    return -1;
}

int mirisdr_get_usb_strings (mirisdr_dev_t *dev, char *manufact, char *product, char *serial) {
(void) dev;
    fprintf( stderr, "mirisdr_get_usb_strings not implemented yet\n");

    memset(manufact, 0, 256);
    memset(product, 0, 256);
    memset(serial, 0, 256);

    return 0;
}

/*
 * Work out which board this is, by asking what it pulls its GPIO lines to.
 *
 * Two boards enumerate identically as 1df7:2500 and need different band-select
 * words: one is driven by the SDRplay plan, the other only by the default one,
 * and choosing wrong leaves the receiver deaf with nothing reported anywhere.
 * The USB descriptors are no help -- both answer bcdDevice 2.00, 400 mA, four
 * alternate settings and no string descriptors at all.
 *
 * The boards differ below that. Register 0x08 is write-only, but its bits 15:12
 * are per-pin output enables and register 0x06 reads the pins back, bits 2:0
 * being GPIO_2, GPIO_1 and GPIO_0. Release a pin and it settles to whatever the
 * board pulls it to, which is a property of the board and not of the chip.
 * Measured, the two are a clean inversion: all three pulled up on one, all three
 * pulled down on the other.
 *
 * Two safeguards are load bearing.
 *
 * GPIO_3 is never released. On RSP1-style hardware it is the bias tee and it
 * floats high, so releasing it switches DC onto the antenna port -- which is
 * how a board was taken off the USB bus during this work. GPIO_0 to GPIO_2
 * discriminate on their own, so there is no reason to touch it. Note this also
 * makes Osmocom's init word 0x006680 worth avoiding here: its enable nibble of
 * 0x6 releases GPIO_3 along with GPIO_0.
 *
 * And the answer is only accepted if it does not follow the data latch. The
 * pins are read once with the latch written low and once with it written high;
 * if the two disagree the readback is reporting the latch rather than the pin,
 * and this returns -1 rather than guess. The caller keeps whatever it had.
 */
int mirisdr_detect_hw_flavour (mirisdr_dev_t *p) {
    long lo, hi;

    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* enable nibble 8: GPIO_3 driven low, GPIO_0..2 released */
    mirisdr_write_reg(p, 0x08, 0x008080);
    lo = mirisdr_read_reg(p, 0x06);
    mirisdr_write_reg(p, 0x08, 0x008780);
    hi = mirisdr_read_reg(p, 0x06);

    /* leave the pins driven again; the caller's band word follows shortly */
    mirisdr_write_reg(p, 0x08, 0x00f380);

    if ((lo < 0) || (hi < 0)) goto failed;
    if ((lo & 0x07) != (hi & 0x07)) goto failed;      /* it is the latch, not the pin */

    switch (lo & 0x07) {
    case 0x07: return MIRISDR_HW_SDRPLAY;             /* all pulled up   */
    case 0x00: return MIRISDR_HW_DEFAULT;             /* all pulled down */
    default:
        /*
         * A mixed pattern is a board neither of the two measured ones, so fall
         * back to the single line that carries the answer rather than guess
         * from the group.
         *
         * The two plans are separated exactly by bit 9, GPIO_1: every word in
         * the SDRplay plan leaves it clear and every word in the default plan
         * sets it. There is no overlap. And on both boards measured, the pull
         * is the opposite of what the correct plan drives -- the RSP1-style
         * board pulls GPIO_1 up and wants the plan that leaves it low, the
         * generic board pulls it down and wants the plan that drives it high.
         *
         * Read that as the board idling the line and the plan asserting it, and
         * one bit decides. It reproduces both known boards, and unlike a
         * majority vote across three pins it has a mechanism behind it rather
         * than a pattern fitted to two samples.
         */
        return ((lo >> 1) & 0x01) ? MIRISDR_HW_SDRPLAY : MIRISDR_HW_DEFAULT;
    }

failed:
    /*
     * Reached only when the readback itself is untrustworthy -- a failed
     * transfer, or pins that follow the data latch. Deliberately distinct from
     * an unrecognised pull pattern above: there the hardware answered and the
     * answer was merely unfamiliar, here it did not answer at all, and the
     * caller should keep whatever it already had rather than take a guess.
     */
    return -1;
}

mirisdr_hw_flavour_t mirisdr_get_hw_flavour (mirisdr_dev_t *p) {
    return p ? p->hw_flavour : MIRISDR_HW_DEFAULT;
}

int mirisdr_set_hw_flavour (mirisdr_dev_t *p, mirisdr_hw_flavour_t hw_flavour) {
    if (!p) goto failed;

    p->hw_flavour = hw_flavour;
    return 0;

failed:
    return -1;
}
