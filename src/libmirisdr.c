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

/* system headers */
#include <errno.h>
#include <stdarg.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#if !defined (_WIN32) || defined(__MINGW32__)
#include <unistd.h>
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include "libusb.h"

#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

/* the public header */
#include "mirisdr.h"

/* internal definitions */
#include "constants.h"
#include "structs.h"

/* internal functions, included */
/* why a stream or tune is refused, unless only checking */
static void mirisdr_refuse (mirisdr_dev_t *p, const char *fmt, ...)
{
    va_list ap;

    if (p->checking) return;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

/* A config with its reserved part set was written for a newer libmirisdr */
static int mirisdr_reserved_set (const uint32_t *r, size_t n)
{
    while (n--) if (*r++) return 1;
    return 0;
}

#define MIRISDR_RESERVED_SET(s) mirisdr_reserved_set((s).reserved, sizeof (s).reserved / sizeof (s).reserved[0])
#define MIRISDR_RESERVED_MSG    "%s sets fields this libmirisdr does not have: the program needs a newer one\n"

#include "reg.c"
#include "debug.c"
#include "gpio.c"
#include "eeprom.c"
#include "uart.c"
#include "i2c.c"
#include "pps.c"
#include "ir.c"
#include "tuner.c"
#include "events.c"
#include "adc.c"
#include "convert/base.c"
#include "iqconverter_float.c"
#include "baseband.c"
#include "async.c"
#include "devices.c"
#include "firmware_pps.c"
#include "firmware.c"
#include "gain.c"
#include "hard.c"
#include "streaming.c"
#include "soft.c"
#include "scan.c"
#include "sync.c"
#include "null.c"

static int mirisdr_setup (mirisdr_dev_t **out_dev, mirisdr_dev_t *dev) {
    int r;

    mirisdr_xfer_lock_init(dev);

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

#ifdef DETACH_KERNEL_DRIVER
        if (dev->driver_active) libusb_attach_kernel_driver(dev->dh, 0);
#endif
        goto failed;
    }

    /* a register list our firmware still runs (a process that ended inside a scan) makes it
       refuse every register write, a USB reset included: stop it first. The ROM, which has no
       such request, refuses it, which is fine */
    libusb_control_transfer(dev->dh, 0x42, 0x5A /* CMD_REG_LIST */, 0, 0, NULL, 0, 1000);

    /* the reset is needed, otherwise the device at times refuses to communicate */
    mirisdr_reset(dev);

    /* and always stop streaming, which can still be running when opening */
    mirisdr_streaming_stop(dev);
    mirisdr_adc_stop(dev);

    if (dev->hw_flavour == MIRISDR_HW_AUTO) dev->hw_flavour = mirisdr_flavour_of(dev->dh);
    dev->exp_valid = 0;

    /* a readback leaves the tuner holding its answer line until its next word, and a
       process that ended there leaves it so: on the RSPduo the expander setup then left the
       other tuner deaf (tuner.c). A register 4 word first lets go of it, on every board. On
       the RSPduo the route is where it was, so it reaches the tuner that was read. Whether
       it reached tuner 0 is not known, which is all the tracking could say */
    if (!dev->external_tuner)
    {
        mirisdr_tuner_write(dev, MIRISDR_TUNER_IDLE, 0, 1);
        dev->tuner_valid = 0;
        dev->tuner_last = 0;
    }

    /* initialise the tuner */
    mirisdr_tuner_defaults(dev);
    mirisdr_tuner_ctx_init(dev);
    dev->rate = DEFAULT_RATE;
    dev->format_auto = MIRISDR_FORMAT_AUTO_ON;
    dev->decimation_bypass = MIRISDR_DECIMATION_BYPASS_AUTO;
    mirisdr_set_i2c_rate(dev, 50000);
    dev->ee_size = -1;              /* not probed yet, see eeprom.c */

    /* the transfer mode is the platform's default, see hard.c */
    mirisdr_stream_config_default(&dev->stream);

    mirisdr_adc_init(dev);
    mirisdr_set_hard(dev);
    mirisdr_set_soft(dev);
    mirisdr_set_gain(dev);

    *out_dev = dev;

    return 0;

failed:
    if (dev) {
        mirisdr_xfer_lock_destroy(dev);
        if (dev->dh) {
            libusb_release_interface(dev->dh, 0);
            libusb_close(dev->dh);
        }
        if (dev->ctx) libusb_exit(dev->ctx);
        free(dev);
    }

    return -1;
}

static int mirisdr_open_raw (mirisdr_dev_t **p, uint32_t index, const char *serial, const char *usb_port,
                             int external_tuner, uint8_t gpio_in, int flavour) {
    mirisdr_dev_t *dev = NULL;
    libusb_device **list, *device = NULL;
    ssize_t i_max;
    int at = (int) index;
    int r;

    *p = NULL;

    if (!(dev = malloc(sizeof(*dev)))) return -ENOMEM;

    memset(dev, 0, sizeof(*dev));

    /* the other parameters */
    dev->index = index;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    if (libusb_init(&dev->ctx) < 0) {
        dev->ctx = NULL;
        fprintf(stderr, "libusb cannot start\n");
        goto failed;
    }

    if ((i_max = libusb_get_device_list(dev->ctx, &list)) < 0) {
        fprintf(stderr, "libusb cannot list the devices\n");
        goto failed;
    }

    device = mirisdr_pick(list, i_max, index, serial, usb_port, &at);
    dev->index = (uint32_t) at;

    /* no device found */
    if (!device) {
        libusb_free_device_list(list, 1);
        if (serial || usb_port) fprintf(stderr, "no miri device %s%s%s found\n", serial ? serial : "",
                                        (serial && usb_port) ? " or at " : "", usb_port ? usb_port : "");
        else fprintf(stderr, "no miri device %u found\n", index);
        goto failed;
    }

    /* open the device */
    if ((r = libusb_open(device, &dev->dh)) < 0) {
        libusb_free_device_list(list, 1);
        fprintf( stderr, "failed to open miri usb device %u with code %d\n", dev->index, r);
        goto failed;
    }

    libusb_free_device_list(list, 1);

    dev->external_tuner = external_tuner;
    dev->hw_flavour = flavour;
    mirisdr_gpio_hold_input(dev, gpio_in);

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

static int mirisdr_open_fd_raw (mirisdr_dev_t **p, int fd, int external_tuner, uint8_t gpio_in, int flavour) {
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
        libusb_exit(dev->ctx);
        free(dev);
        return -1;
    }

    dev->external_tuner = external_tuner;
    dev->hw_flavour = flavour;
    mirisdr_gpio_hold_input(dev, gpio_in);

    return mirisdr_setup(p, dev);
}

int mirisdr_open (mirisdr_dev_t **p, uint32_t index) {
    mirisdr_open_config_t cfg;

    mirisdr_open_config_default(&cfg);
    cfg.index = index;

    return mirisdr_open_ex(p, &cfg);
}

int mirisdr_open_fd (mirisdr_dev_t **p, int fd) {
    mirisdr_open_config_t cfg;

    mirisdr_open_config_default(&cfg);
    cfg.fd = fd;

    return mirisdr_open_ex(p, &cfg);
}

int mirisdr_close (mirisdr_dev_t *p) {
    if (!p) goto failed;

    /* stop the async read right away */
    mirisdr_cancel_async_now(p);

    // similar to rtl-sdr
#if defined(_WIN32) && !defined(__MINGW32__)
            Sleep(1);
#else
            usleep(1000);
#endif

    /* deinitialise the tuner */
    if (p->dh)
    {
        /* no register list left running: the next open's writes would be refused */
        if (p->fw_ours)
        {
            if (p->scan) mirisdr_scan_stop(p);
            mirisdr_load_list(p, 0, 0, NULL, 0);
        }

        libusb_release_interface(p->dh, 0);

#ifdef DETACH_KERNEL_DRIVER
        if (p->driver_active) {
            if (!libusb_attach_kernel_driver(p->dh, 0))
                fprintf(stderr, "Reattached kernel driver\n");
            else
                fprintf(stderr, "Reattaching kernel driver failed!\n");
        }
#endif
        /* transfers a stream gave up on: the device memory is unmapped with the handle
           still there, the transfers freed once closing it has taken them off libusb's
           list (the kernel keeps the pages until it is through with them) */
        if (p->xfer_abandoned && p->xfer_buf && p->xfer_buf_devmem)
        {
            size_t i;

            for (i = 0; i < p->xfer_buf_num; i++)
            {
                if (p->xfer_buf[i]) libusb_dev_mem_free(p->dh, p->xfer_buf[i], p->xfer_buf_size);
                p->xfer_buf[i] = NULL;
            }
        }

        /* Always close the handle. Skipping libusb_close() when the async side
         * had failed leaks the device handle, so the receiver stays claimed and
         * every later open fails with a stalled endpoint (transfer status 4)
         * until it is physically replugged -- i.e. exactly the case where
         * cleaning up matters most. */
        libusb_close(p->dh);
        p->dh = NULL;
    }

    if (p->xfer_abandoned) mirisdr_async_free(p);

    if (p->ctx) libusb_exit(p->ctx);

    if (p->samples) free(p->samples);
    if (p->xfer_out) free(p->xfer_out);
    mirisdr_bb_free(p);
    if (p->sync_in) free(p->sync_in);
    if (p->sync_out) free(p->sync_out);

    mirisdr_xfer_lock_destroy(p);
    free(p);

    return 0;

failed:
    return -1;
}

int mirisdr_reset (mirisdr_dev_t *p) {
    int r;

    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* should we release the device first? */

    /* the part may come back with its registers at their defaults */
    p->reg8_valid = 0;
    p->tuner_valid = 0;

    if ((r = libusb_reset_device(p->dh)) < 0) {
        fprintf( stderr, "failed to reset miri usb device %u with code %d\n", p->index, r);
        goto failed;
    }

    return 0;

failed:
    return -1;
}

int mirisdr_get_stream_stats (mirisdr_dev_t *p, mirisdr_stream_stats_t *s) {
    if (!p || !s) return -1;

    *s = p->stats;

    return 0;
}

int mirisdr_reset_buffer (mirisdr_dev_t *p) {
    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* not yet clear what this would be for, so only restart the async part. A stream the
       app paused stays paused */
    if ((mirisdr_async_get(p) == MIRISDR_ASYNC_RUNNING) && (mirisdr_stop_async(p) == 0) &&
        (mirisdr_stream_resume(p) < 0)) goto failed;

    return 0;

failed:
    return -1;
}

int mirisdr_get_usb_strings (mirisdr_dev_t *dev, char *manufact, char *product, char *serial) {
    struct libusb_device_descriptor dd;
    mirisdr_device_t *known;
    libusb_device *d;

    if (!dev || !dev->dh || !manufact || !product || !serial) return -1;
    if (!(d = libusb_get_device(dev->dh))) return -1;
    if (libusb_get_device_descriptor(d, &dd) < 0) return -1;

    memset(manufact, 0, 256);
    memset(product, 0, 256);
    memset(serial, 0, 256);

    /* both read, so a missing one does not hide the other */
    mirisdr_usb_string(dev->dh, dd.iManufacturer, manufact, 256);
    mirisdr_usb_string(dev->dh, dd.iProduct, product, 256);

    if (!*manufact || !*product)
    {
        if ((known = mirisdr_device_get(dd.idVendor, dd.idProduct)))
        {
            if (!*manufact) strcpy(manufact, known->manufacturer);
            if (!*product) strcpy(product, known->product);
        }
    }

    if (!mirisdr_usb_string(dev->dh, dd.iSerialNumber, serial, 256))
        mirisdr_port_name(d, serial);

    return 0;
}

int mirisdr_get_usb_ids (mirisdr_dev_t *p, uint16_t *vid, uint16_t *pid) {
    struct libusb_device_descriptor dd;
    libusb_device *d;

    if (!p || !p->dh) return -1;
    if (!(d = libusb_get_device(p->dh))) return -1;
    if (libusb_get_device_descriptor(d, &dd) < 0) return -1;

    if (vid) *vid = dd.idVendor;
    if (pid) *pid = dd.idProduct;

    return 0;
}

int mirisdr_get_serial (mirisdr_dev_t *p, char *out, int len) {
    struct libusb_device_descriptor dd;
    libusb_device *d;

    if (!p || !p->dh || !out || (len <= 0)) return -1;
    if (!(d = libusb_get_device(p->dh))) return -1;
    if (libusb_get_device_descriptor(d, &dd) < 0) return -1;

    *out = 0;

    if (!mirisdr_usb_string(p->dh, dd.iSerialNumber, out, len)) return 0;

    return (int) strlen(out);
}

int mirisdr_get_usb_position (mirisdr_dev_t *p, uint8_t *busnum, uint8_t *devnum, char *port, int len) {
    libusb_device *d;
    uint8_t bus;

    if (!p || !p->dh) return -1;
    if (!(d = libusb_get_device(p->dh))) return -1;

    bus = libusb_get_bus_number(d);
    if (busnum) *busnum = bus;
    if (devnum) *devnum = libusb_get_device_address(d);

    if (port) {
#if LIBUSBX_API_VERSION >= 0x01000102
        /* USB allows 7 tiers of hubs below the root */
        uint8_t path[7];
        int n, i, used;

        if (len <= 0) return -1;
        n = libusb_get_port_numbers(d, path, sizeof(path));
        if (n < 0) return -1;

        used = snprintf(port, len, "%u", bus);
        for (i = 0; i < n && used < len; i++)
            used += snprintf(port + used, len - used, i == 0 ? "-%u" : ".%u", path[i]);
        if (used >= len) return -1;
#else
        /* libusb before 1.0.16 cannot tell the port path */
        return -1;
#endif
    }

    return 0;
}

int mirisdr_get_hw_flavour (mirisdr_dev_t *p) {
    return p ? (int) p->hw_flavour : -1;
}

/* What a board does not have, out of the selected tuner's stored tune: a notch, antenna,
   pad or raw part left in it would refuse every later tune */
static void mirisdr_flavour_clean (mirisdr_dev_t *p, mirisdr_hw_flavour_t hw_flavour)
{
    if (!MIRISDR_HW_EXPANDER(hw_flavour))
    {
        p->notch = 0;
        p->tune.frontend.notch = 0;
    }

    if (hw_flavour == MIRISDR_HW_RSPDUO)
    {
        p->bias = 0;
        p->tune.frontend.bias = 0;
    }

    /* the pads differ from board to board */
    p->attenuator = 0;
    p->tune.frontend.attenuator = 0;
    p->tune.frontend.raw.fields &= mirisdr_frontend_parts(hw_flavour);
    p->tune.frontend.flags &= mirisdr_frontend_flags(hw_flavour);

    if (hw_flavour != MIRISDR_HW_RSPDXR2)
    {
        p->antenna = MIRISDR_ANTENNA_A;
        p->tune.frontend.antenna = MIRISDR_ANTENNA_A;
        p->ds = 0;
        p->ds_edge = 0;
        p->tune.frontend.direct_sampling = 0;
    }
}

int mirisdr_set_hw_flavour (mirisdr_dev_t *p, mirisdr_hw_flavour_t hw_flavour) {
    mirisdr_tune_config_t c;
    int t, n, cur;

    if (!p) goto failed;

    if (hw_flavour < MIRISDR_HW_DEFAULT || hw_flavour > MIRISDR_HW_RSPDUO) goto failed;

    /* the tuners as the old board had them, each one's stored tune cleaned, tuner 0 last */
    n = mirisdr_tuners_of(p);
    cur = p->tuner_cur;
    p->hw_flavour = hw_flavour;
    p->exp_valid = 0;
    p->fe_flags &= mirisdr_frontend_flags(hw_flavour);

    for (t = n - 1; t >= 0; t--)
    {
        mirisdr_tuner_select(p, t);
        mirisdr_flavour_clean(p, hw_flavour);
    }
    for (t = 0; t < MIRISDR_TUNERS; t++) p->tctx[t].tune.frontend.flags = p->fe_flags;
    p->tune.frontend.flags = p->fe_flags;

    /* one tuner: the stream takes it, the single setters act on it */
    if (hw_flavour != MIRISDR_HW_RSPDUO) p->stream.tuners = 0;
    else if (cur) mirisdr_tuner_select(p, cur);

    /* the front end as this board has it, register 8 with it: the stored tune again */
    if (p->dh)
    {
        c = p->tune;
        c.gain.mode = MIRISDR_GAIN_KEEP;
        if (mirisdr_tune(p, p->tuner_cur, &c, NULL) < 0) goto failed;
    }

    return 0;

failed:
    return -1;
}
