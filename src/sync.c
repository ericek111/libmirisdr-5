/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
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
 * Bring endpoint 0x81 up for synchronous reading.
 *
 * The sync path is bulk only -- mirisdr_read_sync() uses libusb_bulk_transfer()
 * -- and only alternate setting 3 puts a bulk endpoint on interface 0. After
 * libusb_claim_interface() the interface sits at alternate setting 0, which
 * declares no endpoints at all, so a read issued before the setting is selected
 * cannot do anything but fail with LIBUSB_ERROR_IO.
 *
 * mirisdr_read_async() selects the setting and starts the stream itself;
 * nothing ever did it for the sync path, which is why synchronous reads had
 * never returned a byte. The device wants alternate setting 3 first and control
 * request 0x43 second -- steps 7 and 8 of the MSi2500 startup sequence.
 */
static int mirisdr_start_sync (mirisdr_dev_t *p) {
    int r;

    if (p->transfer != MIRISDR_TRANSFER_BULK) {
        fprintf(stderr, "mirisdr_read_sync: synchronous reading needs bulk transfers,"
                        " call mirisdr_set_transfer(dev, \"BULK\") first\n");
        goto failed;
    }

    if (p->async_status != MIRISDR_ASYNC_INACTIVE) {
        fprintf(stderr, "mirisdr_read_sync: asynchronous reading is already running"
                        " on miri usb device %u\n", p->index);
        goto failed;
    }

    if ((r = libusb_set_interface_alt_setting(p->dh, 0, 3)) < 0) {
        fprintf(stderr, "failed to use alternate setting for Bulk mode on miri usb"
                        " device %u with code %d\n", p->index, r);
        goto failed;
    }

    mirisdr_streaming_start(p);
    p->sync_started = 1;

    return 0;

failed:
    return -1;
}

/* pouze pro bulk transfer a je třeba doplnit konverze formátů */
int mirisdr_read_sync (mirisdr_dev_t *p, void *buf, int len, int *n_read) {
    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* start the stream on the first read, the way read_async does for itself */
    if (!p->sync_started && (mirisdr_start_sync(p) < 0)) goto failed;

    return libusb_bulk_transfer(p->dh, 0x81, buf, len, n_read, DEFAULT_BULK_TIMEOUT);

failed:
    return -1;
}

static int mirisdr_stop_sync (mirisdr_dev_t *p) {
    if (!p) goto failed;
    if (!p->dh) goto failed;
    if (!p->sync_started) return 0;

    mirisdr_streaming_stop(p);
    p->sync_started = 0;

    return 0;

failed:
    return -1;
}
