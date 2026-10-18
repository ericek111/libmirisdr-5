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

/* Bulk only.  Reads whole transfers, converts them like the async path and hands
   out exactly len bytes, keeping the rest for the next call. */
int mirisdr_read_sync (mirisdr_dev_t *p, void *buf, int len, int *n_read) {
    uint8_t *out = buf;
    int got = 0, n, r, k, phase, want;

    if (!p || !p->dh || !buf || len < 0) goto failed;
    if (p->bb) {
        fprintf(stderr, "mirisdr_read_sync() does not do baseband, use mirisdr_read_async()\n");
        goto failed;
    }
    if (n_read) *n_read = 0;
    if (p->transfer != MIRISDR_TRANSFER_BULK) {
        fprintf(stderr, "mirisdr_read_sync() reads bulk transfers: set the transfer to BULK\n");
        goto failed;
    }
    if (mirisdr_async_get(p) != MIRISDR_ASYNC_INACTIVE) goto failed;

    if (!p->sync_in && !(p->sync_in = malloc(DEFAULT_BULK_BUFFER))) goto failed;
    if (!p->sync_out && !(p->sync_out = malloc((DEFAULT_BULK_BUFFER / 1024 + 2) * MIRISDR_BLOCK_OUT_MAX))) goto failed;

    if (!p->sync_ready) {
        if (libusb_set_interface_alt_setting(p->dh, 0, p->alt_setting) < 0) goto failed;
        if (mirisdr_streaming_start(p) < 0) {
            mirisdr_streaming_stop(p);
            goto failed;
        }

        memset(&p->stats, 0, sizeof(p->stats));
        p->count_index = p->count_gaps = 0;
        p->count_addr = 0;
        p->sync_run = 0;
        p->addr_valid = 0;
        p->bulk_carry_n = 0;
        p->bulk_lost = 0;
        p->bulk_wait = 0;
        p->gap_track = 0;
        p->ev_valid = 0;
        p->sync_len = p->sync_pos = 0;
        p->sync_xlen = DEFAULT_BULK_BUFFER;
        p->sync_ready = 1;
    }

    while (got < len) {
        if (p->sync_pos == p->sync_len) {
            want = p->sync_xlen;
            r = libusb_bulk_transfer(p->dh, 0x81, p->sync_in, want, &n, DEFAULT_BULK_TIMEOUT);
            if (r < 0) {
                if (got) break;
                /* the device stops, and the next read starts it from the beginning */
                mirisdr_streaming_stop(p);
                return -1;
            }
            p->sync_xlen = DEFAULT_BULK_BUFFER;

            p->stats_head = 1;
            if (p->fw_ours) {
                p->sync_len = mirisdr_parse_bulk(p, p->sync_in, n, want, p->sync_out);
                p->sync_xlen = mirisdr_bulk_next_len(p);
            } else
                p->sync_len = mirisdr_convert_bulk(p, p->sync_in, p->sync_out, n);
            p->sync_pos = 0;

            /* without the stamp: a few bad blocks in a row are off the 1 kB grid.  Blocks
               start at phase in every read, so one read of 1024 - phase less puts the next
               on the grid. */
            if (!p->fw_ours && p->sync_run > 2) {
                p->sync_run = 0;
                if ((phase = mirisdr_bulk_phase(p, p->sync_in, n)) > 0) {
                    p->stats.resyncs++;
                    p->sync_xlen = DEFAULT_BULK_BUFFER - 1024 + phase;
                    fprintf(stderr, "libmirisdr: block grid is %d bytes out, shifting.\n", phase);
                }
            }
            continue;
        }

        k = p->sync_len - p->sync_pos;
        if (k > len - got) k = len - got;
        memcpy(out + got, p->sync_out + p->sync_pos, k);
        p->sync_pos += k;
        got += k;
    }

    if (n_read) *n_read = got;
    return 0;

failed:
    return -1;
}
