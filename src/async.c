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

#include "async.h"

static int mirisdr_stream_adc (mirisdr_dev_t *p);

static void mirisdr_cb_call (mirisdr_dev_t *p, unsigned char *buf, uint32_t len) {
    mirisdr_buffer_info_t *in = &p->cb_info;
    uint64_t ub = mirisdr_unit_bytes(p), start = p->cb_bytes, end = start + len, later = 0;
    int k = 0, j;

    memset(in, 0, sizeof *in);
    in->sample = p->cb_base + start / ub;
    in->adc = mirisdr_stream_adc(p);
    in->rate = p->rate;
    in->type = (p->format == MIRISDR_FORMAT_504_S8) ? MIRISDR_SAMPLE_S8 : MIRISDR_SAMPLE_S16;
    in->tuner = mirisdr_tuner_primary(p);

    /* the queue's gaps up to this buffer's end, in order */
    while (k < p->gapq_n && p->gapq[k].at < end) {
        uint64_t unfilled = p->gapq[k].samples - p->gapq[k].filled, keep = 0;

        /* zeros running past the end go on in the next buffer, as a gap of
           nothing but fill at its start */
        if (p->gapq[k].at + (uint64_t) p->gapq[k].filled * ub > end)
            keep = (p->gapq[k].at + (uint64_t) p->gapq[k].filled * ub - end) / ub;

        if (in->gaps_len < MIRISDR_GAPS_MAX) {
            mirisdr_gap_t *g = &in->gaps[in->gaps_len++];

            g->offset  = p->gapq[k].at > start ? (uint32_t) ((p->gapq[k].at - start) / ub) : 0;
            g->filled  = p->gapq[k].filled - (uint32_t) keep;
            g->samples = p->gapq[k].samples - keep;
        }
        in->gap_samples += p->gapq[k].samples - keep;

        /* a gap at the start lies before its first sample */
        if (p->gapq[k].at <= start) p->cb_lost += unfilled;
        else later += unfilled;

        if (keep) {
            p->gapq[k].at = end;
            p->gapq[k].samples = keep;
            p->gapq[k].filled = (uint32_t) keep;
            break;
        }
        k++;
    }
    for (j = k; j < p->gapq_n; j++) p->gapq[j - k] = p->gapq[j];
    p->gapq_n -= k;

    in->index = in->sample + p->cb_lost;

    if (p->bb) mirisdr_bb_feed(p, buf, len);
    else p->cb(buf, len, p->cb_ctx);

    p->cb_lost += later;
    p->cb_bytes = end;
}

/* store the data */
static int mirisdr_feed_async (mirisdr_dev_t *p, unsigned char *samples, uint32_t bytes) {
    uint32_t i;

    if (!p) goto failed;
    if (!p->cb) goto failed;

    /* automatic size */
    if (!p->xfer_out_len) {
        /* pass it on directly */
        mirisdr_cb_call(p, samples, bytes);
    /* fixed buffer size, nothing held from before */
    } else if (p->xfer_out_pos == 0) {
        /* the buffer fits exactly: rare, pass it on directly */
        if (bytes == p->xfer_out_len) {
            mirisdr_cb_call(p, samples, bytes);
        /* the buffer is shorter */
        } else if (bytes < p->xfer_out_len) {
            memcpy(p->xfer_out, samples, bytes);
            p->xfer_out_pos = bytes;
        /* the buffer is longer */
        } else {
            /* it can be a multiple of the length too */
            for (i = 0;; i+= p->xfer_out_len) {
                if (i + p->xfer_out_len > bytes) {
                    if (bytes > i) {
                        memcpy(p->xfer_out, samples + i, bytes - i);
                        p->xfer_out_pos = bytes - i;
                    }
                    break;
                }
                mirisdr_cb_call(p, samples + i, p->xfer_out_len);
            }
        }
    /* the data fills it exactly, the internal buffer is used */
    } else if (p->xfer_out_pos + bytes == p->xfer_out_len) {
        memcpy(p->xfer_out + p->xfer_out_pos, samples, bytes);
        mirisdr_cb_call(p, p->xfer_out, p->xfer_out_len);
        p->xfer_out_pos = 0;
    /* not enough data */
    } else if (p->xfer_out_pos + bytes < p->xfer_out_len) {
        memcpy(p->xfer_out + p->xfer_out_pos, samples, bytes);
        p->xfer_out_pos+= bytes;
    /* more data than we need, the hardest case */
    } else {
        memcpy(p->xfer_out + p->xfer_out_pos, samples, p->xfer_out_len - p->xfer_out_pos);
        mirisdr_cb_call(p, p->xfer_out, p->xfer_out_len);
        for (i = p->xfer_out_len - p->xfer_out_pos;; i+= p->xfer_out_len) {
            if (i + p->xfer_out_len > bytes) {
                if (bytes > i) {
                    memcpy(p->xfer_out, samples + i, bytes - i);
                    p->xfer_out_pos = bytes - i;
                } else {
                    p->xfer_out_pos = 0;
                }
                break;
            }
            mirisdr_cb_call(p, samples + i, p->xfer_out_len);
        }
    }
    return 0;

failed:
    return -1;
}

static void mirisdr_feed_converted (mirisdr_dev_t *p, unsigned char *samples, uint32_t bytes) {
    static unsigned char zeros[4096];
    uint32_t at = 0, ub = mirisdr_unit_bytes(p), k;
    uint64_t z;
    int i;

    for (i = 0; i < p->fills_n; i++) {
        if (p->fills[i].off > at) {
            mirisdr_feed_async(p, samples + at, p->fills[i].off - at);
            p->fed_bytes += p->fills[i].off - at;
            at = p->fills[i].off;
        }
        p->bb_filling = 1;
        for (z = (uint64_t) p->fills[i].n * ub; z; z -= k) {
            k = z > sizeof zeros ? (uint32_t) sizeof zeros : (uint32_t) z;
            mirisdr_feed_async(p, zeros, k);
            p->fed_bytes += k;
        }
        p->bb_filling = 0;
    }
    if (bytes > at) {
        mirisdr_feed_async(p, samples + at, bytes - at);
        p->fed_bytes += bytes - at;
    }
    p->fills_n = 0;
}

static uint8_t *samples_realloc(mirisdr_dev_t *p, int size)
{
    if(p->samples_size < size)
    {
        if(p->samples)
            free(p->samples);
        p->samples=malloc(size);
        p->samples_size=p->samples ? size : 0;
    }
    return p->samples;
}

#define MIRISDR_BLOCK_OUT_MAX   2016

static int mirisdr_convert_bulk (mirisdr_dev_t *p, uint8_t *src, uint8_t *dst, int cnt)
{
    switch (p->format) {
    case MIRISDR_FORMAT_252_S16:
    case MIRISDR_FORMAT_504_REAL_S16:
        return mirisdr_samples_convert_252_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_336_S16:
    case MIRISDR_FORMAT_672_REAL_S16:
        return mirisdr_samples_convert_336_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_384_S16:
    case MIRISDR_FORMAT_768_REAL_S16:
        return mirisdr_samples_convert_384_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_504_S16:
        return mirisdr_samples_convert_504_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_504_S8:
        return mirisdr_samples_convert_504_s8(p, src, dst, cnt);
    default:
        return 0;
    }
}

/* Our firmware stamps header bytes 8-11 of every buffer at startup - bytes the
   capture engine never writes - so the mark rides along in every packet the
   part sends.  Byte 12 carries which of the four ring buffers it came from,
   and 13-15 are spare. */
static const uint8_t mirisdr_hdr_magic[4] = { 'B', 'V', 'D', 'B' };

static int mirisdr_bulk_stamped (const uint8_t *block)
{
    return !memcmp(block + 8, mirisdr_hdr_magic, sizeof(mirisdr_hdr_magic));
}

/* A block with the next block's stamp in place behind it */
static int mirisdr_bulk_pair (const uint8_t *b)
{
    return mirisdr_bulk_stamped(b) && mirisdr_bulk_stamped(b + 1024);
}

static void mirisdr_bulk_slip (mirisdr_dev_t *p)
{
    if (!p->bulk_lost) {
        p->bulk_lost = 1;
        p->stats.resyncs++;
    }
}

/* want: the length the transfer asked for. One that came in whole ends on the grid, so its
   last block is taken without its successor's stamp */
static int mirisdr_parse_bulk (mirisdr_dev_t *p, const uint8_t *src, int n, int want, uint8_t *dst)
{
    uint8_t *st = p->bulk_carry;
    int out = 0, pos = 0, k, c, h, l;

    if (p->bulk_carry_n) {
        h = n < 1024 + 12 ? n : 1024 + 12;
        memcpy(st + p->bulk_carry_n, src, (size_t) h);
        l = p->bulk_carry_n + h;

        /* the blocks that start in the leftover */
        while (pos < p->bulk_carry_n && pos + 1024 + 12 <= l) {
            if (mirisdr_bulk_pair(st + pos)) {
                out += mirisdr_convert_bulk(p, st + pos, dst + out, 1024);
                p->bulk_lost = 0;
                pos += 1024;
                continue;
            }
            mirisdr_bulk_slip(p);
            for (pos++; pos + 1024 + 12 <= l && !mirisdr_bulk_pair(st + pos); pos++) ;
        }

        /* a transfer too short to get past them: keep it all */
        if (pos < p->bulk_carry_n) {
            memmove(st, st + pos, (size_t) (l - pos));
            p->bulk_carry_n = l - pos;
            return out;
        }

        pos -= p->bulk_carry_n;
        p->bulk_carry_n = 0;
    }

    while (pos + 1024 <= n) {
        if (pos + 1024 + 12 > n) {
            /* the last block, its successor in the next transfer */
            if (!p->bulk_lost && pos + 1024 == n && n == want && mirisdr_bulk_stamped(src + pos)) {
                out += mirisdr_convert_bulk(p, (uint8_t *) src + pos, dst + out, 1024);
                pos += 1024;
            }
            break;
        }

        if (mirisdr_bulk_pair(src + pos)) {
            for (c = 1; pos + (c + 1) * 1024 + 12 <= n && mirisdr_bulk_stamped(src + pos + (c + 1) * 1024); c++) ;
            out += mirisdr_convert_bulk(p, (uint8_t *) src + pos, dst + out, c * 1024);
            p->bulk_lost = 0;
            pos += c * 1024;
            continue;
        }

        mirisdr_bulk_slip(p);
        for (k = pos + 1; k + 1024 + 12 <= n && !mirisdr_bulk_pair(src + k); k++) ;
        if (k + 1024 + 12 > n) {
            /* none: keep the tail, a pair may straddle into the next transfer */
            if (pos < n - (1024 + 11)) pos = n - (1024 + 11);
            break;
        }
        fprintf(stderr, "libmirisdr: off the block grid, %d bytes skipped\n", k - pos);
        pos = k;
    }

    if (pos < n) {
        memcpy(st, src + pos, (size_t) (n - pos));
        p->bulk_carry_n = n - pos;
    }

    return out;
}

static int mirisdr_bulk_next_len (mirisdr_dev_t *p)
{
    if (!p->bulk_lost && p->bulk_carry_n && !(p->bulk_carry_n % 512))
        return DEFAULT_BULK_BUFFER - p->bulk_carry_n;

    return DEFAULT_BULK_BUFFER;
}

/* Where does the 1 kB block grid really start in this buffer?
   Returns -1 when nothing in it looks like the grid. */
static int mirisdr_bulk_phase (mirisdr_dev_t *p, const uint8_t *b, int n)
{
	int k;

	/* the counter's last read is 4 bytes at k + 2048, k up to 1023 */
	if (n < 3 * 1024 + 3) return -1;

	/* The stamp settles it outright: two of them a block apart cannot be
	   sample data. */
	for (k = 0; k < 1024; k++)
		if (!memcmp(b + k + 8, mirisdr_hdr_magic, sizeof(mirisdr_hdr_magic)) &&
		    !memcmp(b + k + 1024 + 8, mirisdr_hdr_magic, sizeof(mirisdr_hdr_magic)))
			return k;

	/* Firmware that does not stamp so fall back to the counter:
     * a run one block apart stepping by addr_step is the grid. */
	for (k = 0; k < 1024; k++)
	{
		uint32_t c0 = b[k]        | b[k+1]<<8    | b[k+2]<<16    | (uint32_t) b[k+3]<<24;
		uint32_t c1 = b[k+1024]   | b[k+1025]<<8 | b[k+1026]<<16 | (uint32_t) b[k+1027]<<24;
		uint32_t c2 = b[k+2048]   | b[k+2049]<<8 | b[k+2050]<<16 | (uint32_t) b[k+2051]<<24;

		if ((c1 - c0 == p->addr_step) && (c2 - c1 == p->addr_step)) return k;
	}

	return -1;
}

/*
 * It is possible to configure how many 1kB blocks at a time the DSP will
 * hand over to the USB controller. If this value is too low not all buffer
 * memory is used and the chance of gaps increases. If it is too high (eg 4
 * packets in a 1 slot isochronous transfer), the stream stops.
 */
static uint32_t mirisdr_alt_burst(uint8_t alt)
{
	switch (alt)
	{
	case 1:  return 3;          /* isochronous, 3 x 1024 per microframe */
	case 2:  return 1;          /* isochronous, 1 x 1024 */
	case 4:  return 2;          /* isochronous, 2 x 1024 (requires custom firmware) */
	default: return 4;          /* bulk */
	}
}

static uint32_t mirisdr_burst(mirisdr_dev_t *p)
{
	return mirisdr_alt_burst(p->alt_setting);
}

/* The host polls an isochronous endpoint only a few hundred us after the transfers
   are submitted. A stream started before then overflows the device's at 7-8 Msps
   and it skips samples, so wait a bit with starting :) */
static void mirisdr_iso_settle (mirisdr_dev_t *p) {
    if (p->transfer != MIRISDR_TRANSFER_ISOC) return;
#if defined (_WIN32) && !defined(__MINGW32__)
    Sleep(1);
#else
    usleep(1000);
#endif
}

static int mirisdr_async_drain (mirisdr_dev_t *p, int rounds) {
    size_t i;
    int r;
    struct timeval tv = {1, 0};
    p->xfer_draining = 1;
    while (mirisdr_atomic_add(&p->xfer_inflight, 0) > 0) {
        if (rounds-- <= 0) return -1;
        for (i = 0; i < p->xfer_buf_num; i++)
            if (p->xfer[i]) libusb_cancel_transfer(p->xfer[i]);
        if ((r = libusb_handle_events_timeout(p->ctx, &tv)) < 0) {
            fprintf( stderr, "libusb_handle_events returned: %d\n", r);
            if (r == LIBUSB_ERROR_INTERRUPTED) continue; /* stray */
            return -1;
        }
    }
    return 0;
}

/* A failed transfer ends the stream with an error: failed has absolute precedence
   while there is a stream, and read_async takes it up */
static void mirisdr_async_fail (mirisdr_dev_t *p) {
    int s;

    do {
        s = mirisdr_async_get(p);
        if ((s == MIRISDR_ASYNC_INACTIVE) || (s == MIRISDR_ASYNC_FAILED)) return;
    } while (!mirisdr_async_move(p, s, MIRISDR_ASYNC_FAILED));
}

/* the transfer callback, which passes the data on */
static void LIBUSB_CALL _libusb_callback (struct libusb_transfer *xfer) {
    size_t i;
    int len, bytes = 0, s;
    unsigned char *iso_packet_buf;
    mirisdr_dev_t *p = (mirisdr_dev_t*) xfer->user_data;
    uint8_t *samples;

    if (!p) return;

    /* back after its run gave up on it: the callback and its buffers are not this one's */
    if (p->xfer_abandoned) {
        mirisdr_atomic_add(&p->xfer_inflight, -1);
        return;
    }

    samples = p->samples;

    /* only a completed transfer is processed */
    if (xfer->status == LIBUSB_TRANSFER_COMPLETED) {
        p->stats_head = 1;
        p->conv_samples = p->stats.samples;
        p->conv_filled = 0;
        p->fills_n = 0;
        /*
         * Find the right buffer size. This has to be done in one step, otherwise
         * the format can change halfway through; the other option is a lock.
         */
        switch (xfer->type) {
        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
            switch (p->format) {
            case MIRISDR_FORMAT_252_S16:
                if (!(samples = samples_realloc(p, 504 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];

                    /* buffer_simple is only for packets of the same size */
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        /* less than 3072 is fine as a multiple of 1024, a partial 1024 is dropped */
                        len = mirisdr_samples_convert_252_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_336_S16:
                if (!(samples = samples_realloc(p, 672 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_336_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_384_S16:
                if (!(samples = samples_realloc(p, 768 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_384_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_504_S16:
                if (!(samples = samples_realloc(p, 1008 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_504_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_504_S8:
                if (!(samples = samples_realloc(p, 1008 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_504_s8(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_504_REAL_S16:
                if (!(samples = samples_realloc(p, 504 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_252_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_672_REAL_S16:
                if (!(samples = samples_realloc(p, 672 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_336_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_768_REAL_S16:
                if (!(samples = samples_realloc(p, 768 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_384_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            }
            break;
        case LIBUSB_TRANSFER_TYPE_BULK:
            if (!(samples = samples_realloc(p, (DEFAULT_BULK_BUFFER / 1024 + 2) * MIRISDR_BLOCK_OUT_MAX))) goto failed;
            if (p->fw_ours)
                bytes = mirisdr_parse_bulk(p, xfer->buffer, xfer->actual_length, xfer->length, samples);
            else
                bytes = mirisdr_convert_bulk(p, xfer->buffer, samples, xfer->actual_length);
            break;
        default:
            fprintf( stderr, "not isoc or bulk transfer type on usb device: %u\n", p->index);
            goto failed;
        }

        if (bytes > 0 || p->fills_n) mirisdr_feed_converted(p, samples, bytes);
        /* draining: the transfer is done, and should not be reused. Starting (or
           restarting) it goes back in, or isochronous ones completing early drop out */
        s = mirisdr_async_get(p);
        if (p->xfer_draining || ((s != MIRISDR_ASYNC_RUNNING) && (s != MIRISDR_ASYNC_STARTING) &&
                                 (s != MIRISDR_ASYNC_PAUSED))) goto done;

        /* with the stamp: back onto the grid once the queue has drained of the old phase */
        if ((xfer->type == LIBUSB_TRANSFER_TYPE_BULK) && p->fw_ours)
        {
            xfer->length = DEFAULT_BULK_BUFFER;
            if (p->bulk_wait > 0) {
                p->bulk_wait--;
            } else if ((xfer->length = mirisdr_bulk_next_len(p)) != DEFAULT_BULK_BUFFER) {
                p->bulk_wait = (int) p->xfer_buf_num;
            }
        }

        /* without the stamp, find the grid from the counters and shift the transfers onto it */
        if ((xfer->type == LIBUSB_TRANSFER_TYPE_BULK) && !p->fw_ours)
        {
            if(p->sync_run > (int)p->xfer_buf_num)
            {
                /* Rarely, packets containing a count of zero are received, which should not
                 * be seen as desync */
                int phase = mirisdr_bulk_phase(p, xfer->buffer, xfer->actual_length);

                /* A shift only reaches the stream once the transfers already
                   queued behind it have drained, and every one of those still
                   carries the old phase. Counting that wait in transfers is
                   a bug: sync_run counts blocks, and one transfer holds
                   sixteen of them, so the next buffer re-triggered the shift
                   and it oscillates. */
                p->sync_run = -(int) (p->xfer_buf_num * (DEFAULT_BULK_BUFFER / 1024));

                /* Blocks start at phase in every transfer, so one transfer
                   1024 - phase short puts the next on the grid. */
                if (phase > 0) {
                    p->stats.resyncs++;
                    xfer->length = DEFAULT_BULK_BUFFER - 1024 + phase;
                    fprintf(stderr,"libmirisdr: block grid is %d bytes out, shifting.\n", phase);
                } else {
                    xfer->length = DEFAULT_BULK_BUFFER;
                }
            }else
                xfer->length = DEFAULT_BULK_BUFFER;
        }
                /* go on with the next transfer */
        if (libusb_submit_transfer(xfer) < 0) {
            fprintf( stderr, "error re-submitting URB on device %u\n", p->index);
            goto failed;
        }

        /* in flight again */
        return;
    } else if (xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        fprintf( stderr, "error async transfer status %d on device %u\n", xfer->status, p->index);
        goto failed;
    }

done:
    /* back for good: only now, so a drain elsewhere waits until this is through */
    mirisdr_atomic_add(&p->xfer_inflight, -1);
    return;

failed:
    mirisdr_atomic_add(&p->xfer_inflight, -1);
    mirisdr_async_fail(p);
}

/* cancel the async part: one step, so the stream that runs takes it, or none runs */
int mirisdr_cancel_async (mirisdr_dev_t *p) {
    int s;

    if (!p) goto failed;

    for (;;) {
        switch (s = mirisdr_async_get(p)) {
        case MIRISDR_ASYNC_CANCELING:
        case MIRISDR_ASYNC_FAILED:
            return 0;           /* already on its way out: as asked */
        case MIRISDR_ASYNC_INACTIVE:
            goto failed;        /* not -2, which is MIRISDR_REOPEN */
        }

        /* starting, running or paused: the run stops it */
        if (mirisdr_async_move(p, s, MIRISDR_ASYNC_CANCELING)) return 0;
    }

failed:
    return -1;
}

/* cancel the async part and wait until read_async has returned. Not from
   the callback, which would wait for itself */
int mirisdr_cancel_async_now (mirisdr_dev_t *p) {
    if (!p) return -1;

    mirisdr_cancel_async(p);

    /* loop until everything has stopped */
    while (mirisdr_async_get(p) != MIRISDR_ASYNC_INACTIVE)
#if defined (_WIN32) && !defined(__MINGW32__)
    Sleep(20);
#else
    usleep(20000);
#endif

    return 0;
}

/* allocate the async buffers */
/* Byte reads from a usbfs buffer cost less than a nanosecond where the controller
   is IO coherent and hundreds where it is not. The platform does not expose this
   information to the application, so we have to measure it. */
#define MIRISDR_PROBE_BYTES     2048
#define MIRISDR_PROBE_ROUNDS    5
#define MIRISDR_PROBE_RATIO     8

static double mirisdr_read_cost (const volatile uint8_t *buf, int len, int rounds) {
    volatile uint32_t sum = 0;
    double best = 0;
    int i, r;

    for (i = 0; i < len; i++) sum+= buf[i];        /* map the pages first */

    for (r = 0; r < rounds; r++) {
        struct timespec a, b;
        double ns;

        clock_gettime(CLOCK_MONOTONIC, &a);
        for (i = 0; i < len; i++) sum+= buf[i];
        clock_gettime(CLOCK_MONOTONIC, &b);

        ns = ((double) (b.tv_sec - a.tv_sec) * 1e9 + (b.tv_nsec - a.tv_nsec)) / len;

        if (!r || (ns < best)) best = ns;
    }

    return best;
}

/* Slow usbfs buffers are swapped for malloc'd ones: usbfs then copies in the
   kernel, instead of the converters (or a memcpy) reading uncached memory. */
static void mirisdr_probe_buffers (mirisdr_dev_t *p) {
    uint8_t *cached;
    double slow, fast;
    size_t i;

    p->xfer_buf_slow = 0;

    if (!p->xfer_buf_devmem || !p->xfer_buf || !p->xfer_buf[0]) return;
    if (!(cached = malloc(MIRISDR_PROBE_BYTES))) return;

    memset(cached, 0, MIRISDR_PROBE_BYTES);

    fast = mirisdr_read_cost(cached, MIRISDR_PROBE_BYTES, MIRISDR_PROBE_ROUNDS);
    slow = mirisdr_read_cost(p->xfer_buf[0], MIRISDR_PROBE_BYTES, MIRISDR_PROBE_ROUNDS);

    free(cached);

    p->xfer_buf_slow = slow > fast * MIRISDR_PROBE_RATIO;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "transfer buffers: %.1f ns/byte against %.1f cached, %s\n",
            slow, fast, p->xfer_buf_slow ? "using malloc buffers" : "read in place");
#endif

    if (!p->xfer_buf_slow) return;

    for (i = 0; i < p->xfer_buf_num; i++) {
        libusb_dev_mem_free(p->dh, p->xfer_buf[i], p->xfer_buf_size);
        p->xfer_buf[i] = NULL;
    }

    p->xfer_buf_devmem = 0;

    /* a failure leaves NULL, which the caller checks */
    for (i = 0; i < p->xfer_buf_num; i++)
        p->xfer_buf[i] = malloc(p->xfer_buf_size);
}

static int mirisdr_async_free (mirisdr_dev_t *p);

static int mirisdr_async_alloc (mirisdr_dev_t *p) {
    size_t i;

    if (!p->xfer) {
        if (!(p->xfer = calloc(p->xfer_buf_num, sizeof(*p->xfer)))) goto failed;
        p->xfer_kind = p->transfer;
        p->xfer_alt = p->alt_setting;

        for (i = 0; i < p->xfer_buf_num; i++) {
            switch (p->transfer) {
            case MIRISDR_TRANSFER_BULK:
                p->xfer[i] = libusb_alloc_transfer(0);
                break;
            case MIRISDR_TRANSFER_ISOC:
                p->xfer[i] = libusb_alloc_transfer(DEFAULT_ISO_PACKETS);
                break;
            }
            if (!p->xfer[i]) goto failed;
        }
    }

    if (!p->xfer_buf) {
        size_t j, bufsz;

        if (!(p->xfer_buf = calloc(p->xfer_buf_num, sizeof(*p->xfer_buf)))) goto failed;

        switch (p->transfer) {
        case MIRISDR_TRANSFER_ISOC:
            bufsz = (size_t) DEFAULT_ISO_BUFFER * mirisdr_burst(p) * DEFAULT_ISO_PACKETS;
            break;
        default:
            bufsz = DEFAULT_BULK_BUFFER;
            break;
        }
        p->xfer_buf_size = bufsz;

        /* Prefer usbfs DMA-coherent buffers: saves ~5% CPU on N150 */
        p->xfer_buf_devmem = (getenv("MIRISDR_NO_ZEROCOPY") == NULL);
        for (i = 0; p->xfer_buf_devmem && i < p->xfer_buf_num; i++) {
            p->xfer_buf[i] = libusb_dev_mem_alloc(p->dh, bufsz);
            if (!p->xfer_buf[i]) { p->xfer_buf_devmem = 0; break; }
        }
        if (!p->xfer_buf_devmem) {
            for (j = 0; j < i; j++) {
                libusb_dev_mem_free(p->dh, p->xfer_buf[j], bufsz);
                p->xfer_buf[j] = NULL;
            }
            for (i = 0; i < p->xfer_buf_num; i++)
                if (!(p->xfer_buf[i] = malloc(bufsz))) goto failed;
        }

        mirisdr_probe_buffers(p);

        for (i = 0; i < p->xfer_buf_num; i++)
            if (!p->xfer_buf[i]) goto failed;
    }

    if ((!p->xfer_out) &&
        (p->user_out_len)) {
        if (!(p->xfer_out = malloc(p->user_out_len * sizeof(*p->xfer_out)))) goto failed;
    }

    return 0;

failed:
    fprintf(stderr, "libmirisdr: out of memory for the transfer buffers\n");
    mirisdr_async_free(p);
    return -1;
}

/* free the async buffers */
/* The only place the transfers are freed: under the lock, so not while a stop or a
   restart in another thread is using them */
static int mirisdr_async_free (mirisdr_dev_t *p) {
    size_t i;

    mirisdr_xfer_lock(p);

    if (p->xfer) {
        for (i = 0; i < p->xfer_buf_num; i++) {
            if (p->xfer[i]) libusb_free_transfer(p->xfer[i]);
        }

        free(p->xfer);
        p->xfer = NULL;
    }

    if (p->xfer_buf) {
        for (i = 0; i < p->xfer_buf_num; i++) {
            if (!p->xfer_buf[i]) continue;
            if (p->xfer_buf_devmem)
                libusb_dev_mem_free(p->dh, p->xfer_buf[i], p->xfer_buf_size);
            else
                free(p->xfer_buf[i]);
        }

        free(p->xfer_buf);
        p->xfer_buf = NULL;
    }

    if (p->xfer_out) {
        free(p->xfer_out);
        p->xfer_out = NULL;
    }

    mirisdr_xfer_unlock(p);

    return 0;
}

/* The end of a run: every transfer back, then freed, in one hold of the lock. A stop
   or restart under way finishes first; it fails on the canceled state, so the
   transfers it put in are drained here too. -1: they would not come back, and are
   left to libusb */
static int mirisdr_async_finish (mirisdr_dev_t *p) {
    int r = 0;

    mirisdr_xfer_lock(p);

    if (p->xfer) {
        /* Ask the firmware to stop while the transfers are still queued, so the
         * engine can drain */
        mirisdr_streaming_stop(p);

        /* A stalled endpoint may never return its transfers, so do not wait
         * for them for ever -- that hangs the caller's stream thread. Give
         * up after a few seconds and leave without freeing: the transfers
         * are still owned by libusb, and freeing an in-flight transfer
         * corrupts memory. The next run tries them again, and refuses to
         * start while they are out, and mirisdr_close() frees them once the
         * handle has taken them off libusb's list. */
        if (mirisdr_async_drain(p, 5) < 0) {
            fprintf(stderr, "libmirisdr: transfers would not cancel, "
                            "abandoning them\n");
            p->xfer_abandoned = 1;
            r = -1;
        }
        /* every completion is in: nothing of ours is on libusb's list */
        else mirisdr_async_free(p);
    }

    mirisdr_xfer_unlock(p);

    return r;
}

/* One transfer filled in for the transfer mode in force, at its full length */
static int mirisdr_async_fill (mirisdr_dev_t *p, size_t i) {
    switch (p->transfer) {
    case MIRISDR_TRANSFER_BULK:
        libusb_fill_bulk_transfer(p->xfer[i],
                                  p->dh,
                                  0x81,
                                  p->xfer_buf[i],
                                  DEFAULT_BULK_BUFFER,
                                  _libusb_callback,
                                  (void*) p,
                                  DEFAULT_BULK_TIMEOUT);
        return 0;
    case MIRISDR_TRANSFER_ISOC:
        libusb_fill_iso_transfer(p->xfer[i],
                                 p->dh,
                                 0x81,
                                 p->xfer_buf[i],
                                 DEFAULT_ISO_BUFFER * mirisdr_burst(p) * DEFAULT_ISO_PACKETS,
                                 DEFAULT_ISO_PACKETS,
                                 _libusb_callback,
                                 (void*) p,
                                 DEFAULT_ISO_TIMEOUT);
        libusb_set_iso_packet_lengths(p->xfer[i], DEFAULT_ISO_BUFFER * mirisdr_burst(p));
        return 0;
    }

    fprintf( stderr, "unsupported transfer type\n");
    return -1;
}

/* The interface's alternate setting for the transfer mode: bulk or the isochronous
   bandwidth. Not critical, the stream runs on whatever the endpoint gives */
static void mirisdr_async_alt (mirisdr_dev_t *p) {
    int r;

    if ((r = libusb_set_interface_alt_setting(p->dh, 0, p->alt_setting)) < 0)
        fprintf( stderr, "failed to use alternate setting %u for %s mode on miri usb device %u with code %d\n",
                 p->alt_setting, (p->transfer == MIRISDR_TRANSFER_BULK) ? "Bulk" : "Isochronous", p->index, r);
}

/* The parser and the filters from the start of a stream: what came before does not
   go on, and the first blocks of a restart may still be the old stream's */
static void mirisdr_async_rewind (mirisdr_dev_t *p) {
    p->sync_run = 0;
    p->bulk_carry_n = 0;
    p->bulk_lost = 0;
    p->bulk_wait = 0;
    p->addr_restart = 1;
    p->addr_dropped = 0;
}

/* run the async part */
static int mirisdr_read_async_run (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len);

/* From inactive to starting in one step, so only one stream starts. A cancel from
   then on is kept: it moves the state to canceling, for the run to take up once its
   transfers are in. The state is inactive again only when this returns */
int mirisdr_read_async (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len) {
    int r;

    if (!p) return -1;
    if (!mirisdr_async_move(p, MIRISDR_ASYNC_INACTIVE, MIRISDR_ASYNC_STARTING)) return -1;

    r = mirisdr_read_async_run(p, cb, ctx, num, len);
    mirisdr_async_set(p, MIRISDR_ASYNC_INACTIVE);

    return r;
}

static int mirisdr_read_async_run (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len) {
    size_t i;
    int r;
    int transfer_failed = 0;
    struct timeval tv = {1, 0};

    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* a run before gave up on its transfers: they are made for its count and sizes, and
       are reused only once all of them are back */
    if (p->xfer_abandoned) {
        /* still marked while they come in, so they are only counted */
        mirisdr_xfer_lock(p);
        if ((r = mirisdr_async_drain(p, 2)) == 0) {
            mirisdr_async_free(p);
            p->xfer_abandoned = 0;
        }
        mirisdr_xfer_unlock(p);

        if (r < 0) {
            fprintf(stderr, "libmirisdr: the transfers of the last stream are still out, "
                            "no stream until the device is closed\n");
            goto failed;
        }
    }

    p->cb = cb;
    p->cb_ctx = ctx;

    p->xfer_buf_num = (num == 0) ? DEFAULT_BUF_NUMBER : num;
    /* a fixed output buffer size; baseband keeps its own */
    p->user_out_len = len;
    p->xfer_out_len = p->bb ? 0 : len;
    mirisdr_bb_start(p);
    p->xfer_out_pos = 0;
#if MIRISDR_DEBUG >= 1
    fprintf( stderr, "async read on device %u, buffers: %lu, output size: ",
                                p->index, (long)p->xfer_buf_num);
    if (p->xfer_out_len) {
        fprintf( stderr, "%lu", (long)p->xfer_out_len);
    } else {
        fprintf( stderr, "auto");
    }
#endif
    memset(&p->stats, 0, sizeof(p->stats));
    p->count_index = p->count_gaps = 0;
    p->count_addr = 0;
    p->sync_run = 0;
    p->addr_valid = 0;
    p->bulk_carry_n = 0;
    p->bulk_lost = 0;
    p->bulk_wait = 0;
    p->gap_track = 1;
    p->gapq_n = 0;
    p->fills_n = 0;
    p->fed_bytes = 0;
    p->cb_bytes = 0;
    p->cb_base = 0;
    p->cb_lost = 0;
    p->ev_valid = 0;
    p->sync_ready = 0;
#if MIRISDR_DEBUG >= 1
    fprintf( stderr, ", transfer: %s\n", (p->transfer == MIRISDR_TRANSFER_BULK) ? "bulk" : "isochronous");
#endif
    /* use the right interface setting for the data, not critical */
    mirisdr_async_alt(p);

    if (mirisdr_async_alloc(p) < 0) goto failed;

    /* submit the transfers */
    for (i = 0; i < p->xfer_buf_num; i++) {
        if (mirisdr_async_fill(p, i) < 0) goto failed_free;

        r = libusb_submit_transfer(p->xfer[i]);
        if (r < 0) {
            fprintf(stderr, "Failed to submit transfer %lu reason: %d\n", i, r);
            /* the ones before it are in flight: get them back first */
            if (mirisdr_async_drain(p, 5) < 0) { p->xfer_abandoned = 1; goto failed; }
            goto failed_free;
        }
        mirisdr_atomic_add(&p->xfer_inflight, 1);
    }

    /* start streaming. A cancel while starting has made it canceling. Not started: the
       transfers come back through the failed state's drain */
    p->xfer_draining = 0;
    mirisdr_iso_settle(p);
    if (mirisdr_streaming_start(p) < 0) {
        fprintf(stderr, "libmirisdr: the stream would not start\n");
        mirisdr_async_fail(p);
    }
    mirisdr_async_move(p, MIRISDR_ASYNC_STARTING, MIRISDR_ASYNC_RUNNING);

    for (;;) {
        int s = mirisdr_async_get(p);

        if (s == MIRISDR_ASYNC_FAILED) {
            /* Do NOT free the transfers here: on this path some of them are
             * still submitted, and libusb_free_transfer() on an in-flight
             * transfer corrupts memory (it crashed the host process every time
             * a bulk endpoint stalled). Convert the failure into a normal
             * cancel so the branch below drains every transfer properly, and
             * remember that it failed so we still return an error. */
            transfer_failed = 1;
            if (!mirisdr_async_move(p, MIRISDR_ASYNC_FAILED, MIRISDR_ASYNC_CANCELING)) continue;
            s = MIRISDR_ASYNC_CANCELING;
        }

        /* stopping: drained and freed under the lock, after a stop or
           restart in another thread is through with the transfers */
        if (s == MIRISDR_ASYNC_CANCELING) {
            if (mirisdr_async_finish(p) < 0) return -1;
            break;
        }

        /* wait for the next event; on an error the transfers are still out, so
           they are drained as for a failed one, not freed */
        if ((r = libusb_handle_events_timeout(p->ctx, &tv)) < 0) {
            fprintf( stderr, "libusb_handle_events returned: %d\n", r);
            if (r == LIBUSB_ERROR_INTERRUPTED) continue; /* stray */
            mirisdr_async_fail(p);
        }
    }

    /* free the buffers */
    mirisdr_async_free(p);

    /* stop streaming */
#if defined (_WIN32) && !defined(__MINGW32__)
    Sleep(20);
#else
    usleep(20000);
#endif
    mirisdr_streaming_stop(p);
    /* stopping the ADC too would be better, but then the next open would have to initialise it */

    return transfer_failed ? -1 : 0;

failed_free:
    mirisdr_async_free(p);

failed:
    return -1;
}

/* start streaming */
static int mirisdr_start_async_held (mirisdr_dev_t *p) {
    size_t i;

    /* only from paused */
    if (mirisdr_async_get(p) != MIRISDR_ASYNC_PAUSED) goto failed;

        /* reset the internal buffer */
    p->xfer_out_pos = 0;

    /* all of it before the transfers go out: their completions feed the filters */
    mirisdr_bb_restart(p);
    mirisdr_async_rewind(p);

    /* another transfer mode: other transfers, on another alternate setting. None is
       in flight while paused */
    if (p->xfer && ((p->xfer_kind != (int) p->transfer) || (p->xfer_alt != p->alt_setting))) {
        mirisdr_async_free(p);
        mirisdr_async_alt(p);
        if (mirisdr_async_alloc(p) < 0) goto failed;
    }

    if (!p->xfer) goto failed;

    p->xfer_draining = 0;

    for (i = 0; i < p->xfer_buf_num; i++) {
        if (!p->xfer[i] || (mirisdr_async_fill(p, i) < 0)) continue;
        if (libusb_submit_transfer(p->xfer[i])< 0) {
            /* still paused: the ones before it come back, and stay */
            p->xfer_draining = 1;
            mirisdr_async_drain(p, 5);
            goto failed;
        }
        mirisdr_atomic_add(&p->xfer_inflight, 1);
    }

    if (mirisdr_async_get(p) != MIRISDR_ASYNC_PAUSED) goto failed;

    mirisdr_iso_settle(p);
    if (mirisdr_streaming_start(p) < 0) {
        p->xfer_draining = 1;
        mirisdr_async_drain(p, 5);
        goto failed;
    }

    /* a cancel meanwhile is the run's to finish, these transfers with it */
    if (!mirisdr_async_move(p, MIRISDR_ASYNC_PAUSED, MIRISDR_ASYNC_RUNNING)) goto failed;

    return 0;

failed:
    return -1;
}

/* The buffer being filled is dropped at a restart, and the next format may take
   other bytes a sample: count what was not handed out as lost and start the byte
   positions again. The buffers' sample and index carry on across it. */
static void mirisdr_cb_rebase (mirisdr_dev_t *p) {
    uint64_t ub = mirisdr_unit_bytes(p), dropped = p->xfer_out_pos / ub, filled = 0, missing = 0;
    int k;

    for (k = 0; k < p->gapq_n; k++) {
        filled += p->gapq[k].filled;
        missing += p->gapq[k].samples;
    }

    /* the real samples of the dropped part, and all of the gaps not reached */
    p->cb_lost += (dropped > filled ? dropped - filled : 0) + missing;

    /* the stats count what was delivered: the dropped part, fill included, was not.
       Their index (samples + lost) stays */
    if (dropped > p->stats.samples) dropped = p->stats.samples;
    if (filled > dropped) filled = dropped;
    if (filled > p->stats.filled) filled = p->stats.filled;
    p->stats.samples -= dropped;
    p->stats.lost += dropped;
    p->stats.filled -= filled;
    p->cb_base += p->cb_bytes / ub;
    p->cb_bytes = 0;
    p->fed_bytes = 0;
    p->gapq_n = 0;
    p->xfer_out_pos = 0;
}

/* stop streaming */
static int mirisdr_stop_async_held (mirisdr_dev_t *p) {

    /* only from running */
    if (mirisdr_async_get(p) != MIRISDR_ASYNC_RUNNING) goto failed;

    /* ask the firmware to stop while transfers are still queued, as otherwise the ISR never
     * fires, and we can't disable the capture engine */
    mirisdr_streaming_stop(p);

    /* The firmware sends the buffer it holds when it stops, as two 512 byte packets.
       Cancelling between them leaves the second in its FIFO, to start the next
       stream half a block out. */
#if defined(_WIN32) && !defined(__MINGW32__)
    Sleep(2);
#else
    usleep(2000);
#endif

    /* every transfer back from libusb before the pause is declared. They would not come
       back: no longer resubmitted, the stream would stay silent, so it ends with an error */
    if (mirisdr_async_drain(p, 10) < 0) {
        mirisdr_async_fail(p);
        goto failed;
    }

    if (mirisdr_async_get(p) != MIRISDR_ASYNC_RUNNING) goto failed;

    mirisdr_cb_rebase(p);

    /* a cancel meanwhile is the run's to finish */
    if (!mirisdr_async_move(p, MIRISDR_ASYNC_RUNNING, MIRISDR_ASYNC_PAUSED)) goto failed;

    return 0;

failed:
    return -1;
}

/* Both hold the transfers' lock throughout, so a cancel meanwhile cannot have the run
   free the transfers under them; the run frees them once they are through */
int mirisdr_start_async (mirisdr_dev_t *p) {
    int r;

    if (!p) return -1;

    mirisdr_xfer_lock(p);
    r = mirisdr_start_async_held(p);
    mirisdr_xfer_unlock(p);

    return r;
}

/* A restart after a pause by a setting: one that would not go ends the stream with an
   error, as one left paused would wait for ever and the app never paused it */
static int mirisdr_stream_resume (mirisdr_dev_t *p) {
    if (mirisdr_start_async(p) == 0) return 0;

    if (mirisdr_async_get(p) == MIRISDR_ASYNC_PAUSED) mirisdr_async_fail(p);
    return -1;
}

int mirisdr_stop_async (mirisdr_dev_t *p) {
    int r;

    if (!p) return -1;

    mirisdr_xfer_lock(p);
    r = mirisdr_stop_async_held(p);
    mirisdr_xfer_unlock(p);

    return r;
}

int mirisdr_get_buffer_info (mirisdr_dev_t *p, mirisdr_buffer_info_t *info) {
    if (!p || !info) return -1;

    *info = p->cb_info;

    return 0;
}

int mirisdr_set_gap_fill (mirisdr_dev_t *p, int on) {
    if (!p) return -1;

    /* always on under baseband, where the filters need time to pass */
    p->stream.gap_fill = !!on;
    p->gap_fill = !!on || (p->bb_path != MIRISDR_BASEBAND_OFF);

    return 0;
}
