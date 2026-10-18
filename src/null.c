/*
 * Copyright (C) 2026 by Bertold Van den Bergh <vandenbergh@bertold.org>
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

int mirisdr_open_null (mirisdr_dev_t **out, const char *format) {
    static const struct { const char *name; int format; uint32_t step; } f[] = {
        { "252_S16",      MIRISDR_FORMAT_252_S16,      252 },
        { "336_S16",      MIRISDR_FORMAT_336_S16,      336 },
        { "384_S16",      MIRISDR_FORMAT_384_S16,      384 },
        { "504_S16",      MIRISDR_FORMAT_504_S16,      504 },
        { "504_S8",       MIRISDR_FORMAT_504_S8,       504 },
        { "504_REAL_S16", MIRISDR_FORMAT_504_REAL_S16, 504 },
        { "672_REAL_S16", MIRISDR_FORMAT_672_REAL_S16, 672 },
        { "768_REAL_S16", MIRISDR_FORMAT_768_REAL_S16, 768 },
    };
    mirisdr_dev_t *p;
    unsigned i;

    if (!out || !format) return -1;

    for (i = 0; i < sizeof f / sizeof f[0]; i++)
        if (!strcmp(format, f[i].name)) break;
    if (i == sizeof f / sizeof f[0]) return -1;

    if (!(p = calloc(1, sizeof *p))) return -1;
    mirisdr_xfer_lock_init(p);              /* mirisdr_close() destroys it */

    p->format = f[i].format;  /* the enum is declared in the struct */
    p->addr_step = f[i].step;
    p->fw_ours = 1;             /* stamped blocks */
    p->fake = 1;                /* the configs can be applied, and read back */
    p->gap_track = 1;
    p->stream.rate = p->rate = DEFAULT_RATE;    /* so get_stream then set_stream works */
    mirisdr_tuner_defaults(p);   /* so tunes and scan lists come out as on a device */
    mirisdr_tuner_ctx_init(p);

    *out = p;

    return 0;
}

int mirisdr_feed_bulk (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t buf_len,
                       const uint8_t *data, uint32_t n) {
    uint8_t *samples;
    int bytes;

    if (!p || p->dh || !cb || (!data && n)) return -1;
    if (n > (1u << 24)) return -1;          /* in pieces: the parser counts in int */

    /* the callback's buffer size is fixed by the first call, as read_async's is */
    if (!p->cb) {
        p->cb = cb;
        p->user_out_len = buf_len;
        p->xfer_out_len = p->bb ? 0 : buf_len;
        if (buf_len && !(p->xfer_out = malloc(buf_len))) return -1;
        mirisdr_bb_start(p);
    }
    p->cb_ctx = ctx;

    if (!(samples = samples_realloc(p, (n / 1024 + 2) * MIRISDR_BLOCK_OUT_MAX))) return -1;

    p->stats_head = 1;
    p->conv_samples = p->stats.samples;
    p->conv_filled = 0;
    p->fills_n = 0;

    /* as a transfer of the length the library would ask for next */
    bytes = mirisdr_parse_bulk(p, data, (int) n, mirisdr_bulk_next_len(p), samples);
    if (bytes > 0 || p->fills_n) mirisdr_feed_converted(p, samples, (uint32_t) bytes);

    return 0;
}
