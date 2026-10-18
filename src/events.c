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

/*
 * The ADC watches the words written to the tuner port and keeps:
 *
 *    9:0   the gain fields of the last register 1 word (BBGAIN, MIXBU, MIXL, LNAGR)
 *    11    MMIO register 0x10 bit 21, which does nothing else, so a free marker
 *    12    MMIO register 7 bit 2, the output saturating instead of wrapping
 *    13    toggles on every tuner register 1 (gain) word
 *    14    toggles on every tuner register 2 (synthesizer) word
 *    15    toggles on every write to MMIO register 3, the sample rate PLL
 *
 * Bit 10 has never been seen set. The header is fixed when the packet starts, so a
 * change shows in the packet after the one it happened in.
 */

#define MIRISDR_HDR_MARK        (1 << 11)
#define MIRISDR_HDR_SATURATE    (1 << 12)
#define MIRISDR_HDR_GAIN        (1 << 13)
#define MIRISDR_HDR_TUNE        (1 << 14)
#define MIRISDR_HDR_RATE        (1 << 15)

#define MIRISDR_MARK_REG        0x10
#define MIRISDR_MARK_BIT        (1UL << 21)

static void mirisdr_scan_latch (mirisdr_dev_t *p, uint16_t w, uint16_t x, uint64_t sample, uint64_t index,
                                uint32_t step);

/* once per packet, before its samples are counted */
static void mirisdr_events_latch (mirisdr_dev_t *p, const uint8_t *hdr, uint64_t sample, uint64_t index,
                                  uint32_t step) {
    mirisdr_stream_event_t e;
    uint16_t w = hdr[4] | hdr[5] << 8, x;

    if (!p->ev_cb && !p->scan_on) return;

    if (!p->ev_valid) {
        p->ev_valid = 1;
        p->ev_missed = 0;
        p->ev_last = w;
        return;
    }

    x = w ^ p->ev_last;
    p->ev_last = w;

    /* under the lock mirisdr_scan_stop() takes, so a stopped scan is not read after */
    if (p->scan_on) {
        mirisdr_ev_lock(p);
        mirisdr_scan_latch(p, w, x, sample, index, step);
        mirisdr_ev_unlock(p);
    }
    if (!p->ev_cb) {
        p->ev_missed = 0;
        return;
    }

    memset(&e, 0, sizeof e);
    if (x & MIRISDR_HDR_GAIN) e.events |= MIRISDR_EVENT_GAIN;
    if (x & MIRISDR_HDR_TUNE) e.events |= MIRISDR_EVENT_TUNE;
    if (x & MIRISDR_HDR_RATE) e.events |= MIRISDR_EVENT_RATE;
    if (x & MIRISDR_HDR_MARK) e.events |= MIRISDR_EVENT_MARK;
    if (p->ev_missed) e.events |= MIRISDR_EVENT_MISSED;
    p->ev_missed = 0;

    if (!e.events) return;

    e.sample   = sample;
    e.index    = index;
    e.bb_gr    = w & 0x3f;
    e.mixbu    = (w >> 6) & 3;
    e.mixl     = (w >> 8) & 1;
    e.lna      = (w >> 9) & 1;
    e.mark     = !!(w & MIRISDR_HDR_MARK);
    e.saturate = !!(w & MIRISDR_HDR_SATURATE);
    e.raw      = w;

    /* the pair as one, and once the setter returns the old one no longer runs */
    mirisdr_ev_lock(p);
    if (p->ev_cb) p->ev_cb(&e, p->ev_ctx);
    mirisdr_ev_unlock(p);
}

int mirisdr_set_stream_events (mirisdr_dev_t *p, mirisdr_stream_event_cb_t cb, void *ctx) {
    if (!p) goto failed;

    mirisdr_ev_lock(p);
    p->ev_cb = cb;
    p->ev_ctx = ctx;
    p->ev_valid = 0;
    mirisdr_ev_unlock(p);

    return 0;

failed:
    return -1;
}

int mirisdr_set_stream_mark (mirisdr_dev_t *p, int on) {
    if (!p) goto failed;

    return mirisdr_write_reg(p, MIRISDR_MARK_REG, on ? MIRISDR_MARK_BIT : 0);

failed:
    return -1;
}
