/*
 * At some very high sample rates the index counter is not exact. I assume this
 * is because the DSP block takes a snapshot a free running counter. These are
 * not lost samples, as we have tested that the stream always makes up for them.
 * (eg sometimes -1, next packet 0, then +1, ...). It happens, very rarely, at lower
 * rates as well.
 */
#define MIRISDR_ADDR_JITTER 4

/* After a start the device can still hold blocks of the stream before, at times
   with a header cleared or overwritten: up to 4 kB in the sample engine and 2x
   4 kB in the USB FIFO. Those 12 blocks are dropped and the next starts the count */
#define MIRISDR_ADDR_DROP 12

/* Output bytes a sample, I/Q pair or real value, takes */
static uint32_t mirisdr_unit_bytes (mirisdr_dev_t *p) {
    switch (p->format) {
    case MIRISDR_FORMAT_504_S8:
    case MIRISDR_FORMAT_504_REAL_S16:
    case MIRISDR_FORMAT_672_REAL_S16:
    case MIRISDR_FORMAT_768_REAL_S16:
        return 2;
    default:
        return 4;
    }
}

/* A gap of 'missing' samples before the block being converted: queue it for the
   callback that delivers it, and the zeros for gap fill. Returns the samples filled. */
static uint64_t mirisdr_gap_note (mirisdr_dev_t *p, uint64_t missing) {
    uint64_t f = 0, ub = mirisdr_unit_bytes(p), at;
    int i;

    if (!p->gap_track) return 0;

    /* up to one bulk transfer's worth */
    if (p->gap_fill && p->fills_n < (int) (sizeof p->fills / sizeof p->fills[0])) {
        f = missing;
        if (f > 16 * (uint64_t) p->addr_step) f = 16 * (uint64_t) p->addr_step;
        p->fills[p->fills_n].off = (uint32_t) ((p->stats.samples - p->conv_samples - p->conv_filled) * ub);
        p->fills[p->fills_n].n = (uint32_t) f;
        p->fills_n++;
        p->conv_filled += f;
    }

    at = p->fed_bytes + (p->stats.samples - p->conv_samples) * ub;
    i = p->gapq_n;
    if (i == (int) (sizeof p->gapq / sizeof p->gapq[0])) {
        /* full: fold into the last, so the timeline still adds up */
        i--;
        p->gapq[i].samples += missing;
        p->gapq[i].filled += (uint32_t) f;
    } else {
        p->gapq[i].at = at;
        p->gapq[i].samples = missing;
        p->gapq[i].filled = (uint32_t) f;
        p->gapq_n++;
    }

    return f;
}

/* Called once per 1024 byte block with its header, which carries the sample
   counter and the IR block last completed run (if enabled). */
/* returns 1 for a block to drop */
static int mirisdr_addr_next (mirisdr_dev_t *p, const uint8_t *hdr, uint32_t step) {
    uint32_t addr = hdr[3] << 24 | hdr[2] << 16 | hdr[1] << 8 | hdr[0] << 0;
    int32_t d;

    if (p->addr_restart) {
        if (p->addr_dropped++ < MIRISDR_ADDR_DROP) return 1;

        /* the device counts again from its start */
        p->addr_restart = 0;
        if (p->addr_valid) {
            mirisdr_ir_resync(p);
            p->ev_valid = 0;
        }
        p->addr_valid = 0;
    }

    d = (int32_t) (addr - p->addr);

    if (!p->addr_valid) {
        p->addr_valid = 1;
    } else if ((d > MIRISDR_ADDR_JITTER) || (d < -MIRISDR_ADDR_JITTER)) {
        fprintf(stderr, "%d samples lost, %08x:%08x\n", d, p->addr, addr);
        p->stats.gaps++;
        p->sync_run++;
        mirisdr_ir_resync(p);
        p->ev_missed = 1;

        /* a counter that went backwards is a misaligned stream reading sample
           data as a header, not samples that went missing */
        if (d > 0) {
            uint64_t f = mirisdr_gap_note(p, (uint32_t) d);

            /* filled samples are delivered, at their own index */
            p->stats.lost += (uint32_t) d - f;
            p->stats.samples += f;
            p->stats.filled += f;
        }
    } else {
        if (d) p->stats.jitter++;
        p->sync_run = 0;
    }

    if (p->stats_head) {
        p->stats_head = 0;
        p->stats.index = p->stats.samples + p->stats.lost;
    }

    mirisdr_events_latch(p, hdr, p->stats.samples, p->stats.samples + p->stats.lost, step);

    p->stats.samples+= step;
    p->addr = addr + step;

    mirisdr_atomic_add(&p->count_seq, 1);
    p->count_index = p->stats.samples + p->stats.lost;
    p->count_addr = p->addr;
    p->count_gaps = p->stats.gaps;
    mirisdr_atomic_add(&p->count_seq, 1);

    mirisdr_ir_latch(p, hdr[6], addr);

    return 0;
}

#include "252_s16.c"
#include "336_s16.c"
#include "384_s16.c"
#include "504_s16.c"
#include "504_s8.c"

