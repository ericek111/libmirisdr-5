
/*
 * A block arrived whose sequence counter is not the one expected.
 *
 * Read the difference as signed. The counter counts samples, not blocks, so a
 * forward step of any size is a real gap of exactly that many samples -- the
 * device drops what it must and carries on counting, which is why the gaps are
 * routinely not whole blocks (168 and 360 are typical when isochronous is
 * saturated).
 *
 * A counter that has gone backwards is a different matter, and cannot happen
 * while the framing holds: it means what was read as a header is really sample
 * data, so the difference is a fiction. Unsigned arithmetic turns those into
 * values near 2^32, which is how a two second capture came to claim three
 * billion samples lost. Count the event, but attribute no samples to it.
 */
static void mirisdr_note_lost (mirisdr_dev_t *p, uint32_t addr, int cnt)
{
    int32_t delta = (int32_t) (addr - p->addr);

    if (delta > 0) {
        fprintf(stderr, "%d samples lost, %d, %08x:%08x\n", delta, cnt, p->addr, addr);
        p->lost_samples += (uint32_t) delta;
    } else {
        fprintf(stderr, "stream desynchronised, %d, %08x:%08x\n", cnt, p->addr, addr);
        p->desync_events++;
    }

    p->lost_events++;
    p->sync_loss_cnt++;
}

#include "252_s16.c"
#include "336_s16.c"
#include "384_s16.c"
#include "504_s16.c"
#include "504_s8.c"

