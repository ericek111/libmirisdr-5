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
 * The remote control receiver input on GPIO_3.
 *
 * The hardware measures pulse widths and nothing else, leaving the protocol to
 * the application.  Register 0x0A bit 16 runs it and the low 16 bits are the
 * prescaler, the counter clocking at 24 MHz / (N + 1).  What it produces
 * arrives in the stream rather than over the control endpoint, in header byte 6
 * of every 1024 byte packet, and that byte carries two different things:
 *
 *   - when a run ends, its level in bit 7 - set when the input was low - and
 *     its length in ticks in the low seven bits;
 *   - when the run in progress reaches 128 ticks without ending, the live input
 *     level in bit 7 and 127 in the low seven bits.  The counter then restarts,
 *     so this repeats every 128 ticks until the level finally changes, and the
 *     length latched at that point is only the time since the last one.
 *
 * So 127 is not a length, it is "still going, here is the level", and a run
 * longer than full scale has to be timed with the help of the header's sample
 * counter, which is exact and has no range limit.  How, and why that costs
 * nothing in resolution, is below.
 *
 * That changes what the prescaler is for.  It does not have to be large enough
 * to hold the longest run of interest, only large enough that a counter cycle
 * spans more than two packets, which is what keeps the number of restarts
 * recoverable; past that, smaller is simply finer.  The slowest packet the
 * hardware allows is 768 samples at 1.3 Msps, 591 us, so a tick above 9.2 us
 * works at every rate and format.
 *
 * The other extreme is useful too: with the tick at its minimum every run of
 * any length passes full scale almost at once, so byte 6 becomes a plain level
 * sample once per packet and transitions are timed by the stream alone - which
 * is enough to read a UART, or anything else slower than a packet.
 *
 * The one real limit is that byte 6 arrives once per packet, so a run shorter
 * than a packet period can be overwritten before it is ever seen.  At 8 Msps
 * and 252 samples that is 31.5 us.  It is also why this only works while
 * streaming.
 *
 * A run longer than the counter is still measured exactly.  The count latched
 * when it ends is its tail, restarts are exactly 128 ticks apart, and the
 * header's sample counter says how much time actually passed - so the number of
 * restarts is the only unknown and rounding recovers it.  That is the whole of
 * the arithmetic below, and it is worth more than it looks: it gives the length
 * to within a tick rather than to within a packet, for runs of any length.
 *
 * One callback per packet, always describing the run in progress and, on the
 * packet where it ends, its final length.  Reporting only completed runs would
 * be less to explain and worse to use, because a run is only known to have
 * ended when the next edge arrives: a UART character that ended in ones, or the
 * gap between two remote control frames, would never be reported at all, and
 * nothing downstream could tell either from a receiver that had gone quiet.
 */

/* clocked from the 24 MHz reference */
#define MIRISDR_IR_CLOCK        24000000

int mirisdr_set_ir (mirisdr_dev_t *p, uint32_t tick_ns, mirisdr_ir_cb_t cb, void *ctx)
{
    uint32_t n;

    if (!p) return -1;

    /* GPIO_3 is the SDRplay expander's chip select: released, it would float */
    if (MIRISDR_HW_EXPANDER(p->hw_flavour)) return -1;

    /* under the events' lock, which the stream thread holds while it times a packet: the
       callback and its context go together, and the old one has stopped once this is set */
    mirisdr_ev_lock(p);
    p->ir_tick_ns = 0;
    p->ir_cb = cb;
    p->ir_ctx = ctx;
    p->ir_have = 0;
    mirisdr_ev_unlock(p);

    if (!tick_ns)
    {
        if (mirisdr_write_reg(p, 0x0A, 0) < 0) return -1;

        return mirisdr_release_gpio(p, MIRISDR_IR_GPIO);
    }

    /* 24 MHz / (N + 1), so one clock is N = 0 */
    n = (uint32_t) (((uint64_t) tick_ns * 24 + 500) / 1000);
    if (n < 1) n = 1;
    if (n > 0x10000) n = 0x10000;
    n--;

    mirisdr_ev_lock(p);
    p->ir_tick_ns = (uint32_t) (((uint64_t) (n + 1) * 1000000000ULL
                                 + MIRISDR_IR_CLOCK / 2) / MIRISDR_IR_CLOCK);
    mirisdr_ev_unlock(p);

    /* the band plan drives every pin as an output and the sampler reads the
       pad, so the input has to be released before any of this means anything */
    if (mirisdr_set_gpio_direction(p, MIRISDR_IR_GPIO, 0) < 0) return -1;

    return mirisdr_write_reg(p, 0x0A, (1UL << 16) | n);
}

uint32_t mirisdr_get_ir (mirisdr_dev_t *p)
{
    return p ? p->ir_tick_ns : 0;
}

/* Lost packets mean the run in progress cannot be timed, so the next packet
   starts a new one rather than reporting a length that spans the hole. */
static void mirisdr_ir_resync (mirisdr_dev_t *p)
{
    p->ir_have = 0;
}

/* One packet header, from the transfer callback, under the events' lock */
static void mirisdr_ir_latch_held (mirisdr_dev_t *p, uint8_t b6, uint32_t addr)
{
    uint8_t ticks = b6 & 0x7F;
    uint8_t level = (b6 & 0x80) ? 0 : 1;        /* the byte sets the bit for a low pin */
    uint32_t tick_ns = p->ir_tick_ns;
    mirisdr_ir_cb_t cb = p->ir_cb;
    void *ctx = p->ir_ctx;
    uint64_t full = (uint64_t) (MIRISDR_IR_KEEPS_GOING + 1) * tick_ns;
    uint64_t packet, elapsed, ns;
    mirisdr_ir_pulse_t e;

    if (!tick_ns || !cb || !p->rate) return;

    if (!p->ir_have)
    {
        p->ir_have = 1;
        p->ir_partial = 1;                      /* its beginning was not seen */
        /* a count means the run before this one ended, so the level now is the
           other one */
        p->ir_level = (ticks == MIRISDR_IR_KEEPS_GOING) ? level : !level;
        p->ir_start = addr;
        p->ir_prev = b6;
    }

    packet = (uint64_t) p->addr_step * 1000000000ULL / p->rate;
    elapsed = (uint64_t) (addr - p->ir_start) * 1000000000ULL / p->rate;
    ns = elapsed;

    e.index = addr;
    e.ticks = ticks;
    e.level = p->ir_level;
    e.ended = 0;
    e.partial = p->ir_partial;

    /* the byte holds its value until the next thing happens to it, so only a
       change is news */
    if (b6 != p->ir_prev)
    {
        p->ir_prev = b6;

        if (ticks != MIRISDR_IR_KEEPS_GOING)
        {
            uint64_t tail = (uint64_t) ticks * tick_ns;

            /* the count is the time since the counter last restarted, and the
               stream says how many restarts there were - which it can only do
               while a packet is shorter than half a counter cycle */
            if ((2 * packet < full) && (elapsed > tail))
                ns = ((elapsed - tail + full / 2) / full) * full + tail;
            else if (2 * packet < full)
                ns = tail;
            else if (elapsed < tail)
                ns = tail;

            e.level = level;
            e.ended = 1;
        }
        else if (level != p->ir_level)
        {
            /* the run ended on a count that repeated the byte already there, so
               there is no tail and the stream is the only clock. The next run
               began a counter cycle before this packet, so this one ended then */
            ns = elapsed > full ? elapsed - full : 0;
            e.ended = 1;
        }
    }

    e.duration = (uint32_t) ((ns + 500) / 1000);

    cb(&e, ctx);

    if (e.ended)
    {
        p->ir_partial = 0;
        p->ir_level = (ticks == MIRISDR_IR_KEEPS_GOING) ? level : !level;
        /* a run seen only through the level changing under a 127 began one
           counter cycle before this packet */
        p->ir_start = (ticks == MIRISDR_IR_KEEPS_GOING)
            ? addr - (uint32_t) (full * p->rate / 1000000000ULL)
            : addr;
    }
}

static void mirisdr_ir_latch (mirisdr_dev_t *p, uint8_t b6, uint32_t addr)
{
    if (!p->ir_tick_ns) return;

    mirisdr_ev_lock(p);
    mirisdr_ir_latch_held(p, b6, addr);
    mirisdr_ev_unlock(p);
}
