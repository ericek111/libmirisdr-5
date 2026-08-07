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
 * The range the synthesiser can actually reach, established by measurement
 * rather than inherited.
 *
 * Low end: 1.010 MS/s, which is below what the part is specified for and is
 * where it was measured to stop.
 *
 * Two things had to give. The output divider runs to 16 -- three-bit field,
 * encoded as Rout / 2 - 1, so 16 is value 7 -- but this lineage and the kernel
 * driver both wrote the search bound as "< 16" and so never emitted it,
 * throwing away the lowest eighth of the range. And the 202 MHz VCO minimum is
 * conservative: sweeping across it, the synthesiser tracks the request down to
 * 1.006 MS/s, a VCO of 193.2 MHz, and only below that does it rail and sit at
 * about 1.017 MS/s whatever it is asked for.
 *
 * So the floor is set from measurement rather than the datasheet, at 1.010
 * MS/s -- a VCO of 193.9 MHz, four kilosamples above the observed edge. That
 * is deliberately out of spec. It is reproducible here, but the margin is
 * small and another part or another temperature may rail a little higher; a
 * caller that gets a rate it did not ask for near the bottom of the range
 * should suspect this first.
 *
 * For reference the spec-derived floor would be 202 MHz / 192 = 1.0521 MS/s.
 *
 * A second bound is often quoted alongside it -- N >= 2, i.e. 571429 S/s -- and
 * it is worth being clear that it is not a datasheet figure. Its only source is
 * a 2013 comment in the original libmirisdr, carried forward unchanged ever
 * since, citing nothing. It never binds in any case: the VCO gives out at an N
 * of about 4, twice the stated limit. And the number encodes the off-by-one
 * above, being 4 MHz * 2 / 14 -- with the divider's real maximum of 16 the same
 * rule would read 500000, which is itself a sign the author believed 14 was the
 * end of the range.
 *
 * High end: 15.5 MS/s, which is where the delivered rate peaks.
 *
 * Three bounds sit close together up there and they are not the same place.
 * Tracking degrades from about 15.4, the delivered rate reaches its maximum of
 * 15.39 MS/s for a request of 15.5 and then turns over -- past that, asking
 * for more returns less, 15.7 giving about 15.24 -- and between 15.740 and
 * 15.760 the part latches, returning roughly 15.69 whatever it is asked, at a
 * VCO near 756 MHz, some 5% past the 720 MHz it is specified for. That is
 * reproducible approached cold, with mild hysteresis: walking up, 15.750
 * survived where a fresh 15.760 did not.
 *
 * Setting the ceiling at the turnover means the highest rate the part can
 * actually produce is reachable, without admitting the range above it where
 * asking for more makes things worse. It leaves about 240 kS/s to the latch.
 * Note this is a looser criterion than the floor, which is placed where
 * requested and delivered still agree: at 15.5 the delivered rate is about
 * 110 kS/s low, and the request is a ceiling rather than a promise.
 *
 * The latch is not terminal: mirisdr_reset_chip() clears it, where reopening
 * and resetting the USB link do not.
 */
/*
 * Ceiling of the sample-rate PLL's own VCO, which is a different synthesiser
 * from the tuner's below. The kernel driver gives its useful range as roughly
 * 202 - 720 MHz; this is the upper figure, used to choose the output divider.
 *
 * The lower figure is deliberately not encoded. It is part dependent -- 193 MHz
 * on one board here, 215 to 218 on another -- and the divider search no longer
 * needs it, since it now takes the largest divider that fits under this ceiling
 * rather than the smallest that clears a floor.
 */
#define MIRISDR_RATE_VCO_MAX            720000000UL

#define MIRISDR_SAMPLE_RATE_MIN         1010000
#define MIRISDR_SAMPLE_RATE_MAX         15500000

/* reg 0x07 bit 9: exchange I and Q. Undocumented; established by measurement. */
#define MIRISDR_SPECTRUM_INVERT         (1u << 9)

/*
 * VCO lock range. Below the floor or above the ceiling the synthesiser will
 * not lock, which is what caps the fundamental tune at about 2115 MHz: the top
 * band divides by 2.
 *
 * Inherited rather than measured here, and thinner evidence than it looks.
 * Both figures are back-computed from Antti Palosaari's signal-generator
 * measurements of the band edges on an MSi3101 and a Hauppauge WinTV 133559 LF,
 * multiplied through each band's divider: VHF gives out near 132 MHz at /32,
 * so 4224, and Band 3 near 263 MHz at /16, so 4208. None of that set is an
 * L-band measurement. His Band 4/5 edge of 960 MHz is a plan boundary rather
 * than a lock failure, and /4 puts it at only 3840.
 *
 * So the 2115 MHz ceiling is an extrapolation: another part's low-band lock
 * limit, divided by a divider nobody tested it with. What the registers allow
 * is a good deal more. INT is six bits, so the highest VCO that can be
 * programmed at all is 96 MHz * 64 = 6144 MHz, and at /2 that would be a
 * 3072 MHz fundamental -- the whole 2.4 GHz ISM band without a harmonic.
 * Whether the silicon locks anywhere near there is untested.
 *
 * Two things would have to move together to find out, since each caps the
 * other: this ceiling, and the band plan's 2400 MHz end marker in soft.c.
 * TODO.md has the experiment; it needs an antenna on a port that hears
 * something, which this setup does not currently have.
 */
#define MIRISDR_VCO_MIN                 1560000000ULL
#define MIRISDR_VCO_MAX                 4230000000ULL

/* highest odd harmonic the automatic search will consider */
#define MIRISDR_HARMONIC_MAX            9
