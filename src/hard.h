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

#define MIRISDR_SAMPLE_RATE_MIN         1300000
#define MIRISDR_SAMPLE_RATE_MAX         15000000

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
