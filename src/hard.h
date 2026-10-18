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

/* The VCO can go >720MHz, but the PLL can't do this reliably: the highest
 * divider is n=15, and adding a fraction to it will request n=16 for a fraction
 * of the time. This wraps and pulls the VCO in the other direction.
 * Any rate with n=15 and a fraction diverges from the requested rate. */
#define MIRISDR_VCO_MIN                 202000000ULL
#define MIRISDR_VCO_MAX                 720000000ULL

#define MIRISDR_SAMPLE_RATE_MIN         1300000
#define MIRISDR_SAMPLE_RATE_MAX         15000000

/* Verification on a large fleet of devices has shown there is an exact internal
 * bandwidth limit of 56.20MB/s. If this is exceeded the samples and headers
 * are no longer written to the right location, making the stream unusable. */
#define MIRISDR_ENGINE_BLOCK_RATE       56000000ULL

/* rate above which AUTO bypasses the decimator */
#define MIRISDR_DECIMATION_AUTO_RATE    14500000

/* A low IF baseband stream keeps its converters at 4 x IF up to this rate. Above about
 * 7 Msps the receiver's analog performance falls off, gradually */
#define MIRISDR_BB_PREFER_MAX           7000000

/* RSPdx-R2 direct sampling: up to 2 MHz, through its 2 MHz low pass, or the 500 kHz one
   when a baseband stream's band fits under it */
#define MIRISDR_DS_LP500_HZ             500000

/* What the automatic format choice plans on in bulk: 32 MB/s is reached on any host
 * tried. It only steers the choice, see mirisdr_stream_plan() */
#define MIRISDR_BULK_CAPACITY           32000000
