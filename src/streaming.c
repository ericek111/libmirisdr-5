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

int mirisdr_streaming_start (mirisdr_dev_t *p) {
    if (!p) goto failed;
    if (!p->dh) goto failed;

    libusb_control_transfer(p->dh, 0x42, 0x43, 0x0, 0x0, NULL, 0, CTRL_TIMEOUT);

    return 0;

failed:
    return -1;
}

int mirisdr_streaming_stop (mirisdr_dev_t *p) {
    if (!p) goto failed;
    if (!p->dh) goto failed;

    libusb_control_transfer(p->dh, 0x42, 0x45, 0x0, 0x0, NULL, 0, CTRL_TIMEOUT);

    return 0;

failed:
    return -1;
}

/*
 * Dropped sample accounting.
 *
 * Every 1024 byte block carries a 32-bit sample counter in its header; the
 * format converters compare it against the expected value and, on a mismatch,
 * add the gap here. Until now the only report of a gap was a line on stderr,
 * which an application cannot act on and which by itself costs enough at high
 * loss rates to perturb what it is measuring.
 *
 * The counters are free running. Reset them to bound a measurement.
 */
uint32_t mirisdr_get_lost_samples (mirisdr_dev_t *p) {
    return p ? p->lost_samples : 0;
}

uint32_t mirisdr_get_lost_events (mirisdr_dev_t *p) {
    return p ? p->lost_events : 0;
}

/*
 * The subset of events where the block framing was lost rather than a clean gap
 * seen, so no sample count could be attributed. While this is non-zero the
 * lost-sample total is a floor, not a total: whatever went missing during the
 * desynchronised stretch is not in it.
 */
uint32_t mirisdr_get_desync_events (mirisdr_dev_t *p) {
    return p ? p->desync_events : 0;
}

int mirisdr_reset_lost (mirisdr_dev_t *p) {
    if (!p) return -1;

    p->lost_samples = 0;
    p->lost_events = 0;
    p->desync_events = 0;

    return 0;
}

