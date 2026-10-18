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

#define CMD_UART_TX             0x54

#define MIRISDR_UART_UNIT_PS    66920
#define MIRISDR_UART_FIXED_PS   5581000
#define MIRISDR_UART_MAX_DELAY  0xFFFF
#define MIRISDR_UART_CHUNK      64          /* the firmware's transfer buffer */

static int mirisdr_uart_delay (uint32_t baud, uint32_t *bit_ns)
{
    uint64_t bit, delay;

    if (!baud) return -1;

    bit = 1000000000000ULL / baud;

    if (bit <= MIRISDR_UART_FIXED_PS) return -1;

    delay = (bit - MIRISDR_UART_FIXED_PS + MIRISDR_UART_UNIT_PS / 2) / MIRISDR_UART_UNIT_PS;

    /* slower than the delay counts: the line would run at another rate */
    if (delay > MIRISDR_UART_MAX_DELAY) return -1;

    if (bit_ns) *bit_ns = (uint32_t) ((delay * MIRISDR_UART_UNIT_PS + MIRISDR_UART_FIXED_PS) / 1000);

    return (int) delay;
}

int mirisdr_uart_write (mirisdr_dev_t *p, uint32_t baud, const uint8_t *buf, int len)
{
    uint32_t bit_ns;
    int done = 0, delay, before = 0;

    if (!p || !p->dh || !buf || (len < 0)) return -1;
    if (!p->fw_ours) return -1;
    if ((delay = mirisdr_uart_delay(baud, &bit_ns)) < 0) return -1;

    while (done < len)
    {
        int n = len - done;
        unsigned int ms;

        if (n > MIRISDR_UART_CHUNK) n = MIRISDR_UART_CHUNK;

        /* The status stage is acknowledged before the bits go out, so it is the
           next transfer that waits for the line: for the chunk before it too. */
        ms = (unsigned int) ((uint64_t) (before + n) * 10 * bit_ns / 1000000) + CTRL_TIMEOUT;
        before = n;

        /* the firmware drives the line through register 8 */
        p->reg8_valid = 0;

        if (libusb_control_transfer(p->dh, 0x40, CMD_UART_TX, (uint16_t) delay, 0,
                                    (unsigned char *) buf + done, (uint16_t) n, ms) != n) return -1;

        done+= n;
    }

    /* the last chunk's bits too, with a request given their time: else the caller's next
       request waits for the line, and times out at a slow rate */
    if (before)
    {
        uint8_t b[4];
        unsigned int ms = (unsigned int) ((uint64_t) before * 10 * bit_ns / 1000000) + CTRL_TIMEOUT;

        if (libusb_control_transfer(p->dh, 0xC0, CMD_RREG, MIRISDR_MEM_XDATA, 0, b, sizeof b, ms) != (int) sizeof b)
            return -1;
    }

    return 0;
}
