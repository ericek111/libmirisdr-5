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
 * Bit banged master on GPIO_1 and GPIO_2, open drain through the direction
 * bits.  The firmware owns what happens inside a bit - START and STOP shapes,
 * the ACK sample, clock stretching - and everything once per transfer is
 * decided here.  Clock period measured with fw/tools/uartcal.c.
 */

#define CMD_I2C_WRITE           0x55
#define CMD_I2C_READ            0x56
#define CMD_I2C_STATUS          0x57
#define CMD_I2C_RECOVER         0x58

#define MIRISDR_I2C_UNIT_PS     200830
#define MIRISDR_I2C_FIXED_PS    16420000
#define MIRISDR_I2C_MAX_DELAY   0xFFFF
#define MIRISDR_I2C_CHUNK       64      /* the firmware's transfer buffer */

int mirisdr_set_i2c_rate (mirisdr_dev_t *p, uint32_t hz)
{
    uint64_t period, delay = 0;

    if (!p || !hz) return -1;

    period = 1000000000000ULL / hz;

    if (period > MIRISDR_I2C_FIXED_PS)
        delay = (period - MIRISDR_I2C_FIXED_PS + MIRISDR_I2C_UNIT_PS / 2) / MIRISDR_I2C_UNIT_PS;

    if (delay > MIRISDR_I2C_MAX_DELAY) delay = MIRISDR_I2C_MAX_DELAY;

    p->i2c_delay = (uint16_t) delay;
    p->i2c_clock_ns = (uint32_t) ((delay * MIRISDR_I2C_UNIT_PS + MIRISDR_I2C_FIXED_PS) / 1000);

    return 0;
}

/* ms: the bits of a write still going out, which this request waits for */
static int mirisdr_i2c_status (mirisdr_dev_t *p, unsigned int ms)
{
    uint8_t st;

    if (libusb_control_transfer(p->dh, 0xC0, CMD_I2C_STATUS, 0, 0, &st, 1, ms) != 1) return -1;

    return st;
}

static int mirisdr_i2c_xfer (mirisdr_dev_t *p, uint8_t addr, unsigned int flags,
                             uint8_t *buf, int len, int in)
{
    unsigned int ms;

    if (!p || !p->dh || !p->fw_ours) return -1;
    if ((addr > 0x7F) || (len < 0) || (len > MIRISDR_I2C_CHUNK) || (!buf && len)) return -1;

    ms = (unsigned int) ((uint64_t) (len + 1) * 9 * p->i2c_clock_ns / 1000000) + CTRL_TIMEOUT;

    /* the firmware drives the lines through register 8 */
    p->reg8_valid = 0;

    if (libusb_control_transfer(p->dh, in ? 0xC0 : 0x40, in ? CMD_I2C_READ : CMD_I2C_WRITE,
                                (uint16_t) (addr | (flags << 8)), p->i2c_delay,
                                buf, (uint16_t) len, ms) != len) return -1;

    /* a write's bits go out after its status stage, so the status waits for them */
    return mirisdr_i2c_status(p, in ? CTRL_TIMEOUT : ms);
}

int mirisdr_i2c_write (mirisdr_dev_t *p, uint8_t addr, unsigned int flags,
                       const uint8_t *buf, int len)
{
    return mirisdr_i2c_xfer(p, addr, flags, (uint8_t *) buf, len, 0);
}

int mirisdr_i2c_read (mirisdr_dev_t *p, uint8_t addr, unsigned int flags,
                      uint8_t *buf, int len)
{
    return mirisdr_i2c_xfer(p, addr, flags, buf, len, 1);
}

int mirisdr_i2c_recover (mirisdr_dev_t *p)
{
    if (!p || !p->dh || !p->fw_ours) return -1;

    p->reg8_valid = 0;

    if (libusb_control_transfer(p->dh, 0x40, CMD_I2C_RECOVER, 0, p->i2c_delay,
                                NULL, 0, CTRL_TIMEOUT) < 0) return -1;

    /* nine clocks and a stop */
    return mirisdr_i2c_status(p, (unsigned int) ((uint64_t) 11 * p->i2c_clock_ns / 1000000) + CTRL_TIMEOUT);
}

/* Standard write-repeated-start-read i2c transaction */
int mirisdr_i2c_transfer (mirisdr_dev_t *p, uint8_t addr,
                          const uint8_t *out, int outlen, uint8_t *in, int inlen)
{
    if (outlen || !inlen)
    {
        int st = mirisdr_i2c_write(p, addr, inlen ? MIRISDR_I2C_NO_STOP : 0, out, outlen);

        if (st || !inlen)
        {
            /* only a write held open for the read can be left stuck, also when the request
               itself failed */
            if (st && inlen) mirisdr_i2c_recover(p);

            return st;
        }
    }

    return mirisdr_i2c_read(p, addr, outlen ? MIRISDR_I2C_REPEAT : 0, in, inlen);
}
