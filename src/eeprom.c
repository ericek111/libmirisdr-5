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

#define MIRISDR_EE_READ         0x03
#define MIRISDR_EE_WRITE        0x02
#define MIRISDR_EE_WREN         0x06
#define MIRISDR_EE_WRDI         0x04
#define MIRISDR_EE_RDSR         0x05

#define MIRISDR_EE_WEL          0x02
#define MIRISDR_EE_SMALL        0x200
#define MIRISDR_EE_LARGE        0x10000

#define MIRISDR_EE_BOOT_IMAGE   0xD2

#define MIRISDR_EE_WIP          0x01
#define MIRISDR_EE_WRITE_MS     500

static int mirisdr_ee_get (mirisdr_dev_t *p, uint16_t addr, uint8_t *v)
{
    if (p->ee_size == MIRISDR_EE_SMALL)
        return mirisdr_spi(p, 3, (uint8_t) (MIRISDR_EE_READ | ((addr >> 5) & 0x08)),
                           (uint8_t) addr, 0x00, 0, v);

    return mirisdr_spi(p, 4, MIRISDR_EE_READ, (uint8_t) (addr >> 8), (uint8_t) addr, 0x00, v);
}

static int mirisdr_ee_put (mirisdr_dev_t *p, uint16_t addr, uint8_t v)
{
    int i;

    if (mirisdr_spi(p, 1, MIRISDR_EE_WREN, 0, 0, 0, NULL) < 0) return -1;

    if (p->ee_size == MIRISDR_EE_SMALL)
    {
        if (mirisdr_spi(p, 3, (uint8_t) (MIRISDR_EE_WRITE | ((addr >> 5) & 0x08)),
                        (uint8_t) addr, v, 0, NULL) < 0) return -1;
    }
    else if (mirisdr_spi(p, 4, MIRISDR_EE_WRITE, (uint8_t) (addr >> 8),
                         (uint8_t) addr, v, NULL) < 0) return -1;

    for (i = 0; i < MIRISDR_EE_WRITE_MS; i++)
    {
        uint8_t sr = 0;

        if (mirisdr_spi(p, 2, MIRISDR_EE_RDSR, 0x00, 0, 0, &sr) < 0) return -1;
        if (!(sr & MIRISDR_EE_WIP)) return 0;

        usleep(1000);
    }

    return -1;
}

/* strap: the GPIO inputs read before GPIO_2 was driven low, GPIO_2 is the strap */
static int mirisdr_ee_probe (mirisdr_dev_t *p, int strap)
{
    uint8_t wren = 0, wrdi = 0;

    /* a failure leaves it unknown, to probe again; only an answer is kept */
    if (mirisdr_spi(p, 1, MIRISDR_EE_WREN, 0, 0, 0, NULL) < 0) return -1;
    if (mirisdr_spi(p, 2, MIRISDR_EE_RDSR, 0x00, 0, 0, &wren) < 0) return -1;
    if (mirisdr_spi(p, 1, MIRISDR_EE_WRDI, 0, 0, 0, NULL) < 0) return -1;
    if (mirisdr_spi(p, 2, MIRISDR_EE_RDSR, 0x00, 0, 0, &wrdi) < 0) return -1;

    if (!(wren & MIRISDR_EE_WEL) || (wrdi & MIRISDR_EE_WEL)) return p->ee_size = 0;

    p->ee_size = (strap & 0x04) ? MIRISDR_EE_LARGE : MIRISDR_EE_SMALL;

    return p->ee_size;
}

static void mirisdr_ee_end (mirisdr_dev_t *p);

/* -1: refused, nothing touched (or a failure, already ended), else the size, and the
   caller ends it */
static int mirisdr_ee_begin (mirisdr_dev_t *p)
{
    uint32_t w0, w1;
    int strap = 0;

    if (!p || !p->dh) return -1;

    w0 = MIRISDR_HW_EXPANDER(p->hw_flavour) ? 0x00AA80 : 0x002280;
    w1 = MIRISDR_HW_EXPANDER(p->hw_flavour) ? 0x00EA80 : 0x006280;

    /* the tuner is programmed over this same port */
    if (mirisdr_async_get(p) != MIRISDR_ASYNC_INACTIVE) return -1;

    /* the port drives pins the application holds as inputs */
    if (p->gpio_hold & (((w0 | w1) >> 12) & 0xF)) return -1;

    /* GPIO for EEPROM access as the ROM has it: the strap (the GPIO_2 pad) read with
       the pin an input, then GPIO_2 low, as the RSPduo's EEPROM and expander only
       drive MISO then */
    if (mirisdr_write_reg(p, 0x08, w0) < 0) goto failed;
    if ((p->ee_size < 0) && ((strap = mirisdr_get_gpio_inputs(p)) < 0)) goto failed;
    if (mirisdr_write_reg(p, 0x08, w1) < 0) goto failed;

    /* register 8 is the EEPROM's now: give it back if this fails */
    if ((p->ee_size < 0) && (mirisdr_ee_probe(p, strap) < 0)) goto failed;

    return p->ee_size;

failed:
    mirisdr_ee_end(p);
    return -1;
}

static void mirisdr_ee_end (mirisdr_dev_t *p)
{
    mirisdr_write_reg(p, 0x0B, 0);

    update_reg_8(p);
}

/* Attempt to drive the strap pin incorrectly to EEPROM configuration is skipped */
static int mirisdr_ee_ignore (mirisdr_dev_t *p)
{
    uint8_t first;
    int r;

    if ((r = mirisdr_ee_begin(p)) < 0) return -1;
    if (!r) goto out;
    if ((r = mirisdr_ee_get(p, 0, &first)) < 0) goto out;

    r = (first == MIRISDR_EE_BOOT_IMAGE) ? 1 : 0;

out:
    mirisdr_ee_end(p);

    if (r <= 0) return r;

    if (mirisdr_set_gpio_direction(p, 2, 1) < 0) return -1;

    return (mirisdr_set_gpio_output(p, 2, p->ee_size == MIRISDR_EE_SMALL) < 0) ? -1 : 1;
}

int mirisdr_read_eeprom (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len)
{
    int i, size, r = -1;

    if (!p || !buf || (len < 0)) return -1;
    if ((size = mirisdr_ee_begin(p)) < 0) return -1;
    if (!size || ((int) addr + len > size)) goto out;

    for (i = 0; i < len; i++)
        if (mirisdr_ee_get(p, (uint16_t) (addr + i), buf + i) < 0) goto out;

    r = 0;

out:
    mirisdr_ee_end(p);

    return r;
}

int mirisdr_write_eeprom (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len)
{
    int i, size, r = -1;

    if (!p || !buf || (len < 0)) return -1;
    if ((size = mirisdr_ee_begin(p)) < 0) return -1;
    if (!size || ((int) addr + len > size)) goto out;

    for (i = 0; i < len; i++)
    {
        uint16_t at = (uint16_t) (addr + i);
        uint8_t now;

        /* dont write identical bytes */
        if (mirisdr_ee_get(p, at, &now) < 0) goto out;
        if (now == buf[i]) continue;

        if (mirisdr_ee_put(p, at, buf[i]) < 0) goto out;
    }

    r = 0;

out:
    mirisdr_ee_end(p);

    return r;
}

int mirisdr_eeprom_size (mirisdr_dev_t *p)
{
    int r;

    if (!p) return -1;
    if (mirisdr_ee_begin(p) < 0) return -1;

    r = p->ee_size;

    mirisdr_ee_end(p);

    return r;
}
