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

int mirisdr_adc_init (mirisdr_dev_t *p) {
    if (!p) goto failed;

    /* initialisation, fixed values */
    p->reg8 = 0x006080;                   /* kernel driver */
    update_reg_8(p);
    mirisdr_write_reg(p, 0x05, 0x00000c);
    mirisdr_write_reg(p, 0x00, 0x000200);
    mirisdr_write_reg(p, 0x02, 0x004801);
    p->reg8 = 0x00f380;                   /* kernel driver */
    update_reg_8(p);

    return 0;

failed:
    return -1;
}

static int mirisdr_adc_stop (mirisdr_dev_t *p) {
    if (!p) goto failed;

    /* put the USB interface and the ADC to sleep */
    if (mirisdr_write_reg(p, 0x03, 0x010000) < 0) goto failed;

    return 0;

failed:
    return -1;
}
