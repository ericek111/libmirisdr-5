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

static int mirisdr_ee_ignore (mirisdr_dev_t *p);

#define MIRISDR_MEM_READ_UNIT   4
#define MIRISDR_MEM_READ_BLOCK  64
#define MIRISDR_MEM_WRITE_CHUNK 64

int mirisdr_read_mem (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len, int source)
{
    uint8_t four[MIRISDR_MEM_READ_UNIT];
    int done = 0;

    if (!p) goto failed;
    if (!p->dh || !buf || (len < 0)) goto failed;
    if (source < MIRISDR_MEM_XDATA || source > MIRISDR_MEM_IRAM) goto failed;
    if (source == MIRISDR_MEM_IRAM && (!p->fw_ours || addr + len > 0x100)) goto failed;

    while (done < len)
    {
        int n = len - done;
        uint16_t at = (uint16_t) (addr + done);

        if (p->fw_ours) {
            if (n > MIRISDR_MEM_READ_BLOCK) n = MIRISDR_MEM_READ_BLOCK;
            if (mirisdr_rreg(p, source, at, buf + done, n) != n) goto failed;
            done+= n;
            continue;
        }

        if (mirisdr_rreg(p, source, at, four, sizeof(four)) != (int) sizeof(four)) goto failed;

        if (n > MIRISDR_MEM_READ_UNIT) n = MIRISDR_MEM_READ_UNIT;
        memcpy(buf + done, four, n);
        done+= n;
    }

    return 0;

failed:
    return -1;
}

int mirisdr_write_mem (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len, int remap)
{
    int done = 0;

    if (!p) goto failed;
    if (!p->dh || !buf || (len < 0)) goto failed;

    while (done < len)
    {
        int n = len - done;

        if (n > MIRISDR_MEM_WRITE_CHUNK) n = MIRISDR_MEM_WRITE_CHUNK;

        if (libusb_control_transfer(p->dh, 0x40, CMD_DOWNLOAD,
                                    (uint16_t) (addr + done), remap ? 1 : 0,
                                    (unsigned char *) buf + done, n,
                                    CTRL_TIMEOUT) != n) goto failed;

        done+= n;
    }

    return 0;

failed:
    return -1;
}

#define CMD_CALL                0x59

int mirisdr_call (mirisdr_dev_t *p, uint16_t addr, mirisdr_call_regs_t *regs)
{
    uint8_t ctx[6], reply[6];

    if (!p) goto failed;
    if (!p->dh || !regs || !p->fw_ours) goto failed;

    ctx[0] = regs->a;
    ctx[1] = regs->b;
    ctx[2] = (uint8_t) regs->dptr;
    ctx[3] = (uint8_t) (regs->dptr >> 8);
    ctx[4] = regs->r0;
    ctx[5] = regs->r1;

    if (mirisdr_write_mem(p, MIRISDR_CALL_CTX, ctx, sizeof(ctx), 0) < 0) goto failed;

    if (libusb_control_transfer(p->dh, 0xC0, CMD_CALL, addr, 0, reply,
                                sizeof(reply), CTRL_TIMEOUT) != (int) sizeof(reply))
        goto failed;

    regs->a = reply[0];
    regs->b = reply[1];
    regs->dptr = (uint16_t) (reply[2] | (reply[3] << 8));
    regs->r0 = reply[4];
    regs->r1 = reply[5];

    return 0;

failed:
    return -1;
}

/*
 * Reboot the device. Returns MIRISDR_REOPEN on success, since the device leaves
 * the bus and the handle has to be closed and reopened, or -1 if the request failed.
 */
int mirisdr_reboot (mirisdr_dev_t *p, int mode)
{
    if (!p) goto failed;
    if (!p->dh) goto failed;

    if (mode == MIRISDR_BOOT_IGNORE_EEPROM)
    {
        int held = mirisdr_ee_ignore(p);

        if (held < 0) goto failed;
        if (!held && (mirisdr_set_gpio_direction(p, 2, 0) < 0)) goto failed;
    }
    else if ((mode != MIRISDR_BOOT_RAM) && (mirisdr_set_gpio_direction(p, 2, 0) < 0)) goto failed;

    if (libusb_control_transfer(p->dh, 0x40, CMD_RESET, (mode == MIRISDR_BOOT_RAM) ? 1 : 0, 0,
                                NULL, 0, CTRL_TIMEOUT) < 0) goto failed;

    return MIRISDR_REOPEN;

failed:
    return -1;
}
