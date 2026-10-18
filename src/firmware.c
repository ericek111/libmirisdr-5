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

static int mirisdr_open_raw (mirisdr_dev_t **p, uint32_t index, const char *serial, const char *usb_port,
                             int external_tuner, uint8_t gpio_in, int flavour);
static int mirisdr_open_fd_raw (mirisdr_dev_t **p, int fd, int external_tuner, uint8_t gpio_in, int flavour);

const uint8_t *mirisdr_default_firmware (uint32_t *size)
{
    if (size) *size = msi2500pps_fw_len;

    return (const uint8_t *) msi2500pps_fw;
}

void mirisdr_open_config_default (mirisdr_open_config_t *cfg)
{
    if (!cfg) return;

    memset(cfg, 0, sizeof(*cfg));

    cfg->fd = -1;
    cfg->firmware_ids = MIRISDR_FW_IDS_DEVICE;
    cfg->firmware = mirisdr_default_firmware(&cfg->firmware_size);
    cfg->hw_flavour = MIRISDR_HW_AUTO;
}

#define MIRISDR_FW_GONE_MS      2000
#define MIRISDR_FW_WAIT_MS      6000
#define MIRISDR_FW_ACCESS_MS    20000   /* udev here has taken 4-10 s */

static int mirisdr_fw_block (mirisdr_dev_t *p, uint8_t *block)
{
    if (!p) return -1;
    if (mirisdr_read_mem(p, MIRISDR_FW_BLOCK, block, 16, MIRISDR_MEM_XDATA) < 0) return -1;

    if ((block[0] != 'B') || (block[1] != 'V')
        || (block[2] != 'D') || (block[3] != 'B')) return -1;

    return 0;
}

int mirisdr_get_fw_id (mirisdr_dev_t *p, uint8_t *buf, int len)
{
    uint8_t block[16];

    if (!buf || (len <= 0)) return -1;
    if (mirisdr_fw_block(p, block) < 0) return -1;

    if (len > MIRISDR_FW_ID_LEN) len = MIRISDR_FW_ID_LEN;
    memcpy(buf, block + 4, (size_t) len);

    return len;
}

int mirisdr_running_from_rom (mirisdr_dev_t *p)
{
    uint8_t low[4], mirror[4];
    uint16_t at;

    if (!p) return -1;

    for (at = 0; at < 0x1000; at+= sizeof(low))
    {
        if (mirisdr_read_mem(p, at, low, sizeof(low), MIRISDR_MEM_XDATA) < 0) return -1;
        if (mirisdr_read_mem(p, (uint16_t) (0x2000 + at), mirror, sizeof(mirror), MIRISDR_MEM_XDATA) < 0) return -1;

        if (memcmp(low, mirror, sizeof(low))) return 0;
    }

    return 1;
}

/* The USB port the device is in, as mirisdr_port_name() gives it: it comes back there
   after each reboot, and is opened there by name. */ 
typedef struct
{
    char    port[96];
    int     valid;
    int     others;     /* no port, and another receiver about: it cannot be found again */
} mirisdr_fw_path_t;

static void mirisdr_fw_path_of (mirisdr_dev_t *p, mirisdr_fw_path_t *path)
{
    libusb_device *d;

    path->valid = 0;
    path->others = 0;

    if (!p || !p->dh) return;
    if (!(d = libusb_get_device(p->dh))) return;

    mirisdr_port_name(d, path->port);
    path->valid = strchr(path->port, ':') != NULL;
    path->others = !path->valid && (mirisdr_get_device_count() > 1);
}

static int mirisdr_fw_find (const mirisdr_fw_path_t *path, uint32_t fallback, int *denied)
{
    libusb_context *ctx = NULL;
    libusb_device **list, *d = NULL;
    ssize_t i_max;
    int found = -1;

    if (libusb_init(&ctx) < 0) return -1;

    i_max = libusb_get_device_list(ctx, &list);

    if (i_max > 0) d = mirisdr_pick(list, i_max, fallback, NULL, path->valid ? path->port : NULL, &found);
    if (!d) found = -1;

    /* udev can take seconds to give the new node its permissions */
    if ((found >= 0) && denied)
    {
        libusb_device_handle *h;
        int r = libusb_open(d, &h);

        if (r == 0) libusb_close(h);
        *denied = (r == LIBUSB_ERROR_ACCESS);
    }

    if (i_max >= 0) libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return found;
}

static int mirisdr_fw_wait (const mirisdr_fw_path_t *path, uint32_t fallback)
{
    int waited, index = -1, denied = 0;

    for (waited = 0; waited < MIRISDR_FW_GONE_MS; waited+= 100)
    {
        if (mirisdr_fw_find(path, fallback, NULL) < 0) break;
        usleep(100000);
    }

    for (waited = 0; waited < MIRISDR_FW_WAIT_MS; waited+= 100)
    {
        if ((index = mirisdr_fw_find(path, fallback, &denied)) >= 0) break;
        usleep(100000);
    }

    for (waited = 0; (index >= 0) && denied && (waited < MIRISDR_FW_ACCESS_MS); waited+= 100)
    {
        usleep(100000);
        index = mirisdr_fw_find(path, fallback, &denied);
    }

    if ((index >= 0) && !denied) usleep(300000);

    /* still denied: the open reports it */
    return index;
}

/* Locate the device descriptor */
static int mirisdr_fw_descriptor (const uint8_t *image, uint32_t size)
{
    uint32_t at;

    if ((size < MIRISDR_FW_BLOCK + 16)
        || memcmp(image + MIRISDR_FW_BLOCK, "BVDB", 4)) return -1;

    at = image[MIRISDR_FW_BLOCK + 12] | ((uint32_t) image[MIRISDR_FW_BLOCK + 13] << 8);

    if (((uint64_t) at + 18 > size) || (image[at] != 18) || (image[at + 1] != 1)) return -1;

    return (int) at;
}

/* The serial string descriptor, at the other pointer the block publishes. */
static int mirisdr_fw_serial_at (const uint8_t *image, uint32_t size)
{
    uint32_t at;

    if ((size < MIRISDR_FW_BLOCK + 16)
        || memcmp(image + MIRISDR_FW_BLOCK, "BVDB", 4)) return -1;

    at = image[MIRISDR_FW_BLOCK + 14] | ((uint32_t) image[MIRISDR_FW_BLOCK + 15] << 8);

    if ((uint64_t) at + 2 + 2 * MIRISDR_FW_SERIAL_MAX > size) return -1;

    return (int) at;
}

static int mirisdr_fw_get_serial (const uint8_t *image, uint32_t size, char *out, int len)
{
    int at = mirisdr_fw_serial_at(image, size), n, i;

    *out = 0;

    if (at < 0) return -1;

    n = (image[at] > 2) ? (image[at] - 2) / 2 : 0;

    if (n >= len) n = len - 1;

    for (i = 0; i < n; i++) out[i] = (char) image[at + 2 + 2 * i];

    out[n] = 0;

    return n;
}

static int mirisdr_fw_set_serial (uint8_t *image, uint32_t size, const char *serial)
{
    int at = mirisdr_fw_serial_at(image, size), desc, n, i;

    if ((at < 0) || ((desc = mirisdr_fw_descriptor(image, size)) < 0)) return -1;

    n = (int) strlen(serial);

    if (n > MIRISDR_FW_SERIAL_MAX) return -1;

    image[at] = (uint8_t) (n ? 2 + 2 * n : 0);
    image[at + 1] = 3;

    for (i = 0; i < MIRISDR_FW_SERIAL_MAX; i++)
    {
        image[at + 2 + 2 * i] = (i < n) ? (uint8_t) serial[i] : 0;
        image[at + 3 + 2 * i] = 0;
    }

    image[desc + 16] = n ? 3 : 0;

    return 0;
}

static int mirisdr_fw_running (mirisdr_dev_t *p, const uint8_t *image, uint32_t size)
{
    uint8_t block[16];

    if (size < MIRISDR_FW_BLOCK + sizeof(block)) return 0;
    if (mirisdr_read_mem(p, MIRISDR_FW_BLOCK, block, sizeof(block), MIRISDR_MEM_XDATA) < 0) return 0;

    return !memcmp(block, image + MIRISDR_FW_BLOCK, sizeof(block));
}

static int mirisdr_fw_get_ids (const uint8_t *image, uint32_t size, uint16_t *vid, uint16_t *pid)
{
    int at = mirisdr_fw_descriptor(image, size);

    if (at < 0) return -1;

    if (vid) *vid = (uint16_t) (image[at + 8] | (image[at + 9] << 8));
    if (pid) *pid = (uint16_t) (image[at + 10] | (image[at + 11] << 8));

    return 0;
}

static int mirisdr_fw_set_ids (uint8_t *image, uint32_t size, uint16_t vid, uint16_t pid)
{
    int at = mirisdr_fw_descriptor(image, size);

    if (at < 0)
    {
        fprintf(stderr, "the firmware image has no information block, its USB ids cannot be set\n");

        return -1;
    }

    image[at + 8] = (uint8_t) vid;
    image[at + 9] = (uint8_t) (vid >> 8);
    image[at + 10] = (uint8_t) pid;
    image[at + 11] = (uint8_t) (pid >> 8);

    return 0;
}

int mirisdr_fw_get (const uint8_t *image, uint32_t size, mirisdr_fw_patch_t *out)
{
    if (!image || !out) return -1;

    memset(out, 0, sizeof(*out));

    if (!mirisdr_fw_get_ids(image, size, &out->vid, &out->pid))
        out->fields|= MIRISDR_FW_PATCH_IDS;

    if (mirisdr_fw_get_serial(image, size, out->serial, sizeof(out->serial)) >= 0)
        out->fields|= MIRISDR_FW_PATCH_SERIAL;

    return out->fields ? 0 : -1;
}

int mirisdr_fw_patch (uint8_t *image, uint32_t size, const mirisdr_fw_patch_t *p)
{
    uint8_t *work;
    int r = -1;

    if (!image || !p || MIRISDR_RESERVED_SET(*p)) return -1;
    if (p->fields & ~(MIRISDR_FW_PATCH_IDS | MIRISDR_FW_PATCH_SERIAL)) return -1;

    /* on a copy, so a refused part leaves the image as it was */
    if (!(work = malloc(size ? size : 1))) return -1;
    memcpy(work, image, size);

    if ((p->fields & MIRISDR_FW_PATCH_IDS)
        && mirisdr_fw_set_ids(work, size, p->vid, p->pid)) goto out;

    if ((p->fields & MIRISDR_FW_PATCH_SERIAL)
        && mirisdr_fw_set_serial(work, size, p->serial)) goto out;

    memcpy(image, work, size);
    r = 0;

out:
    free(work);

    return r;
}

static uint8_t *mirisdr_fw_read (const char *path, uint32_t *size)
{
    uint8_t *image = NULL;
    long len;
    FILE *f = fopen(path, "rb");

    if (!f) goto failed;
    if (fseek(f, 0, SEEK_END)) goto failed;
    if ((len = ftell(f)) <= 0) goto failed;
    if (fseek(f, 0, SEEK_SET)) goto failed;

    if (len > 0x1800) goto failed;

    if (!(image = malloc((size_t) len))) goto failed;
    if (fread(image, 1, (size_t) len, f) != (size_t) len) goto failed;

    fclose(f);
    *size = (uint32_t) len;

    return image;

failed:
    if (image) free(image);
    if (f) fclose(f);
    fprintf(stderr, "cannot read firmware %s\n", path);

    return NULL;
}

static int mirisdr_fw_ids_of (mirisdr_dev_t *p, uint16_t *vid, uint16_t *pid)
{
    struct libusb_device_descriptor dd;
    libusb_device *d;

    if (!(d = libusb_get_device(p->dh))) return -1;
    if (libusb_get_device_descriptor(d, &dd) < 0) return -1;

    *vid = dd.idVendor;
    *pid = dd.idProduct;

    return 0;
}

static int mirisdr_fw_ids_wanted (mirisdr_dev_t *p, const mirisdr_open_config_t *cfg,
                                  const uint8_t *image, uint32_t size,
                                  uint16_t *vid, uint16_t *pid)
{
    if ((cfg->firmware_ids == MIRISDR_FW_IDS_SET)
        && (cfg->firmware_patch.fields & MIRISDR_FW_PATCH_IDS))
    {
        *vid = cfg->firmware_patch.vid;
        *pid = cfg->firmware_patch.pid;

        return 1;
    }

    if (cfg->firmware_ids == MIRISDR_FW_IDS_DEVICE)
        return (mirisdr_fw_ids_of(p, vid, pid) < 0) ? -1 : 1;

    return mirisdr_fw_get_ids(image, size, vid, pid) ? 0 : 1;
}

#define MIRISDR_EE_IDS          0xB4    /* byte 0: the ROM takes ids from bytes 1-6 */
#define MIRISDR_EE_SERIAL_AT    7       /* SDRplay keep the serial right behind them */

/* An RSP1B, RSPdx-R2 or RSPduo on its own ids running the ROM declares no serial, but its EEPROM
   holds one behind the ids. Not when byte 0 says microcode follows instead. */
static int mirisdr_fw_ee_serial (mirisdr_dev_t *p, char *out, int len)
{
    uint8_t b[MIRISDR_FW_SERIAL_MAX];
    mirisdr_device_t *d;
    uint16_t vid, pid;
    int n;

    if (mirisdr_fw_ids_of(p, &vid, &pid) < 0) return 0;
    if (!(d = mirisdr_device_get(vid, pid)) || !MIRISDR_HW_EXPANDER(d->flavour)) return 0;

    if ((mirisdr_read_eeprom(p, 0, b, 1) < 0) || (b[0] != MIRISDR_EE_IDS)) return 0;
    if (mirisdr_read_eeprom(p, MIRISDR_EE_SERIAL_AT, b, sizeof b) < 0) return 0;

    /* printable up to the first byte that is not, 0xFF on the RSP1B and RSPdx-R2 */
    for (n = 0; (n < (int) sizeof b) && (b[n] >= 0x20) && (b[n] < 0x7F); n++);
    if (!n || (n >= len)) return 0;

    memcpy(out, b, (size_t) n);
    out[n] = 0;

    return 1;
}

/* The same three ways the ids are chosen, since a serial is part of the identity
   the image carries. Read before any reboot: the ROM declares none, so an SDRplay's
   comes from its EEPROM. */
static int mirisdr_fw_serial_wanted (mirisdr_dev_t *p, const mirisdr_open_config_t *cfg,
                                     const uint8_t *image, uint32_t size, char *out, int len)
{
    if (mirisdr_fw_serial_at(image, size) < 0) return 0;

    if ((cfg->firmware_ids == MIRISDR_FW_IDS_SET)
        && (cfg->firmware_patch.fields & MIRISDR_FW_PATCH_SERIAL))
    {
        if ((int) strlen(cfg->firmware_patch.serial) >= len) return -1;

        strcpy(out, cfg->firmware_patch.serial);

        return 1;
    }

    if (cfg->firmware_ids == MIRISDR_FW_IDS_DEVICE)
    {
        char full[256];

        /* one longer than the image holds would come back cut, under another name */
        if (mirisdr_get_serial(p, full, sizeof full) > 0)
        {
            if ((int) strlen(full) < len)
            {
                strcpy(out, full);
                return 1;
            }
            fprintf(stderr, "libmirisdr: serial %s is longer than the firmware holds (%d), "
                            "keeping the image's\n", full, len - 1);
        }
        else if (mirisdr_fw_ee_serial(p, out, len)) return 1;
    }

    return (mirisdr_fw_get_serial(image, size, out, len) < 0) ? 0 : 1;
}

static int mirisdr_fw_serial_ok (mirisdr_dev_t *p, int have, const char *want)
{
    char now[MIRISDR_FW_SERIAL_MAX + 1];

    if (!have) return 1;
    if (mirisdr_get_serial(p, now, sizeof(now)) < 0) return 0;

    return !strcmp(now, want);
}

static int mirisdr_fw_ids_ok (mirisdr_dev_t *p, int have, uint16_t vid, uint16_t pid)
{
    uint16_t now_vid, now_pid;

    if (!have) return 1;
    if (mirisdr_fw_ids_of(p, &now_vid, &now_pid) < 0) return 0;

    return (now_vid == vid) && (now_pid == pid);
}

/* by index, or by serial and port, see mirisdr_pick() */
static int mirisdr_fw_open (mirisdr_dev_t **dev, const mirisdr_open_config_t *cfg, uint32_t at,
                            const char *serial, const char *usb_port)
{
    uint8_t block[16];
    int r = (cfg->fd >= 0) ? mirisdr_open_fd_raw(dev, cfg->fd, cfg->external_tuner, cfg->gpio_input_mask, cfg->hw_flavour)
                           : mirisdr_open_raw(dev, at, serial, usb_port, cfg->external_tuner,
                                              cfg->gpio_input_mask, cfg->hw_flavour);

    /* the ROM answers the memory requests too, so anything that needs our own
       firmware asks this rather than assuming */
    if (r == 0) (*dev)->fw_ours = (mirisdr_fw_block(*dev, block) == 0);

    /* where its status blocks are, after the block's first 16 bytes; older
       images have zeros there */
    if ((r == 0) && (*dev)->fw_ours) {
        uint8_t t[4];

        if (mirisdr_read_mem(*dev, MIRISDR_FW_BLOCK + 16, t, sizeof t, MIRISDR_MEM_XDATA) == 0) {
            (*dev)->fw_pps_at = t[0];
            (*dev)->fw_anchor_at = (uint16_t) (t[1] | t[2] << 8);
            (*dev)->fw_list_at = t[3];
        }
    }

    return r;
}

/* Opened by fd, each reboot ends the call, so how far the EEPROM steps got is kept for the
   next open of the same device (its serial), for a little while: else a board that boots
   firmware from its EEPROM would be rebooted to the ROM, and into the EEPROM, for ever.
   1: the plain reboot was done, 2: the one past the EEPROM too */
#define MIRISDR_FD_NOTE_S       30

static struct {
    char serial[MIRISDR_FW_SERIAL_MAX + 1];
    time_t at;
    int stage;
} mirisdr_fd_note;

static int mirisdr_fd_stage (mirisdr_dev_t *dev)
{
    char ser[MIRISDR_FW_SERIAL_MAX + 1] = "";

    if (!mirisdr_fd_note.stage || (time(NULL) - mirisdr_fd_note.at > MIRISDR_FD_NOTE_S)) return 0;
    mirisdr_get_serial(dev, ser, sizeof ser);
    return strcmp(ser, mirisdr_fd_note.serial) ? 0 : mirisdr_fd_note.stage;
}

static void mirisdr_fd_mark (mirisdr_dev_t *dev, int stage)
{
    mirisdr_fd_note.serial[0] = '\0';
    if (stage) mirisdr_get_serial(dev, mirisdr_fd_note.serial, sizeof mirisdr_fd_note.serial);
    mirisdr_fd_note.at = time(NULL);
    mirisdr_fd_note.stage = stage;
}

static int mirisdr_fw_recycle (mirisdr_dev_t **dev, const mirisdr_open_config_t *cfg,
                               mirisdr_fw_path_t *path, uint32_t *at, uint32_t index)
{
    int again;

    mirisdr_close(*dev);
    *dev = NULL;

    if (cfg->fd >= 0) return MIRISDR_REOPEN;

    /* found again by its index alone: with another receiver about that could be the other
       one, which would then get this one's firmware */
    if (path->others)
    {
        fprintf(stderr, "libmirisdr: the receiver's USB port is unknown and there is another: "
                        "not finding it again by index, open it again\n");
        return -1;
    }

    if ((again = mirisdr_fw_wait(path, index)) < 0) return -1;
    *at = (uint32_t) again;

    return mirisdr_fw_open(dev, cfg, *at, NULL, path->valid ? path->port : NULL);
}

int mirisdr_open_ex (mirisdr_dev_t **out, const mirisdr_open_config_t *cfg)
{
    const uint8_t *image;
    uint8_t *owned = NULL, *work = NULL;
    mirisdr_dev_t *dev = NULL;
    mirisdr_fw_path_t path;
    uint32_t index, size, at;
    uint16_t vid = 0, pid = 0;
    char serial[MIRISDR_FW_SERIAL_MAX + 1] = "";
    int have_ids = 0, have_serial = 0;
    int from_rom, fd_stage;
    int r = -1;

    if (!out || !cfg) return -1;

    *out = NULL;

    if ((cfg->hw_flavour < MIRISDR_HW_AUTO) || (cfg->hw_flavour > MIRISDR_HW_RSPDUO))
    {
        fprintf(stderr, "libmirisdr: hw_flavour %d is not a board\n", cfg->hw_flavour);
        return -1;
    }

    if (MIRISDR_RESERVED_SET(*cfg) || MIRISDR_RESERVED_SET(cfg->firmware_patch))
    {
        fprintf(stderr, "libmirisdr: " MIRISDR_RESERVED_MSG, "the open config");
        return -1;
    }

    index = at = cfg->index;
    path.valid = 0;
    path.others = 0;

    image = cfg->firmware;
    size = cfg->firmware_size;

    /* A path wins over a buffer: the default config carries the built-in image,
       so a caller that names a file would otherwise be silently ignored. */
    if (cfg->firmware_path && !cfg->keep_running)
    {
        if (!(owned = mirisdr_fw_read(cfg->firmware_path, &size))) return -1;
        image = owned;
    }

    /* RAM from 0x1800 on is the firmware's own, see mirisdr_fw_read() */
    if (image && (size > 0x1800) && !cfg->keep_running)
    {
        fprintf(stderr, "the firmware image is %u bytes, more than the 0x1800 there is room for\n", size);
        goto out;
    }

    if (image && size)
    {
        if (!(work = malloc(size))) goto out;
        memcpy(work, image, size);
    }

    if (mirisdr_fw_open(&dev, cfg, at, (cfg->fd < 0) ? cfg->serial : NULL,
                        (cfg->fd < 0) ? cfg->usb_port : NULL) < 0) goto out;

    if (cfg->fd < 0)
    {
        index = at = dev->index;
        mirisdr_fw_path_of(dev, &path);
    }

    if (work && !cfg->keep_running)
    {
        if ((have_ids = mirisdr_fw_ids_wanted(dev, cfg, work, size, &vid, &pid)) < 0) goto out;

        /* it comes back on those ids, which have to be found to reopen it */
        if (have_ids && !mirisdr_device_get(vid, pid))
        {
            fprintf(stderr, "not loading: %04x:%04x is not a receiver this library finds again\n", vid, pid);
            goto out;
        }
        if (have_ids && (mirisdr_fw_set_ids(work, size, vid, pid) < 0)) goto out;

        if ((have_serial = mirisdr_fw_serial_wanted(dev, cfg, work, size, serial,
                                                    sizeof(serial))) < 0) goto out;
        if (have_serial && (mirisdr_fw_set_serial(work, size, serial) < 0)) goto out;
    }

    if (cfg->keep_running || !image || !size
        || (mirisdr_fw_running(dev, work, size) && mirisdr_fw_ids_ok(dev, have_ids, vid, pid)
            && mirisdr_fw_serial_ok(dev, have_serial, serial)))
    {
        *out = dev;
        dev = NULL;
        r = 0;

        goto out;
    }

    if ((from_rom = mirisdr_running_from_rom(dev)) < 0)
    {
        fprintf(stderr, "not loading: cannot tell whether the device runs from ROM or RAM\n");

        r = -1;

        goto out;
    }

    /* by fd: the steps an earlier open of this device took, see mirisdr_fd_note */
    fd_stage = (cfg->fd >= 0) ? mirisdr_fd_stage(dev) : 0;
    if ((from_rom == 1) && (cfg->fd >= 0)) mirisdr_fd_mark(dev, 0);

    if ((from_rom == 0) && (fd_stage == 2))
    {
        fprintf(stderr, "not loading: the device boots firmware from its SPI flash\n");
        mirisdr_fd_mark(dev, 0);

        *out = dev;
        dev = NULL;
        r = 0;

        goto out;
    }

    if ((from_rom == 0) && (fd_stage == 1))
    {
        /* the plain reboot came back to firmware: past the EEPROM this time */
        mirisdr_fd_mark(dev, 2);
        if (mirisdr_reboot(dev, MIRISDR_BOOT_IGNORE_EEPROM) != MIRISDR_REOPEN) goto out;
        r = mirisdr_fw_recycle(&dev, cfg, &path, &at, index);
        goto out;
    }

    if (from_rom == 0)
    {
        /* Try a plain reboot first */
        if (cfg->fd >= 0) mirisdr_fd_mark(dev, 1);
        if (mirisdr_reboot(dev, MIRISDR_BOOT_ROM) != MIRISDR_REOPEN) goto out;

        if ((r = mirisdr_fw_recycle(&dev, cfg, &path, &at, index))) goto out;

        if ((from_rom = mirisdr_running_from_rom(dev)) < 0)
        {
            fprintf(stderr, "not loading: cannot tell what the device booted into\n");

            r = -1;

            goto out;
        }

        /* Firmware from flash: there is an EEPROM, so try to avoid it */
        if (from_rom == 0)
        {
            if (mirisdr_reboot(dev, MIRISDR_BOOT_IGNORE_EEPROM) != MIRISDR_REOPEN) goto out;

            if ((r = mirisdr_fw_recycle(&dev, cfg, &path, &at, index))) goto out;

            if ((from_rom = mirisdr_running_from_rom(dev)) < 0)
            {
                fprintf(stderr, "not loading: cannot tell what the device booted into\n");

                r = -1;

                goto out;
            }
        }

        /* Booting to the ROM landed on firmware even past the EEPROM (the strap held
           wrong, or nothing to hold: no boot image there), so the handover is not the
           EEPROM's doing. RAM cannot be written under it, but what booted is usable,
           so hand that back. */
        if (from_rom != 1)
        {
            fprintf(stderr, "not loading: the device boots firmware from its SPI flash\n");

            *out = dev;
            dev = NULL;
            r = 0;

            goto out;
        }
    }

    if (mirisdr_write_mem(dev, 0x0000, work, (int) size, 0) < 0)
    {
        r = -1;

        goto out;
    }

    if (mirisdr_reboot(dev, MIRISDR_BOOT_RAM) != MIRISDR_REOPEN) goto out;

    if ((r = mirisdr_fw_recycle(&dev, cfg, &path, &at, index))) goto out;

    if (mirisdr_fw_running(dev, work, size) && mirisdr_fw_ids_ok(dev, have_ids, vid, pid)
            && mirisdr_fw_serial_ok(dev, have_serial, serial))
    {
        *out = dev;
        dev = NULL;
        r = 0;

        goto out;
    }

    fprintf(stderr, "the firmware did not come up\n");
    r = -1;

out:
    /* a device not handed back is a failure, whatever an earlier step left in r */
    if (dev)
    {
        mirisdr_close(dev);
        if (r == 0) r = -1;
    }
    if (work) free(work);
    if (owned) free(owned);

    return r;
}
