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

static mirisdr_device_t mirisdr_devices[] = {
    { 0x1df7, 0x2500, "Mirics MSi2500 default (e.g. VTX3D card, SDRplay RSP1)", "Mirics", "MSi2500", MIRISDR_HW_DEFAULT},
    { 0x1df7, 0x3000, "SDRplay RSP1A", "SDRPlay", "RSP1A", MIRISDR_HW_DEFAULT},
    { 0x1df7, 0x3010, "SDRplay RSP2/RSP2pro", "SDRPlay", "RSP2", MIRISDR_HW_DEFAULT},
    { 0x1df7, 0x3050, "SDRplay RSP1B", "SDRPlay", "RSP1B", MIRISDR_HW_RSP1B},
    { 0x1df7, 0x3060, "SDRplay RSPdx-R2", "SDRPlay", "RSPdxR2", MIRISDR_HW_RSPDXR2},
    { 0x1df7, 0x3020, "SDRplay RSPduo", "SDRPlay", "RSPduo", MIRISDR_HW_RSPDUO},
    { 0x16d0, 0x158c, "BertoldVdb MSi2500 Acquisition Board", "BertoldVdb", "MSI2500-PPS", MIRISDR_HW_DEFAULT},
    { 0x2040, 0xd300, "Hauppauge WinTV 133559 LF", "Hauppauge", "WinTV 133559 LF", MIRISDR_HW_DEFAULT},
    { 0x07ca, 0x8591, "AverMedia A859 Pure DVBT", "AverTV", "A859 Pure DVBT", MIRISDR_HW_DEFAULT},
    { 0x04bb, 0x0537, "IO-DATA GV-TV100 stick", "IO-DATA", "GV-TV100", MIRISDR_HW_DEFAULT},
    { 0x0511, 0x0037, "Logitec LDT-1S310U/J", "Logitec", "LDT-1S310U/J", MIRISDR_HW_DEFAULT}
};

static mirisdr_device_t *mirisdr_device_get (uint16_t vid, uint16_t pid) {
    size_t i;

    for (i = 0; i < sizeof(mirisdr_devices) / sizeof(mirisdr_device_t); i++) {
        if ((mirisdr_devices[i].vid == vid) && (mirisdr_devices[i].pid == pid)) return &mirisdr_devices[i];
    }

    return NULL;
}

#define MIRISDR_OWN_VID         0x16d0
#define MIRISDR_OWN_PID         0x158c
#define MIRISDR_OWN_RSP1B       "B-"    /* our ids hide the board: an RSP1B serial starts so, */
#define MIRISDR_OWN_RSPDXR2     "C-"    /* an RSPdx-R2's so, */
#define MIRISDR_OWN_RSPDUO      "D-"    /* an RSPduo's so */

/* the board behind an open handle, by its VID:PID, and under our own ids by the serial */
static int mirisdr_flavour_of (libusb_device_handle *dh)
{
    struct libusb_device_descriptor dd;
    mirisdr_device_t *d;
    unsigned char serial[MIRISDR_FW_SERIAL_MAX + 1];

    if (libusb_get_device_descriptor(libusb_get_device(dh), &dd) < 0) return MIRISDR_HW_DEFAULT;

    if ((dd.idVendor == MIRISDR_OWN_VID) && (dd.idProduct == MIRISDR_OWN_PID) && dd.iSerialNumber
        && (libusb_get_string_descriptor_ascii(dh, dd.iSerialNumber, serial, sizeof serial) >= 2))
    {
        if (!memcmp(serial, MIRISDR_OWN_RSP1B, 2)) return MIRISDR_HW_RSP1B;
        if (!memcmp(serial, MIRISDR_OWN_RSPDXR2, 2)) return MIRISDR_HW_RSPDXR2;
        if (!memcmp(serial, MIRISDR_OWN_RSPDUO, 2)) return MIRISDR_HW_RSPDUO;
    }

    d = mirisdr_device_get(dd.idVendor, dd.idProduct);

    return d ? d->flavour : MIRISDR_HW_DEFAULT;
}

/* the USB devices, or -1 with nothing to free when libusb cannot start or list them */
static ssize_t mirisdr_list_devices (libusb_context **ctx, libusb_device ***list)
{
    ssize_t n;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    if (libusb_init(ctx) < 0) return -1;

    if ((n = libusb_get_device_list(*ctx, list)) < 0)
    {
        libusb_exit(*ctx);
        return -1;
    }

    return n;
}

/* the number of devices available */
uint32_t mirisdr_get_device_count (void) {
    ssize_t i, i_max;
    uint32_t ret = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return 0;

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if (mirisdr_device_get(dd.idVendor, dd.idProduct)) ret++;
    }

    libusb_free_device_list(list, 1);

    libusb_exit(ctx);

    return ret;
}

/* the device name */
const char *mirisdr_get_device_name (uint32_t index) {
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    mirisdr_device_t *device = NULL;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return "";

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if ((device = mirisdr_device_get(dd.idVendor, dd.idProduct)) &&
            (j++ == index)) {
            libusb_free_device_list(list, 1);
            libusb_exit(ctx);
            return device->name;
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return "";
}

static int mirisdr_usb_string (libusb_device_handle *dh, uint8_t at, char *out, int len)
{
    if (!at || !dh) return 0;

    return libusb_get_string_descriptor_ascii(dh, at, (unsigned char *) out, len) > 0;
}

static int mirisdr_device_serial_cached (libusb_device *d, char *out, int len)
{
#if LIBUSBX_API_VERSION >= 0x0100010C
    if (libusb_get_device_string(d, LIBUSB_DEVICE_STRING_SERIAL_NUMBER, out, len) > 1) return 1;
#endif

#if defined(__linux__) && (LIBUSBX_API_VERSION >= 0x01000102)
    {
        uint8_t ports[7];
        char path[96];
        int n, i, at;
        size_t got;
        FILE *f;

        if ((n = libusb_get_port_numbers(d, ports, sizeof(ports))) <= 0) return 0;

        at = snprintf(path, sizeof path, "/sys/bus/usb/devices/%u", libusb_get_bus_number(d));
        for (i = 0; i < n; i++)
            at+= snprintf(path + at, sizeof path - at, i ? ".%u" : "-%u", ports[i]);
        snprintf(path + at, sizeof path - at, "/serial");

        if (!(f = fopen(path, "r"))) return 0;
        got = fgets(out, len, f) ? strcspn(out, "\n") : 0;
        fclose(f);

        out[got] = 0;
        return got > 0;
    }
#else
    (void) d; (void) out; (void) len;
    return 0;
#endif
}

static int mirisdr_device_serial (libusb_device *d, uint8_t at, char *out, int len)
{
    libusb_device_handle *dh;
    int got;

    if (!at) return 0;
    if (mirisdr_device_serial_cached(d, out, len)) return 1;
    if (libusb_open(d, &dh)) return 0;

    got = mirisdr_usb_string(dh, at, out, len);
    libusb_close(dh);

    return got;
}

// because all known SDRplay clones do not have an EEPROM chip with the serial number,
// use the physical location of the USB port
static void mirisdr_port_name (libusb_device *d, char *serial)
{
    char *cursor = serial;

    cursor+= sprintf(cursor, "%d:", libusb_get_bus_number(d));

#if LIBUSBX_API_VERSION >= 0x01000102
    {
        uint8_t usb_path[16];
        int path_len = libusb_get_port_numbers(d, usb_path, sizeof(usb_path));
        int u;

        if (path_len == LIBUSB_ERROR_OVERFLOW) path_len = sizeof(usb_path); // array too small

        for (u = 0; u < path_len; u++) cursor+= sprintf(cursor, "%d.", usb_path[u]);
    }
#endif

    *(cursor - 1) = '\0'; // remove last dot or :
}

/* our own implementation */
int mirisdr_get_device_usb_strings (uint32_t index, char *manufact, char *product, char *serial) {
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    mirisdr_device_t *device = NULL;

    if (!manufact || !product) return -1;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0)
    {
        memset(manufact, 0, 256);
        memset(product, 0, 256);
        if (serial) memset(serial, 0, 256);

        return -1;
    }

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if ((device = mirisdr_device_get(dd.idVendor, dd.idProduct)) &&
            (j++ == index)) {
            strcpy(manufact, device->manufacturer);
            strcpy(product, device->product);

            /* the serial is the only one that opens the device */
            if (serial && !mirisdr_device_serial(list[i], dd.iSerialNumber, serial, 256))
                mirisdr_port_name(list[i], serial);

            libusb_free_device_list(list, 1);
            libusb_exit(ctx);
            return 0;
        }
    }

    memset(manufact, 0, 256);
    memset(product, 0, 256);
    if (serial) memset(serial, 0, 256);

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return -1;
}

int mirisdr_get_device_port (uint32_t index, char *port, int port_len, char *serial, int serial_len)
{
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    char name[256];
    int ret = -1;

    if (!port || (port_len <= 0) || (serial && (serial_len <= 0))) return -1;

    *port = 0;
    if (serial) *serial = 0;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return -1;

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if (mirisdr_device_get(dd.idVendor, dd.idProduct) && (j++ == index)) {
            mirisdr_port_name(list[i], name);
            if ((int) strlen(name) < port_len) {
                strcpy(port, name);
                ret = 0;
            }
            if (serial && !mirisdr_device_serial(list[i], dd.iSerialNumber, serial, serial_len))
                *serial = 0;
            break;
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return ret;
}

static int mirisdr_pick_is (libusb_device *d, const struct libusb_device_descriptor *dd,
                            int own, const char *want)
{
    char have[256];

    if (!own) mirisdr_port_name(d, have);
    else if (!mirisdr_device_serial(d, dd->iSerialNumber, have, sizeof have)) *have = 0;

    return *have && !strcmp(have, want);
}

/* The receiver an open or a lookup takes from one device list. */
static libusb_device *mirisdr_pick (libusb_device **list, ssize_t n, uint32_t index,
                                    const char *serial, const char *usb_port, int *at)
{
    struct libusb_device_descriptor dd;
    char own[256];
    ssize_t i;
    int j, pass, hit;

    for (pass = 0; pass < 3; pass++) {
        if ((pass < 2) ? !serial : !usb_port) continue;

        for (i = 0, j = 0; i < n; i++) {
            if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;
            if (!mirisdr_device_get(dd.idVendor, dd.idProduct)) continue;

            if (pass < 2) hit = mirisdr_pick_is(list[i], &dd, pass, serial);
            else hit = mirisdr_pick_is(list[i], &dd, 0, usb_port)
                       && (!serial || !mirisdr_device_serial(list[i], dd.iSerialNumber, own, sizeof own));

            if (hit) {
                if (at) *at = j;
                return list[i];
            }

            j++;
        }
    }

    if (serial || usb_port) return NULL;

    for (i = 0, j = 0; i < n; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;
        if (!mirisdr_device_get(dd.idVendor, dd.idProduct)) continue;

        if ((uint32_t) j == index) {
            if (at) *at = j;
            return list[i];
        }

        j++;
    }

    return NULL;
}

int mirisdr_get_index_by_serial_port (const char *serial, const char *usb_port)
{
    ssize_t i_max;
    int found = -1;
    libusb_context *ctx;
    libusb_device **list;

    if (!serial && !usb_port) return -1;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return -1;

    if (!mirisdr_pick(list, i_max, 0, serial, usb_port, &found)) found = -1;

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return found;
}

int mirisdr_get_index_by_serial (const char *serial)
{
    return serial ? mirisdr_get_index_by_serial_port(serial, NULL) : -1;
}
