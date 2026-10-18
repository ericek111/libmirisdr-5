
/*
 * 8 bit format
 * 1024 bytes hold 1008 values
 * structure:
 *   16b header
 *  1008b of blocks holding 1008 8b values
 */
static int mirisdr_samples_convert_504_s8 (mirisdr_dev_t *p, unsigned char* src, uint8_t *dst, int cnt) {
    int i, ret = 0;

    /* only whole blocks: a transfer cut short by a resynchronisation ends in a
       partial one, and the header of the next block is not where it looks */
    for (i = 0; i + 1024 <= cnt; i+= 1024) {
        unsigned char *b = src + i;

        /* lost data */
        if (mirisdr_addr_next(p, b, p->addr_step)) continue;

        memcpy(dst + ret, b + 16, 1008);
        ret+= 1008;
    }

    return ret;
}
