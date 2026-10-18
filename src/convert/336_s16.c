
/*
 * 12 bit format
 * 1024 bytes hold 672 values
 * structure:
 *   16b header
 *  1008b of blocks holding 672 12b values
 */
static int mirisdr_samples_convert_336_s16 (mirisdr_dev_t *p, unsigned char* buf, uint8_t *dst8, int cnt) {
    int i, i_max, j, ret = 0;
    uint8_t *src = buf;
    int16_t *dst = (int16_t*) dst8;

    /* we get 1-3 blocks of 1024 bytes */
    for (i_max = cnt >> 10, i = 0; i < i_max; i++, src+= 1008) {
        /* potentially lost data */
        if (mirisdr_addr_next(p, src, p->addr_step)) {
            src+= 16;
            continue;
        }

        /* skip the 16 byte header, 336 I+Q pairs */
        for (src+= 16, j = 0; j < 1008; j+= 3, ret+= 2) {
            /* full scale gives the right sign */
            dst[ret + 0] = ((src[j + 0] & 0xff) << 4) | ((src[j + 1] & 0x0f) << 12);
            dst[ret + 1] = ((src[j + 1] & 0xf0) << 0) | ((src[j + 2] & 0xff) << 8);
        }
    }


    /* total used bytes */
    return ret * 2;
}
