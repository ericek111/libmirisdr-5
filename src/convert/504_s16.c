
/*
 * 8 bit format
 * 1024 bytes hold 1008 values
 * structure:
 *   16b header
 *  1008b of blocks holding 1008 8b values
 */
static int mirisdr_samples_convert_504_s16 (mirisdr_dev_t *p, unsigned char* buf, uint8_t *dst8, int cnt) {
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

        /* skip the 16 byte header, 504 I+Q pairs */
        for (src+= 16, j = 0; j < 1008; j+= 2, ret+= 2) {
            /* the shift gives full scale and the right sign at once */
            dst[ret + 0] = src[j + 0] << 8;
            dst[ret + 1] = src[j + 1] << 8;
        }
    }


    /* total used bytes */
    return ret * 2;
}
