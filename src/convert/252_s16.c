
/*
 * 14 bit format
 * 1024 bytes corresponds to 504 values
 * structure:
 *    16b header
 *  1008b blocks containing 504 14b values pretending to be 16b
 */
static int mirisdr_samples_convert_252_s16 (mirisdr_dev_t *p, unsigned char* buf, uint8_t *dst8, int cnt) {
    int i, i_max, j, ret = 0;
    uint8_t *src = buf;
    int16_t *dst = (int16_t*) dst8;

    /* we get 1-3 blocks of 1024 bytes each */
    for (i_max = cnt >> 10, i = 0; i < i_max; i++, src+= 1008) {
        /* potentially lost data */
        if (mirisdr_addr_next(p, src, p->addr_step)) {
            src+= 16;
            continue;
        }

        /* skip the 16 byte header, do 252 I+Q pairs */
        for (src+= 16, j = 0; j < 1008; j+= 4, ret+= 2) {
            /* maximum range */
            dst[ret + 0] = (src[j + 0] << 2) | (src[j + 1] << 10);
            dst[ret + 1] = (src[j + 2] << 2) | (src[j + 3] << 10);
        }
    }


    /* total used bytes */
    return ret * 2;
}
