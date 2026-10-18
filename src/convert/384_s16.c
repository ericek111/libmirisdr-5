
/*
 * 10+2 bit format
 * 1024 bytes hold 768 values
 * structure:
 *   16b header
 *  984b 6 blocks of 164b
 *      160b = 128x 10b values
 *        4b = 2b left shifts
 *   24b checksum
 */
static int mirisdr_samples_convert_384_s16 (mirisdr_dev_t *p, unsigned char* buf, uint8_t *dst8, int cnt) {
    int i, i_max, j, k, ret = 0;
    uint32_t shift;
    uint8_t *src = buf;
    int16_t *dst = (int16_t*) dst8;

    /* we get 1-3 blocks of 1024 bytes, the last 24 bytes of each are not used */
    for (i_max = cnt >> 10, i = 0; i < i_max; i++, src+= 24) {
        /* potentially lost data */
        if (mirisdr_addr_next(p, src, p->addr_step)) {
            src+= 1000;
            continue;
        }

        /* skip the 16 byte header, 6 blocks, each with its 4 byte shift block at the end */
        for (src+= 16, j = 0; j < 6; j++, src+= 4) {
            /* 2 bits for each value: its shift */
            shift = src[160 + 3] << 24 | src[160 + 2] << 16 | src[160 + 1] << 8 | src[160 + 0] << 0;

            /* 16x 10 bytes */
            for (k = 0; k < 16; k++, src+= 10, ret+= 8) {
                /* 10 bytes for 8 samples, full scale gives the right sign */
                dst[ret + 0] = ((src[0] & 0xff) << 6) | ((src[1] & 0x03) << 14);
                dst[ret + 1] = ((src[1] & 0xfc) << 4) | ((src[2] & 0x0f) << 12);
                dst[ret + 2] = ((src[2] & 0xf0) << 2) | ((src[3] & 0x3f) << 10);
                dst[ret + 3] = ((src[3] & 0xc0) << 0) | ((src[4] & 0xff) <<  8);
                dst[ret + 4] = ((src[5] & 0xff) << 6) | ((src[6] & 0x03) << 14);
                dst[ret + 5] = ((src[6] & 0xfc) << 4) | ((src[7] & 0x0f) << 12);
                dst[ret + 6] = ((src[7] & 0xf0) << 2) | ((src[8] & 0x3f) << 10);
                dst[ret + 7] = ((src[8] & 0xc0) << 0) | ((src[9] & 0xff) <<  8);

                /* the right shift keeps the sign bit */
                switch ((shift >> (2 * k)) & 0x3) {
                case 0:
                    dst[ret + 0]>>= 2;
                    dst[ret + 1]>>= 2;
                    dst[ret + 2]>>= 2;
                    dst[ret + 3]>>= 2;
                    dst[ret + 4]>>= 2;
                    dst[ret + 5]>>= 2;
                    dst[ret + 6]>>= 2;
                    dst[ret + 7]>>= 2;
                    break;
                case 1:
                    dst[ret + 0]>>= 1;
                    dst[ret + 1]>>= 1;
                    dst[ret + 2]>>= 1;
                    dst[ret + 3]>>= 1;
                    dst[ret + 4]>>= 1;
                    dst[ret + 5]>>= 1;
                    dst[ret + 6]>>= 1;
                    dst[ret + 7]>>= 1;
                    break;
                /* 2 = 3 */
                }
            }
        }
    }


    /* total used bytes */
    return ret * 2;
}
