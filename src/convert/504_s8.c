
/*
 * 8 bitový formát
 * 1024 bajtů odpovídá 1008 hodnotám
 * struktura:
 *   16b hlavička
 *  1008b bloků obsahujících 1008 8b hodnot
 */
static int mirisdr_samples_convert_504_s8 (mirisdr_dev_t *p, unsigned char* src, uint8_t *dst, int cnt) {
    int i, ret = 0;
    uint32_t addr;

    for (i = 0; i + 1024 <= cnt; i+= 1024, ret+= 1008) {
        addr = src[i + 3] << 24 | src[i + 2] << 16 | src[i + 1] << 8 | src[i + 0] << 0;

        /* ztracená data */
        if (p->addr != addr) {
            uint32_t lost = addr - p->addr;
            fprintf(stderr, "%u samples lost, %d, %08x:%08x\n", lost, cnt, p->addr, addr);
            p->sync_loss_cnt++;
            p->stream_lost += lost;
            p->stream_total += lost;
        }
        p->addr = addr + 504;

        memcpy(dst + ret, src + i + 16, 1008);
        p->stream_total += 504;
    }

    return ret;
}
