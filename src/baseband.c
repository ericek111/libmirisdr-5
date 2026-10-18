/*
 * The down-converter in front of the callback (see mirisdr_stream_config_t.baseband).
 * The stream hands it the converters' samples as before, gaps filled. It gives the
 * callback complex float with the tune's frequency at 0 Hz.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "iqconverter_float.h"
#include "hb_kernel.h"

#define MIRISDR_BB_STAGES_MAX   8       /* half-band stages, so down to 1/256 */
#define MIRISDR_BB_GAPQ         64
#define MIRISDR_BB_DC_TAU       0.05    /* seconds the DC estimate averages over */
#define MIRISDR_BB_NCO_BLOCK    256     /* samples the NCO runs from one exact phase */
#define MIRISDR_RS_EXACT_MAX    256     /* a ratio with at most this many phases gets each exactly */
#define MIRISDR_RS_P            256     /* else: phases worked out, the ones between interpolated */
#define MIRISDR_RS_BETA         5.98    /* Kaiser window for 63 dB, as the last half-band */

/* GCC and Clang vector types: NEON on ARM, SSE on x86, without intrinsics */
#if defined(__GNUC__) || defined(__clang__)
#define MIRISDR_BB_V4
typedef float mirisdr_v4_t __attribute__((vector_size(16)));
typedef int mirisdr_v4i_t __attribute__((vector_size(16)));
static inline mirisdr_v4_t mirisdr_v4_load (const float *p) { mirisdr_v4_t v; memcpy(&v, p, sizeof v); return v; }
static inline void mirisdr_v4_store (float *p, mirisdr_v4_t v) { memcpy(p, &v, sizeof v); }
/* two pairs with I and Q swapped in each */
static inline mirisdr_v4_t mirisdr_v4_swap (mirisdr_v4_t v)
{
#if defined(__clang__)
    return __builtin_shufflevector(v, v, 1, 0, 3, 2);
#else
    return __builtin_shuffle(v, (mirisdr_v4i_t) { 1, 0, 3, 2 });
#endif
}
#endif

/* A half-band decimator by 2 on interleaved complex float, its kernel by where it
   stands: the last flat to 0.2 fs and 63 dB down from 0.3, the ones before it shorter,
   as all they need is to keep aliases out of the final band (see hb_kernel.h) */
typedef struct mirisdr_hb
{
    const float *k;         /* the kernel */
    int     len;
    float   *e, *o;         /* the inputs, split: e those at output positions, o the ones
                               between, from the next output's first on */
    int     en, on;         /* pairs in each */
    int     odd;            /* 1: the next input goes to o */
    int     cap;            /* pairs a block may have */
} mirisdr_hb_t;

/* The resampler after the half-band stages, from between 1 and 2 times the output rate
   to it. Output m lies m x num / den inputs after the first and is centred T / 2 inputs
   before that. A windowed sinc of T taps, flat to 0.4 of the output rate and 63 dB down
   from 0.6, as the last half-band, its taps for each output's fraction worked out once per phase (a ratio with
   few), or for P + 1 fractions and blended between the two nearest */
typedef struct mirisdr_rs
{
    int     T;              /* taps, even */
    uint64_t num, den;      /* inputs per output, reduced */
    uint64_t step, rem;     /* num as whole inputs and the den-ths left */
    uint64_t at, ph;        /* the next output: at inputs from the block's first, and ph / den */
    uint64_t mul;           /* ph x mul / 2^32 is ph x P / den: the row, and the blend below */
    int     exact;          /* den phases in c, else P + 1 */
    int     W;              /* T rounded up to whole vectors of 4, the extra taps (the
                               oldest) 0 */
    float   *c;             /* rows of W taps in input order (the oldest first): row ph
                               the taps of an output ph / den past an input, or ph / P */
    float   *xi, *xq;       /* the inputs' I and Q apart: W - 1 of history, then the block,
                               so one load of taps serves both */
    int     cap;            /* pairs the block may have */
} mirisdr_rs_t;

typedef struct mirisdr_bb
{
    int     path, stages, sign;
    int     tuner;          /* the one its samples are from, as the buffers say */
    int     low;            /* one output off a quarter of the rate (direct sampling, or a
                               tuner's low IF, see low_if): the real samples as I with Q 0, the NCO
                               taking the frequency to 0 Hz at the full rate, and a
                               half-band stage in the converter's place to filter its
                               mirror. The converter, centred on a quarter of the rate,
                               would pass only a little either side of it at the narrow
                               kernels, where the frequency is anywhere from 0 Hz */
    int     nhb;            /* half-band stages run: stages, one more when low */
    int     low_if;         /* low for a tuner's low IF (one output with the IF off a quarter
                               of the rate, or two tuners, either one so): its band at -IF, so
                               the NCO goes the other way */
    uint32_t rate;          /* the converters' */
    int     nco_move;       /* a tune's new IF + 1 for the NCO, taken at the next buffer, 0 for none */
    double  decim;          /* converter samples per output sample */
    iqconverter_float_t *cnv;
    mirisdr_hb_t hb[MIRISDR_BB_STAGES_MAX];
    mirisdr_rs_t *rs;       /* NULL where the rate is the converters' divided by a power of 2 */
    float   dc_a, dc_i, dc_q, dc_r; /* the DC estimates: I and Q, or the one output's */
    int     dc_on;          /* this buffer: the tune's dc_removal for this path */
    unsigned rot;

    /* the NCO after the quarter-rate shift: the rest of the way to 0 Hz when the
       converters do not run at 4 x IF. Phase and step in turns, 2^64 to one. A pair
       times phasor c + js is the pair times (c, c) plus it swapped times (-s, s): the
       phasors are kept in that form, so the product takes one swap per two pairs */
    uint64_t nco_ph, nco_step;
    double  nco_w1r, nco_w1i;   /* one step */
    float   nco_w4r, nco_w4i;   /* four steps */
    float   *work;
    size_t  work_cap;       /* floats */
    float   carry;          /* one output: an odd sample waiting for its pair */
    int     have_carry;

    /* output side */
    float   *out;           /* a fixed-length buffer being filled */
    uint32_t out_cap, out_n;    /* pairs */
    uint64_t sample;        /* pairs handed out */
    uint64_t produced;      /* pairs out of the filters */
    double  lost;           /* output samples missing, from the converters' unfilled gaps */
    double  lost_pend;      /* those behind the buffer being filled, counted once it is out */
    uint64_t lost_in;       /* the converters' unfilled samples seen */
    struct { uint64_t at, samples, filled; } gq[MIRISDR_BB_GAPQ];
    int     gq_n;
} mirisdr_bb_t;

static int mirisdr_hb_size (mirisdr_hb_t *h, int cap)
{
    const int H = h->len - 1;
    float *e, *o;

    /* the history and half a block, one more for an odd one */
    if (!(e = realloc(h->e, (size_t) (H / 2 + cap / 2 + 2) * 2 * sizeof(float)))) return -1;
    h->e = e;
    if (!(o = realloc(h->o, (size_t) (H / 2 + cap / 2 + 2) * 2 * sizeof(float)))) return -1;
    h->o = o;
    h->cap = cap;

    return 0;
}

/* the history zero: H / 2 pairs of each phase before the first input, which goes to e */
static void mirisdr_hb_clear (mirisdr_hb_t *h)
{
    const int H = h->len - 1;

    memset(h->e, 0, (size_t) (H / 2) * 2 * sizeof(float));
    memset(h->o, 0, (size_t) (H / 2) * 2 * sizeof(float));
    h->en = h->on = H / 2;
    h->odd = 0;
}

/* A halving's kernel by how many follow it: the last shapes the band, the one before it
   and the ones before that only keep aliases out of it (see hb_kernel.h) */
static const float *mirisdr_hb_kernel_at (int after)
{
    return after == 0 ? mirisdr_hb_last : after == 1 ? mirisdr_hb_mid : mirisdr_hb_early;
}

static int mirisdr_hb_len_at (int after)
{
    return after == 0 ? MIRISDR_HB_LAST_LEN : after == 1 ? MIRISDR_HB_MID_LEN : MIRISDR_HB_EARLY_LEN;
}

/* The halvings in a decimation, the one output's converter counted: the largest power of
   2 that rate / out holds. The resampler makes the rest, between 1 and 2 */
static uint32_t mirisdr_bb_pow2 (uint32_t rate, uint32_t out)
{
    uint32_t p2 = 1;

    while (out && ((uint64_t) p2 * 2 * out <= rate)) p2 <<= 1;
    return p2;
}

/* The resampler's taps for a ratio between 1 and 2: its transition, 0.4 to 0.6 of the
   output rate, is 0.2 / ratio of its input rate wide, which 63 dB (the last half-band's)
   takes 19.2 x ratio taps for (Kaiser's estimate), rounded up to an even number */
static int mirisdr_rs_taps (double ratio)
{
    return 2 * (int) ceil(9.6 * ratio);
}

/* The filters' delay in converter samples. A half-band stage's output m is centred on its
   input 2m - C, C its kernel's half-length, and the real to I/Q converter of the one
   output path is one more such halving, its output pair m on real samples 2m - C. With
   the long kernel last, S halvings put output m on converter sample 2^S m - D, D the sum
   of C 2^i over them from the first, each C of the kernel its place takes. The
   resampler's output m is centred on its input m x ratio - T / 2, which adds
   T / 2 x 2^S */
static double mirisdr_bb_delay (int path, uint32_t rate, uint32_t out)
{
    uint32_t p2, w;
    double d = 0;

    if ((path != MIRISDR_BASEBAND_COMPLEX && path != MIRISDR_BASEBAND_REAL && path != MIRISDR_BASEBAND_DUAL) ||
        !out) return 0;
    p2 = mirisdr_bb_pow2(rate, out);
    for (w = 1; w < p2; w <<= 1)
        d += w * (double) (mirisdr_hb_len_at((w << 1) == p2 ? 0 : (w << 2) == p2 ? 1 : 2) / 2);
    if ((uint64_t) p2 * out != rate)
        d += (double) p2 * mirisdr_rs_taps((double) rate / ((double) p2 * out)) / 2;

    return d;
}

static uint64_t mirisdr_gcd (uint64_t a, uint64_t b)
{
    while (b) { uint64_t t = a % b; a = b; b = t; }
    return a;
}

/* the kernel at t inputs, 0 to T: a sinc cut off at fc of the input rate, centred on T / 2,
   in a Kaiser window */
static double mirisdr_rs_kernel (double t, int T, double fc)
{
    double u = t - T / 2.0, s = (fabs(u) < 1e-12) ? 2 * fc : sin(2 * M_PI * fc * u) / (M_PI * u);
    double r = 2 * t / T - 1, w, i0b, i0w, k, term;

    if (r <= -1 || r >= 1) return 0;
    /* I0 by its series */
    for (i0b = 1, term = 1, k = 1; term > 1e-12 * i0b; k++) {
        term *= (MIRISDR_RS_BETA / (2 * k)) * (MIRISDR_RS_BETA / (2 * k));
        i0b += term;
    }
    w = MIRISDR_RS_BETA * sqrt(1 - r * r);
    for (i0w = 1, term = 1, k = 1; term > 1e-12 * i0w; k++) {
        term *= (w / (2 * k)) * (w / (2 * k));
        i0w += term;
    }
    return s * i0w / i0b;
}

static void mirisdr_rs_free (mirisdr_rs_t *r)
{
    if (!r) return;
    free(r->c);
    free(r->xi);
    free(r->xq);
    free(r);
}

/* num / den inputs per output, between 1 and 2 */
static mirisdr_rs_t *mirisdr_rs_create (uint64_t num, uint64_t den)
{
    mirisdr_rs_t *r = calloc(1, sizeof *r);
    uint64_t g = mirisdr_gcd(num, den);
    double fc;
    int u, ph, rows;

    if (!r) return NULL;
    r->num = num / g;
    r->den = den / g;
    r->T = mirisdr_rs_taps((double) r->num / r->den);
    r->W = (r->T + 3) & ~3;
    r->step = r->num / r->den;
    r->rem = r->num % r->den;
    r->mul = ((uint64_t) MIRISDR_RS_P << 32) / r->den;
    fc = 0.5 * r->den / r->num;     /* half the output rate, in the input's */
    r->exact = r->den <= MIRISDR_RS_EXACT_MAX;

    /* row ph: the output a fraction ph / rows past an input. Its tap on input u of the W
       it spans (the oldest first) sits W - 1 - u + ph / rows before it, the kernel 0 past
       T; each row summed to 1, so every phase passes DC alike */
    rows = r->exact ? (int) r->den : MIRISDR_RS_P;
    if (!(r->c = malloc((size_t) (rows + 1) * r->W * sizeof(float)))) goto failed;
    for (ph = 0; ph <= rows; ph++) {
        float *row = r->c + (size_t) ph * r->W;
        double sum = 0, v[64];

        for (u = 0; u < r->W; u++) sum += v[u] = mirisdr_rs_kernel(r->W - 1 - u + (double) ph / rows, r->T, fc);
        for (u = 0; u < r->W; u++) row[u] = (float) (v[u] / sum);
    }

    return r;

failed:
    mirisdr_rs_free(r);
    return NULL;
}

/* history cleared: after a restart the samples do not follow on */
static void mirisdr_rs_restart (mirisdr_rs_t *r)
{
    if (!r) return;
    if (r->xi) memset(r->xi, 0, (size_t) (r->W - 1) * sizeof(float));
    if (r->xq) memset(r->xq, 0, (size_t) (r->W - 1) * sizeof(float));
    r->at = r->ph = 0;
}

/* The sums of W inputs (I and Q apart) times W taps, W a multiple of 4. Two sums each,
   so a multiply-add need not wait for the one before */
static inline float mirisdr_v4_sum (
#ifdef MIRISDR_BB_V4
                                    mirisdr_v4_t v
#else
                                    const float *v
#endif
                                    )
{
    return (v[0] + v[1]) + (v[2] + v[3]);
}

static inline void mirisdr_rs_dot (const float *xi, const float *xq, const float *c, int W, float *yi, float *yq)
{
    int u = 0;
#ifdef MIRISDR_BB_V4
    mirisdr_v4_t i0 = { 0, 0, 0, 0 }, q0 = i0, i1 = i0, q1 = i0;

    for (; u + 8 <= W; u += 8) {
        const mirisdr_v4_t c0 = mirisdr_v4_load(c + u), c1 = mirisdr_v4_load(c + u + 4);

        i0 += mirisdr_v4_load(xi + u) * c0;
        q0 += mirisdr_v4_load(xq + u) * c0;
        i1 += mirisdr_v4_load(xi + u + 4) * c1;
        q1 += mirisdr_v4_load(xq + u + 4) * c1;
    }
    if (u < W) {
        const mirisdr_v4_t c0 = mirisdr_v4_load(c + u);

        i0 += mirisdr_v4_load(xi + u) * c0;
        q0 += mirisdr_v4_load(xq + u) * c0;
    }
    *yi = mirisdr_v4_sum(i0 + i1);
    *yq = mirisdr_v4_sum(q0 + q1);
#else
    float i = 0, q = 0;

    for (; u < W; u++) {
        i += xi[u] * c[u];
        q += xq[u] * c[u];
    }
    *yi = i;
    *yq = q;
#endif
}

/* the same with two rows of taps at once, each input loaded once */
static inline void mirisdr_rs_dot2 (const float *xi, const float *xq, const float *c0, const float *c1, int W,
                                    float *yi0, float *yq0, float *yi1, float *yq1)
{
    int u = 0;
#ifdef MIRISDR_BB_V4
    mirisdr_v4_t i0 = { 0, 0, 0, 0 }, q0 = i0, i1 = i0, q1 = i0;

    for (; u < W; u += 4) {
        const mirisdr_v4_t a = mirisdr_v4_load(xi + u), b = mirisdr_v4_load(xq + u);
        const mirisdr_v4_t h0 = mirisdr_v4_load(c0 + u), h1 = mirisdr_v4_load(c1 + u);

        i0 += a * h0;
        q0 += b * h0;
        i1 += a * h1;
        q1 += b * h1;
    }
    *yi0 = mirisdr_v4_sum(i0);
    *yq0 = mirisdr_v4_sum(q0);
    *yi1 = mirisdr_v4_sum(i1);
    *yq1 = mirisdr_v4_sum(q1);
#else
    float a0 = 0, b0 = 0, a1 = 0, b1 = 0;

    for (; u < W; u++) {
        a0 += xi[u] * c0[u];
        b0 += xq[u] * c0[u];
        a1 += xi[u] * c1[u];
        b1 += xq[u] * c1[u];
    }
    *yi0 = a0;
    *yq0 = b0;
    *yi1 = a1;
    *yq1 = b1;
#endif
}

/* n pairs in, the outputs they complete out, in place; returns how many */
static int mirisdr_rs_run (mirisdr_rs_t *r, float *io, int n)
{
    const int H = r->W - 1;
    int out = 0, k;

    if (n > r->cap) {
        float *xi = realloc(r->xi, (size_t) (H + n) * sizeof(float));
        float *xq = xi ? realloc(r->xq, (size_t) (H + n) * sizeof(float)) : NULL;

        if (!xi || !xq) {
            if (xi) r->xi = xi;
            return 0;
        }
        if (!r->xi || !r->cap) {
            memset(xi, 0, (size_t) H * sizeof(float));
            memset(xq, 0, (size_t) H * sizeof(float));
        }
        r->xi = xi;
        r->xq = xq;
        r->cap = n;
    }
    {
        float *xi = r->xi + H, *xq = r->xq + H;

        for (k = 0; k < n; k++) {
            xi[k] = io[2 * k];
            xq[k] = io[2 * k + 1];
        }
    }

    /* output at input 'at' plus ph / den: its W inputs end on 'at', at H + at */
    while (r->at < (uint64_t) n) {
        const float *xi = r->xi + r->at, *xq = r->xq + r->at;
        float yi, yq;

        if (r->exact) mirisdr_rs_dot(xi, xq, r->c + (size_t) r->ph * r->W, r->W, &yi, &yq);
        else {
            /* between the two nearest rows, linearly: the blend of the two sums */
            const uint64_t f = r->ph * r->mul;
            const float *h0 = r->c + (size_t) (f >> 32) * r->W;
            const float a = (float) (uint32_t) f * (1.0f / 4294967296.0f);
            float i0, q0, i1, q1;

            mirisdr_rs_dot2(xi, xq, h0, h0 + r->W, r->W, &i0, &q0, &i1, &q1);
            yi = i0 + a * (i1 - i0);
            yq = q0 + a * (q1 - q0);
        }

        io[2 * out] = yi;
        io[2 * out + 1] = yq;
        out++;
        r->at += r->step;
        r->ph += r->rem;
        if (r->ph >= r->den) {
            r->ph -= r->den;
            r->at++;
        }
    }

    r->at -= (uint64_t) n;
    memmove(r->xi, r->xi + n, (size_t) H * sizeof(float));
    memmove(r->xq, r->xq + n, (size_t) H * sizeof(float));

    return out;
}

static int mirisdr_hb_init (mirisdr_hb_t *h, int cap, int after)
{
    h->k = mirisdr_hb_kernel_at(after);
    h->len = mirisdr_hb_len_at(after);
    h->e = h->o = NULL;
    if (mirisdr_hb_size(h, cap) < 0) return -1;
    mirisdr_hb_clear(h);

    return 0;
}

/* n pairs in, at most n / 2 + 1 out, in place.
 *
 * With the newest input at t, output m is the sum over the kernel of h[k] x[t - k].
 * The kernel is zero at even distances from its centre C (odd), so with the outputs
 * at t = 2m + H the side taps meet only x[2m + 2i], i = 0..H/2, and the centre
 * x[2m + C]. Kept split, e the even inputs and o the odd ones: output m = sum of
 * h[2i] (e[m + i] + e[m + H/2 - i]) over the pairs, plus h[C] o[m + C/2]. Consecutive
 * outputs then read consecutive pairs, so four floats are two outputs. The input goes
 * straight into e and o, which keep from the next output's first pair on: an output is
 * made for each e past the H/2 the history holds.
 *
 * Note: Do not try to merge the two kernels into a single function, this broke GCCs
 * unrolling and costs more performance than using the large kernel everywhere... */
#ifdef MIRISDR_BB_V4
#define MIRISDR_HB_V4(KERN, PAIRS, H, C)                                                \
    for (; m + 8 <= out; m += 8) {                                                      \
        const float *oc = o + 2 * (m + C / 2);                                          \
        mirisdr_v4_t a0 = KERN[C] * mirisdr_v4_load(oc), a1 = KERN[C] * mirisdr_v4_load(oc + 4); \
        mirisdr_v4_t a2 = KERN[C] * mirisdr_v4_load(oc + 8), a3 = KERN[C] * mirisdr_v4_load(oc + 12); \
                                                                                        \
        for (i = 0; i < PAIRS; i++) {                                                   \
            const float g = KERN[2 * i];                                                \
            const float *lo = e + 2 * (m + i), *hi = e + 2 * (m + H / 2 - i);           \
                                                                                        \
            a0 += g * (mirisdr_v4_load(lo) + mirisdr_v4_load(hi));                      \
            a1 += g * (mirisdr_v4_load(lo + 4) + mirisdr_v4_load(hi + 4));              \
            a2 += g * (mirisdr_v4_load(lo + 8) + mirisdr_v4_load(hi + 8));              \
            a3 += g * (mirisdr_v4_load(lo + 12) + mirisdr_v4_load(hi + 12));            \
        }                                                                               \
        mirisdr_v4_store(io + 2 * m, a0);                                               \
        mirisdr_v4_store(io + 2 * m + 4, a1);                                           \
        mirisdr_v4_store(io + 2 * m + 8, a2);                                           \
        mirisdr_v4_store(io + 2 * m + 12, a3);                                          \
    }
#else
#define MIRISDR_HB_V4(KERN, PAIRS, H, C)
#endif

#define MIRISDR_HB_RUN(NAME, KERN, LEN)                                                 \
static int NAME (mirisdr_hb_t *h, float *io, int n)                                     \
{                                                                                       \
    enum { C = (LEN) / 2, H = (LEN) - 1, PAIRS = ((LEN) / 2 + 1) / 2 };                 \
    float *e, *o;                                                                       \
    int i, k, m, out, ne, no;                                                           \
                                                                                        \
    if (n > h->cap && mirisdr_hb_size(h, n) < 0) return 0;                              \
    e = h->e; o = h->o;                                                                 \
                                                                                        \
    /* the inputs split onto the two phases, the first where the last left off */      \
    {                                                                                   \
        const float *xe = io + 2 * h->odd, *xo = io + 2 * (1 - h->odd);                 \
                                                                                        \
        ne = (n - h->odd + 1) / 2;                                                      \
        no = (n - (1 - h->odd) + 1) / 2;                                                \
        float *ed = e + 2 * h->en, *od = o + 2 * h->on;                                  \
                                                                                        \
        for (k = 0; k < ne; k++) memcpy(ed + 2 * k, xe + 4 * k, 2 * sizeof(float));     \
        for (k = 0; k < no; k++) memcpy(od + 2 * k, xo + 4 * k, 2 * sizeof(float));     \
        h->en += ne;                                                                    \
        h->on += no;                                                                    \
        h->odd = (h->odd + n) & 1;                                                      \
    }                                                                                   \
    out = h->en - H / 2;                                                                \
                                                                                        \
    m = 0;                                                                              \
    MIRISDR_HB_V4(KERN, PAIRS, H, C)                                                    \
    for (; m < out; m++) {                                                              \
        float yi = KERN[C] * o[2 * (m + C / 2)], yq = KERN[C] * o[2 * (m + C / 2) + 1]; \
                                                                                        \
        for (i = 0; i < PAIRS; i++) {                                                   \
            const float g = KERN[2 * i];                                                \
                                                                                        \
            yi += g * (e[2 * (m + i)] + e[2 * (m + H / 2 - i)]);                        \
            yq += g * (e[2 * (m + i) + 1] + e[2 * (m + H / 2 - i) + 1]);                \
        }                                                                               \
        io[2 * m] = yi;                                                                 \
        io[2 * m + 1] = yq;                                                             \
    }                                                                                   \
                                                                                        \
    /* from the next output's first pair on */                                          \
    h->en -= out;                                                                       \
    h->on -= out;                                                                       \
    memmove(e, e + 2 * out, (size_t) h->en * 2 * sizeof(float));                        \
    memmove(o, o + 2 * out, (size_t) h->on * 2 * sizeof(float));                        \
                                                                                        \
    return out;                                                                         \
}

MIRISDR_HB_RUN(mirisdr_hb_run_last, mirisdr_hb_last, MIRISDR_HB_LAST_LEN)
MIRISDR_HB_RUN(mirisdr_hb_run_mid, mirisdr_hb_mid, MIRISDR_HB_MID_LEN)
MIRISDR_HB_RUN(mirisdr_hb_run_early, mirisdr_hb_early, MIRISDR_HB_EARLY_LEN)

static int mirisdr_hb_run (mirisdr_hb_t *h, float *io, int n)
{
    return (h->len == MIRISDR_HB_LAST_LEN) ? mirisdr_hb_run_last(h, io, n) :
           (h->len == MIRISDR_HB_MID_LEN) ? mirisdr_hb_run_mid(h, io, n) : mirisdr_hb_run_early(h, io, n);
}

static void mirisdr_bb_free_one (mirisdr_bb_t *b)
{
    int i;

    if (!b) return;
    if (b->cnv) iqconverter_float_free(b->cnv);
    mirisdr_rs_free(b->rs);
    for (i = 0; i < MIRISDR_BB_STAGES_MAX; i++) {
        free(b->hb[i].e);
        free(b->hb[i].o);
    }
    free(b->work);
    free(b->out);
    free(b);
}

static void mirisdr_bb_free (mirisdr_dev_t *p)
{
    mirisdr_bb_free_one(p->bb);
    mirisdr_bb_free_one(p->bb2);
    p->bb = p->bb2 = NULL;
    free(p->bb_split);
    p->bb_split = NULL;
    p->bb_split_cap = 0;
}

/* the filters' history only: after a restart the samples do not follow on */
static void mirisdr_bb_restart_one (mirisdr_bb_t *b)
{
    int i;

    if (!b) return;
    if (b->cnv) iqconverter_float_reset(b->cnv);
    for (i = 0; i < b->nhb; i++) {
        mirisdr_hb_clear(&b->hb[i]);
    }
    mirisdr_rs_restart(b->rs);
    b->dc_i = b->dc_q = b->dc_r = 0;
    b->rot = 0;
    b->nco_ph = 0;
    b->have_carry = 0;
    b->lost += b->out_n + b->lost_pend;    /* the buffer being filled is dropped */
    b->lost_pend = 0;
    b->out_n = 0;
    b->gq_n = 0;
    b->produced = b->sample;
}

static void mirisdr_bb_restart (mirisdr_dev_t *p)
{
    mirisdr_bb_restart_one(p->bb);
    mirisdr_bb_restart_one(p->bb2);
}

/* a new read: the counts from 0 */
static void mirisdr_bb_start_one (mirisdr_bb_t *b)
{
    if (!b) return;
    mirisdr_bb_restart_one(b);
    b->sample = b->produced = 0;
    b->lost = 0;
    b->lost_in = 0;
}

static void mirisdr_bb_start (mirisdr_dev_t *p)
{
    mirisdr_bb_start_one(p->bb);
    mirisdr_bb_start_one(p->bb2);
}

/* The band sits at IF - rate / 4 after the shift, the right way round, on both paths
   (mirrored with swap_iq and both outputs, as the shift is). The converter of one
   output runs at half the rate. At 4 x IF the step is 0 and the NCO is skipped */
static void mirisdr_bb_nco_for (mirisdr_bb_t *b, uint32_t if_hz)
{
    double at, f;

    if (b->path != MIRISDR_BASEBAND_COMPLEX && b->path != MIRISDR_BASEBAND_REAL) return;

    at = (double) b->rate / (b->path == MIRISDR_BASEBAND_REAL ? 2 : 1);
    f = ((double) if_hz - (double) b->rate / 4) / at * (b->path == MIRISDR_BASEBAND_COMPLEX ? b->sign : 1);
    if (b->low) f = (b->low_if ? 1 : -1) * (double) if_hz / b->rate;

    b->nco_step = (uint64_t) (int64_t) (f * 18446744073709551616.0);
    b->nco_w1r = cos(2 * M_PI * f);
    b->nco_w1i = sin(2 * M_PI * f);
    b->nco_w4r = (float) cos(8 * M_PI * f);
    b->nco_w4i = (float) sin(8 * M_PI * f);
}

/* A tune moved the IF (direct sampling) and nothing else: the stream's thread takes the
   new NCO at its next buffer, no restart */
static void mirisdr_bb_move (mirisdr_dev_t *p, uint32_t if_hz)
{
    p->bb_if = if_hz;
    if (p->bb) mirisdr_atomic_xchg(&p->bb->nco_move, (int) if_hz + 1);
}

/* One chain: path as it runs (two tuners run two of MIRISDR_BASEBAND_REAL), its IF and
   tuner, low_if for a tuner's IF off a quarter of the rate (the converter is flat only near
   it, so the low path takes it as for direct sampling). The counts carry on from old. NULL
   out of memory */
static mirisdr_bb_t *mirisdr_bb_create (mirisdr_dev_t *p, int path, uint32_t if_hz, int tuner, int low_if,
                                        const mirisdr_bb_t *old)
{
    mirisdr_bb_t *b;
    int i;

    if (!(b = calloc(1, sizeof *b))) return NULL;
    b->path = path;
    b->tuner = tuner;
    b->stages = p->bb_stages;
    b->decim = p->bb_rate ? (double) p->rate / p->bb_rate : 1;

    /* the band lies the IF below the LO: up by a quarter of the rate, or down with I
       and Q swapped. The converter's output of a single one is the right way round */
    b->sign = p->swap_iq ? -1 : 1;

    /* the converter is the last stage when none follow it. Low, a half-band stage with
       its kernel takes its place */
    b->low = (p->bb_ds || low_if) && (b->path == MIRISDR_BASEBAND_REAL);
    b->low_if = low_if;
    b->nhb = b->stages + b->low;
    if (b->path == MIRISDR_BASEBAND_REAL && !b->low &&
        !(b->cnv = iqconverter_float_create(mirisdr_hb_kernel_at(b->stages), mirisdr_hb_len_at(b->stages)))) goto failed;

    for (i = 0; i < b->nhb; i++)
        if (mirisdr_hb_init(&b->hb[i], 4096, b->nhb - 1 - i) < 0) goto failed;

    /* what the halvings leave, between 1 and 2: the resampler */
    if ((b->path == MIRISDR_BASEBAND_COMPLEX || b->path == MIRISDR_BASEBAND_REAL) && p->bb_rate) {
        uint64_t p2 = mirisdr_bb_pow2(p->rate, p->bb_rate);

        if (p2 * p->bb_rate != p->rate && !(b->rs = mirisdr_rs_create(p->rate, p2 * p->bb_rate))) goto failed;
    }

    b->dc_a = (float) (1.0 / (MIRISDR_BB_DC_TAU * (p->rate ? p->rate : 1)));

    b->rate = p->rate;
    mirisdr_bb_nco_for(b, if_hz);

    if (old) {
        b->sample = b->produced = old->sample;
        b->lost = old->lost + old->out_n + old->lost_pend;   /* its buffer being filled is dropped */
        b->lost_in = old->lost_in;
    }

    return b;

failed:
    mirisdr_bb_free_one(b);
    return NULL;
}

/* For the stream the plan chose (p->bb_*), from stream_regs; the counts carry on */
static int mirisdr_bb_setup (mirisdr_dev_t *p)
{
    mirisdr_bb_t *b = NULL, *b2 = NULL;
    const int dual = p->bb_path == MIRISDR_BASEBAND_DUAL;

    if (p->bb_path == MIRISDR_BASEBAND_OFF) {
        mirisdr_bb_free(p);
        return 0;
    }

    /* two tuners: tuner 0 on I, tuner 1 on Q, each as one output. A tuner's IF off 4 x IF
       on one output, with one tuner or two, takes the low path: the converter keeps
       aliases out of a band at its centre only (direct sampling takes it anyway) */
    /* Two tuners both one way: the converter path and the low one give a sample apart after
       an odd count, and their buffers would drift */
    const int off = (uint64_t) 4 * p->bb_if != p->rate, off2 = dual && ((uint64_t) 4 * p->bb_if2 != p->rate);

    if (!(b = mirisdr_bb_create(p, dual ? MIRISDR_BASEBAND_REAL : p->bb_path, p->bb_if,
                                dual ? 0 : mirisdr_tuner_primary(p),
                                dual ? (off || off2) : ((p->bb_path == MIRISDR_BASEBAND_REAL) && !p->bb_ds && off),
                                p->bb))) goto failed;
    if (dual && !(b2 = mirisdr_bb_create(p, MIRISDR_BASEBAND_REAL, p->bb_if2, 1, off || off2,
                                         p->bb2 ? p->bb2 : p->bb))) goto failed;

    mirisdr_bb_free(p);
    p->bb = b;
    p->bb2 = b2;

    return 0;

failed:
    mirisdr_bb_free_one(b);
    mirisdr_bb_free(p);
    p->bb_path = MIRISDR_BASEBAND_OFF;
    fprintf(stderr, "libmirisdr: out of memory for the baseband filters\n");

    return -1;
}

static float *mirisdr_bb_work (mirisdr_bb_t *b, size_t floats)
{
    if (floats > b->work_cap) {
        float *w = realloc(b->work, floats * sizeof(float));
        if (!w) return NULL;
        b->work = w;
        b->work_cap = floats;
    }

    return b->work;
}

/* The DC estimate moves once a buffer toward the buffer's mean (sum / n), by as much of
   the way as the buffer is of MIRISDR_BB_DC_TAU: so the loops have no chain from sample
   to sample. All three paths take it, where the tune's dc_removal has it on */
static inline void mirisdr_bb_dc_update (mirisdr_bb_t *b, float *est, double sum, int n)
{
    double f = (double) b->dc_a * n;

    if (!n) return;
    if (f > 1) f = 1;
    *est += (float) (f * (sum / n - *est));
}

/* Both outputs: one pass from the integers to float with the DC taken off (where on)
   and the shift, a quarter turn a sample, as a pattern of four from where the last
   buffer left it. After the shift the DC would sit at a quarter of the rate */
#define MIRISDR_BB_SHIFT(T)                                                             \
static int mirisdr_bb_shift_##T (mirisdr_bb_t *b, const T *in, int n, float sc, float *w) \
{                                                                                       \
    int64_t si = 0, sq = 0;                                                             \
    unsigned r = b->rot;                                                                \
    float di = 0, dq = 0;                                                               \
    int k;                                                                              \
                                                                                        \
    if (b->dc_on) {                                                                     \
        for (k = 0; k < n; k++) {                                                       \
            si += in[2 * k];                                                            \
            sq += in[2 * k + 1];                                                        \
        }                                                                               \
        mirisdr_bb_dc_update(b, &b->dc_i, (double) si * sc, n);                         \
        mirisdr_bb_dc_update(b, &b->dc_q, (double) sq * sc, n);                         \
        di = b->dc_i;                                                                   \
        dq = b->dc_q;                                                                   \
    }                                                                                   \
                                                                                        \
    for (k = 0; k < n && (r & 3); k++, r++)                                             \
        mirisdr_bb_rot1(w + 2 * k, in[2 * k] * sc - di, in[2 * k + 1] * sc - dq, b->sign, r); \
    if (b->sign > 0)                                                                    \
        for (; k + 4 <= n; k += 4, r += 4) {                                            \
            const T *p = in + 2 * k;                                                    \
            float *q = w + 2 * k;                                                       \
            q[0] = p[0] * sc - di;   q[1] = p[1] * sc - dq;                             \
            q[2] = dq - p[3] * sc;   q[3] = p[2] * sc - di;                             \
            q[4] = di - p[4] * sc;   q[5] = dq - p[5] * sc;                             \
            q[6] = p[7] * sc - dq;   q[7] = di - p[6] * sc;                             \
        }                                                                               \
    else                                                                                \
        for (; k + 4 <= n; k += 4, r += 4) {                                            \
            const T *p = in + 2 * k;                                                    \
            float *q = w + 2 * k;                                                       \
            q[0] = p[0] * sc - di;   q[1] = p[1] * sc - dq;                             \
            q[2] = p[3] * sc - dq;   q[3] = di - p[2] * sc;                             \
            q[4] = di - p[4] * sc;   q[5] = dq - p[5] * sc;                             \
            q[6] = dq - p[7] * sc;   q[7] = p[6] * sc - di;                             \
        }                                                                               \
    for (; k < n; k++, r++)                                                             \
        mirisdr_bb_rot1(w + 2 * k, in[2 * k] * sc - di, in[2 * k + 1] * sc - dq, b->sign, r); \
                                                                                        \
    b->rot = r;                                                                         \
    return n;                                                                           \
}

static inline void mirisdr_bb_rot1 (float *q, float vi, float vq, int sign, unsigned r)
{
    switch ((sign > 0 ? r : (0u - r)) & 3) {
    case 0: q[0] = vi;  q[1] = vq;  break;
    case 1: q[0] = -vq; q[1] = vi;  break;
    case 2: q[0] = -vi; q[1] = -vq; break;
    case 3: q[0] = vq;  q[1] = -vi; break;
    }
}

MIRISDR_BB_SHIFT(int16_t)
MIRISDR_BB_SHIFT(int8_t)

/* n pairs times the NCO, in place. Each block starts from the exact phase, from which
   phasors a sample apart step four samples at a time in float: no drift from block to
   block, and chains with no dependency between them */
static void mirisdr_bb_nco (mirisdr_bb_t *b, float *w, int n)
{
    const double turn = 2 * M_PI / 18446744073709551616.0;
    const float w4r = b->nco_w4r, w4i = b->nco_w4i;
    int at, k, j, m;

    for (at = 0; at < n; at += m) {
        float *x = w + 2 * at;
        float zr[4], zi[4];
        double th = (double) b->nco_ph * turn, cr = cos(th), ci = sin(th), t;

        m = (n - at < MIRISDR_BB_NCO_BLOCK) ? n - at : MIRISDR_BB_NCO_BLOCK;
        for (j = 0; j < 4; j++) {
            zr[j] = (float) cr;
            zi[j] = (float) ci;
            t = cr * b->nco_w1r - ci * b->nco_w1i;
            ci = cr * b->nco_w1i + ci * b->nco_w1r;
            cr = t;
        }

        k = 0;
#ifdef MIRISDR_BB_V4
        {
            /* samples k, k + 1 and k + 2, k + 3 as (c, c) and (-s, s); a step of four is
               c' = c wr - s wi, s' = c wi + s wr, which in that form needs no swap */
            const mirisdr_v4_t wr = { w4r, w4r, w4r, w4r }, wa = { w4i, -w4i, w4i, -w4i }, wb = -wa;
            mirisdr_v4_t c0 = { zr[0], zr[0], zr[1], zr[1] }, s0 = { -zi[0], zi[0], -zi[1], zi[1] };
            mirisdr_v4_t c1 = { zr[2], zr[2], zr[3], zr[3] }, s1 = { -zi[2], zi[2], -zi[3], zi[3] }, t;

            for (; k + 4 <= m; k += 4) {
                const mirisdr_v4_t x0 = mirisdr_v4_load(x + 2 * k), x1 = mirisdr_v4_load(x + 2 * k + 4);

                mirisdr_v4_store(x + 2 * k, x0 * c0 + mirisdr_v4_swap(x0) * s0);
                mirisdr_v4_store(x + 2 * k + 4, x1 * c1 + mirisdr_v4_swap(x1) * s1);
                t = c0 * wr + s0 * wa;
                s0 = c0 * wb + s0 * wr;
                c0 = t;
                t = c1 * wr + s1 * wa;
                s1 = c1 * wb + s1 * wr;
                c1 = t;
            }
            /* where the tail starts, from the vectors */
            zr[0] = c0[0]; zi[0] = s0[1]; zr[1] = c0[2]; zi[1] = s0[3];
            zr[2] = c1[0]; zi[2] = s1[1]; zr[3] = c1[2]; zi[3] = s1[3];
        }
#endif
        for (; k + 4 <= m; k += 4) {
            for (j = 0; j < 4; j++) {
                const float xi = x[2 * (k + j)], xq = x[2 * (k + j) + 1];
                const float r = zr[j] * w4r - zi[j] * w4i;

                x[2 * (k + j)] = xi * zr[j] - xq * zi[j];
                x[2 * (k + j) + 1] = xi * zi[j] + xq * zr[j];
                zi[j] = zr[j] * w4i + zi[j] * w4r;
                zr[j] = r;
            }
        }
        for (j = 0; k < m; k++, j++) {
            const float xi = x[2 * k], xq = x[2 * k + 1];

            x[2 * k] = xi * zr[j] - xq * zi[j];
            x[2 * k + 1] = xi * zi[j] + xq * zr[j];
        }

        b->nco_ph += b->nco_step * (uint64_t) m;
    }
}

/* The converters' samples to output pairs in b->work, returns how many */
static int mirisdr_bb_process (mirisdr_dev_t *p, mirisdr_bb_t *b, const unsigned char *buf, uint32_t len)
{
    const int s8 = p->format == MIRISDR_FORMAT_504_S8;
    const uint32_t vals = s8 ? len : len / 2;     /* int8 or int16 values */
    const float full = s8 ? 1.0f / 128.0f : 1.0f / 32768.0f;
    float *w;
    uint32_t i;
    int n, s;

    if (!(w = mirisdr_bb_work(b, (b->low ? 2 * vals : vals) + 2))) return 0;

    if ((s = mirisdr_atomic_xchg(&b->nco_move, 0))) mirisdr_bb_nco_for(b, (uint32_t) (s - 1));

    /* its tuner's tune's choice, read once a buffer. Gap fill's zeros are no signal: they
       leave the estimate, and stay 0 (the signal's mean once its DC is off), not a step of -DC */
    b->dc_on = ((p->tctx[b->tuner].tune.dc_removal == MIRISDR_DC_ON) ||
                ((p->tctx[b->tuner].tune.dc_removal == MIRISDR_DC_AUTO) && (b->path != MIRISDR_BASEBAND_ZERO_IF))) &&
               !p->bb_filling;

    if (b->path == MIRISDR_BASEBAND_COMPLEX) {
        n = s8 ? mirisdr_bb_shift_int8_t(b, (const int8_t *) buf, (int) vals / 2, full, w)
               : mirisdr_bb_shift_int16_t(b, (const int16_t *) buf, (int) vals / 2, full, w);
        if (b->nco_step) mirisdr_bb_nco(b, w, n);
    } else {
        /* one output carries the band at half the amplitude both do: twice the scale,
           so a signal comes out at the same level either way */
        const float sc = (b->path == MIRISDR_BASEBAND_REAL ? 2.0f : 1.0f) * full;

        /* the odd sample of the last buffer first */
        i = 0;
        if (b->path == MIRISDR_BASEBAND_REAL && !b->low && b->have_carry) {
            w[0] = b->carry;
            i = 1;
        }
        /* The DC off where on: the integers summed first (I and Q apart at zero IF, the
           one output's real samples), the estimate moved, then taken off in the
           conversion to float */
        float d0 = 0, d1 = 0;

        if (b->dc_on) {
            int64_t s0 = 0, s1 = 0;
            uint32_t k;

            if (s8) {
                const int8_t *in = (const int8_t *) buf;
                for (k = 0; k + 1 < vals; k += 2) { s0 += in[k]; s1 += in[k + 1]; }
                if (k < vals) s0 += in[k];
            } else {
                const int16_t *in = (const int16_t *) buf;
                for (k = 0; k + 1 < vals; k += 2) { s0 += in[k]; s1 += in[k + 1]; }
                if (k < vals) s0 += in[k];
            }
            if (b->path == MIRISDR_BASEBAND_ZERO_IF) {
                mirisdr_bb_dc_update(b, &b->dc_i, (double) s0 * sc, (int) vals / 2);
                mirisdr_bb_dc_update(b, &b->dc_q, (double) s1 * sc, (int) vals / 2);
                d0 = b->dc_i;
                d1 = b->dc_q;
            } else {
                mirisdr_bb_dc_update(b, &b->dc_r, (double) (s0 + s1) * sc, (int) vals);
                d0 = d1 = b->dc_r;
            }
        }

        /* The one output: the real samples, the DC off, times the converter's fs/4
           translation (-1, -c, 1, c a sample, c its kernel's centre tap) from where the
           last buffer left it, all in the conversion; the converter only filters after.
           x sc - d, times r, is x (sc r) - d r: one multiply-add a sample */
        /* low: the real samples as I, Q 0, the DC off, and the NCO takes the band to 0 Hz at
           the full rate and the stages filter its mirror */
        if (b->low) {
            uint32_t k;

            if (s8) {
                const int8_t *in = (const int8_t *) buf;
                for (k = 0; k < vals; k++) { w[2 * k] = in[k] * sc - d0; w[2 * k + 1] = 0; }
            } else {
                const int16_t *in = (const int16_t *) buf;
                for (k = 0; k < vals; k++) { w[2 * k] = in[k] * sc - d0; w[2 * k + 1] = 0; }
            }
        }
        else if (b->path == MIRISDR_BASEBAND_REAL) {
            const float c = b->cnv->hbc, r[4] = { -1.0f, -c, 1.0f, c };
            const int ph = b->cnv->phase;
            float a[4], o[4];
            uint32_t k;
            int j;

            for (j = 0; j < 4; j++) {
                a[j] = sc * r[(ph + j) & 3];
                o[j] = d0 * r[(ph + j) & 3];
            }
            if (s8) {
                const int8_t *in = (const int8_t *) buf;
                for (k = 0; k + 4 <= vals; k += 4)
                    for (j = 0; j < 4; j++) w[i + k + j] = in[k + j] * a[j] - o[j];
                for (j = 0; k < vals; k++, j++) w[i + k] = in[k] * a[j] - o[j];
            } else {
                const int16_t *in = (const int16_t *) buf;
                for (k = 0; k + 4 <= vals; k += 4)
                    for (j = 0; j < 4; j++) w[i + k + j] = in[k + j] * a[j] - o[j];
                for (j = 0; k < vals; k++, j++) w[i + k] = in[k] * a[j] - o[j];
            }
            b->cnv->phase = (int) ((ph + vals) & 3);
        }
        /* at zero IF I and Q alternate from the buffer's start */
        else if (s8) {
            const int8_t *in = (const int8_t *) buf;
            uint32_t k;
            for (k = 0; k + 1 < vals; k += 2) {
                w[i + k] = in[k] * sc - d0;
                w[i + k + 1] = in[k + 1] * sc - d1;
            }
            if (k < vals) w[i + k] = in[k] * sc - d0;
        } else {
            const int16_t *in = (const int16_t *) buf;
            uint32_t k;
            for (k = 0; k + 1 < vals; k += 2) {
                w[i + k] = in[k] * sc - d0;
                w[i + k + 1] = in[k + 1] * sc - d1;
            }
            if (k < vals) w[i + k] = in[k] * sc - d0;
        }
        n = (int) (vals + i);

        if (b->low) {
            n = (int) vals;
            if (b->nco_step) mirisdr_bb_nco(b, w, n);
        } else {
            if (b->path == MIRISDR_BASEBAND_REAL) {
                b->have_carry = n & 1;
                if (b->have_carry) b->carry = w[--n];
                b->cnv->fir(b->cnv, w, n);  /* translated above, the carry with its buffer */
            }
            n /= 2;
            if (b->path == MIRISDR_BASEBAND_REAL && b->nco_step) mirisdr_bb_nco(b, w, n);
        }
    }

    for (s = 0; s < b->nhb; s++) n = mirisdr_hb_run(&b->hb[s], w, n);
    if (b->rs) n = mirisdr_rs_run(b->rs, w, n);

    return n;
}

/* One output buffer to the callback, with its info in output samples */
static void mirisdr_bb_call (mirisdr_dev_t *p, mirisdr_bb_t *b, float *iq, uint32_t n)
{
    mirisdr_buffer_info_t *in = &p->cb_info;
    uint64_t start = b->sample, end = start + n;
    int k = 0, j;

    memset(in, 0, sizeof *in);
    in->sample = start;
    in->index = start + (uint64_t) (b->lost + 0.5);
    in->adc = MIRISDR_IQ_BOTH;
    in->rate = p->bb_rate;
    in->type = MIRISDR_SAMPLE_F32;
    in->tuner = b->tuner;

    while (k < b->gq_n && b->gq[k].at < end) {
        if (in->gaps_len < MIRISDR_GAPS_MAX) {
            mirisdr_gap_t *g = &in->gaps[in->gaps_len++];

            g->offset = b->gq[k].at > start ? (uint32_t) (b->gq[k].at - start) : 0;
            g->samples = b->gq[k].samples;
            g->filled = (uint32_t) b->gq[k].filled;
        }
        in->gap_samples += b->gq[k].samples;
        k++;
    }
    for (j = k; j < b->gq_n; j++) b->gq[j - k] = b->gq[j];
    b->gq_n -= k;

    b->sample = end;
    b->lost += b->lost_pend;
    b->lost_pend = 0;
    p->cb((unsigned char *) iq, n * 2 * sizeof(float), p->cb_ctx);
}

/* One chain's share of a converters' buffer, in its info (whose counts each converter
   shares. It is a copy, as the deliveries overwrite p->cb_info) */
static void mirisdr_bb_feed_one (mirisdr_dev_t *p, mirisdr_bb_t *b, unsigned char *buf, uint32_t len,
                                 const mirisdr_buffer_info_t *in)
{
    uint64_t lost_in = in->index - in->sample;
    uint32_t k, n, at, chunk;
    float *w;

    /* the unfilled gaps before this buffer, as output samples. Behind a buffer being filled
       they come after its start, so its index leaves them out */
    if (lost_in > b->lost_in) {
        double d = (double) (lost_in - b->lost_in) / b->decim;

        if (b->out_n) b->lost_pend += d;
        else b->lost += d;
    }
    b->lost_in = lost_in;

    /* its gaps, where they come out */
    for (k = 0; k < in->gaps_len; k++) {
        uint64_t samples = (uint64_t) ceil(in->gaps[k].samples / b->decim);
        /* filled whole stays filled whole, or the rounding reads as a loss */
        uint32_t filled = (in->gaps[k].filled == in->gaps[k].samples) ? (uint32_t) samples
                                                                       : (uint32_t) (in->gaps[k].filled / b->decim);

        /* a full queue: the last one takes it, so the counts stay right */
        if (b->gq_n == MIRISDR_BB_GAPQ) {
            b->gq[b->gq_n - 1].samples += samples;
            b->gq[b->gq_n - 1].filled += filled;
            continue;
        }
        b->gq[b->gq_n].at = b->produced + (uint64_t) (in->gaps[k].offset / b->decim);
        b->gq[b->gq_n].samples = samples;
        b->gq[b->gq_n].filled = filled;
        b->gq_n++;
    }

    n = (uint32_t) mirisdr_bb_process(p, b, buf, len);
    w = b->work;
    b->produced += n;

    /* buffers as they come */
    if (!p->user_out_len || p->user_out_len < 2 * sizeof(float)) {
        if (n) mirisdr_bb_call(p, b, w, n);
        return;
    }

    /* or of the size asked for */
    if (!b->out || b->out_cap != p->user_out_len / (2 * sizeof(float))) {
        float *o = realloc(b->out, p->user_out_len);
        if (!o) return;
        b->out = o;
        b->out_cap = (uint32_t) (p->user_out_len / (2 * sizeof(float)));
        b->out_n = 0;
    }

    for (at = 0; at < n; at += chunk) {
        chunk = b->out_cap - b->out_n;
        if (chunk > n - at) chunk = n - at;

        if (!b->out_n && chunk == b->out_cap) {
            mirisdr_bb_call(p, b, w + 2 * at, chunk);
            continue;
        }
        memcpy(b->out + 2 * b->out_n, w + 2 * at, (size_t) chunk * 2 * sizeof(float));
        b->out_n += chunk;
        if (b->out_n == b->out_cap) {
            mirisdr_bb_call(p, b, b->out, b->out_cap);
            b->out_n = 0;
        }
    }
}

/* From mirisdr_cb_call, with the converters' buffer and its info in p->cb_info. Two
   tuners: the I values are tuner 0's real samples and the Q values tuner 1's, each chain
   takes its own. Tuner 0's buffers come first, then tuner 1's for the same span */
static void mirisdr_bb_feed (mirisdr_dev_t *p, unsigned char *buf, uint32_t len)
{
    const mirisdr_buffer_info_t in = p->cb_info;
    const uint32_t half = len / 2;
    uint32_t k, n;

    if (!p->bb2) {
        mirisdr_bb_feed_one(p, p->bb, buf, len, &in);
        return;
    }

    if (len > p->bb_split_cap) {
        uint8_t *s = realloc(p->bb_split, len);
        if (!s) return;
        p->bb_split = s;
        p->bb_split_cap = len;
    }

    if (p->format == MIRISDR_FORMAT_504_S8) {
        const int8_t *in = (const int8_t *) buf;
        int8_t *o = (int8_t *) p->bb_split;
        n = len / 2;
        for (k = 0; k < n; k++) {
            o[k] = in[2 * k];
            o[n + k] = in[2 * k + 1];
        }
    } else {
        const int16_t *in = (const int16_t *) buf;
        int16_t *o = (int16_t *) p->bb_split;
        n = len / 4;
        for (k = 0; k < n; k++) {
            o[k] = in[2 * k];
            o[n + k] = in[2 * k + 1];
        }
    }

    mirisdr_bb_feed_one(p, p->bb, p->bb_split, half, &in);
    mirisdr_bb_feed_one(p, p->bb2, p->bb_split + half, half, &in);
}
