/* miri_test's soapy group, built when SoapySDR is found */
#ifndef MIRI_TEST_SOAPY_H
#define MIRI_TEST_SOAPY_H

#ifdef __cplusplus
extern "C" {
#endif

#define SOAPY_T_PASS 0
#define SOAPY_T_FAIL 1
#define SOAPY_T_SKIP 2

typedef void (*soapy_say_t) (const char *fmt, ...);

/* which: probe, stream, cycles, retune, bandranges, channels, regress or report. The
 * module is loaded from its path and opens the receiver itself, so the caller must have
 * closed it */
int soapy_test (const char *which, const char *module, const char *serial, int verbose,
                soapy_say_t say, soapy_say_t note);

#ifdef __cplusplus
}
#endif

#endif
