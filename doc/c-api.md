The libmirisdr C API
====================

This document explains how to use libmirisdr from C. It covers the calls that set up a receiver in one go, the way samples arrive, low IF reception with the band delivered at 0 Hz, and the extra features of the receivers.

The older calls in the style of rtl-sdr, such as `mirisdr_set_center_freq()` and `mirisdr_set_sample_rate()`, still work. New programs should preferably use the calls described here, because they check a whole configuration at once and tell you what you got.

Everything is declared in `mirisdr.h`. Link with `-lmirisdr`.


<h3>The main idea</h3>

A receiver has two configurations.

  - The tune: the frequency, the IF, the filter and the gain. It lives in a `mirisdr_tune_config_t`.
  - The stream: the sample rate, how samples cross the USB, and what the program gets. It lives in a `mirisdr_stream_config_t`.

You change either one the same way. Read the one in force, change the fields you care about and apply it. The library checks the whole configuration first. If any part of it is not possible, it refuses all of it, prints the reason on stderr and keeps what it had. It never quietly picks something else.

Each apply call can fill in a result. The result says what is really running, such as the exact rate or the LO frequency the synthesizer reached. There is also a check call for each configuration, which gives the same result without changing anything.


<h3>A complete example</h3>

This program receives 433.5 MHz through the 1620 kHz IF and gets 810000 complex float samples a second, with 433.5 MHz at 0 Hz.

```c
#include <mirisdr.h>
#include <stdio.h>

static mirisdr_dev_t *dev;
static uint64_t received;

static void callback(unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t info;
    const float *iq = (const float *) buf;   /* I, Q, I, Q, ... */

    mirisdr_get_buffer_info(dev, &info);
    if (info.gap_samples)
        printf("%llu samples missing before sample %llu\n",
               (unsigned long long) info.gap_samples, (unsigned long long) info.sample);

    received += len / (2 * sizeof(float));
    if (received >= 10 * 810000)
        mirisdr_cancel_async(dev);
    (void) iq; (void) ctx;
}

int main(void)
{
    mirisdr_tune_config_t tc;
    mirisdr_tune_result_t tr;
    mirisdr_stream_config_t sc;
    mirisdr_stream_result_t sr;

    if (mirisdr_open(&dev, 0) < 0)
        return 1;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.frequency = 433500000;
    tc.if_freq = 1620000;
    tc.bandwidth = 0;          /* the library picks */
    tc.low_if_auto = 1;        /* LO one IF above the frequency */
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = 10;
    if (mirisdr_tune(dev, 0, &tc, &tr) < 0)
        return 1;

    mirisdr_get_stream(dev, &sc, NULL);
    sc.baseband = 1;
    sc.rate = 810000;
    if (mirisdr_set_stream(dev, &sc, &sr) < 0)
        return 1;
    printf("%u samples/s from %u, LO %u Hz, bandwidth %u Hz\n",
           sr.rate, sr.adc_rate, tr.lo, tr.bandwidth);

    mirisdr_reset_buffer(dev);
    mirisdr_read_async(dev, callback, NULL, 0, 0);
    mirisdr_close(dev);
    return 0;
}
```

It prints `810000 samples/s from 6480000, LO 435120000 Hz, bandwidth 1536000 Hz`. The rest of this document explains each part.


<h3>Opening a receiver</h3>

`mirisdr_open(&dev, index)` opens a receiver by its place in the list. `mirisdr_get_device_count()`, `mirisdr_get_device_name()` and `mirisdr_get_device_usb_strings()` describe the receivers found. `mirisdr_get_index_by_serial()` finds one by its serial, or by the USB port it is listed at when it has none (`1:3`). The USB port keeps working once it has a serial. `mirisdr_get_device_port()` gives the port of any receiver in the list, and the serial it reports, empty when it reports none. `mirisdr_get_index_by_serial_port()` finds one by serial, port or both, the way `mirisdr_open_ex()` does. None of these open a receiver, so one in use by another program is not disturbed: the serial is the copy the system keeps, from libusb 1.0.30 on or from sysfs on Linux. Only with an older libusb on other systems is each receiver opened for a moment to read its serial; a port alone and `mirisdr_get_device_usb_strings()` with `serial` NULL never do.

For more control use `mirisdr_open_ex()`. Fill the config with `mirisdr_open_config_default()` first and change only what you need.

```c
mirisdr_open_config_t cfg;
mirisdr_open_config_default(&cfg);
cfg.serial = "2405278B60";
if (mirisdr_open_ex(&dev, &cfg) < 0)
    return 1;
```

The fields you are most likely to use:

  - `index`, `serial`, `usb_port` or `fd`: which receiver. A serial or port wins over the index. `serial` also takes the USB port a receiver is listed at, such as `1:3`. `fd` takes a USB file descriptor that is already open, as on Android.

With both `serial` and `usb_port`, the receiver with that serial is taken, in whatever port. When none has it, the receiver in `usb_port` is taken, but only if it reports no serial, as one running from its ROM after a power cycle. One that reports another serial is not.

Avoid opening based on an index: a receiver plugged in or out shifts the index of the others. With `serial` or `usb_port`, the receiver is matched in the same device list it is opened from, so another one cannot take its place. `mirisdr_get_index_by_serial()` followed by `mirisdr_open()` leaves a moment for that to happen. While the firmware loads, the receiver reboots and is opened again by its USB port, whichever way it was chosen.
  - `hw_flavour`: the kind of board. `MIRISDR_HW_AUTO`, the default, tells from the USB ids. Other values are `MIRISDR_HW_DEFAULT`, `MIRISDR_HW_SDRPLAY`, `MIRISDR_HW_RSP1B`, `MIRISDR_HW_RSPDXR2` and `MIRISDR_HW_RSPDUO`. A wrong value can switch things the board does not have, such as a bias-T.
  - `firmware`, `firmware_path` and `keep_running`: the library loads its own firmware into the receiver by default, and replaces a different one that is running. Give another image, or set `keep_running` to use what runs.

`mirisdr_close()` stops any stream and frees everything.


<h3>Tuning</h3>

A tune sets the frequency, the IF, the filter, which tuner outputs run and the gain, all in one request.

```c
mirisdr_tune_config_t tc;
mirisdr_tune_result_t tr;

mirisdr_get_tune(dev, 0, &tc, NULL);   /* start from what is in force */
tc.frequency = 145000000;
tc.gain.mode = MIRISDR_GAIN_KEEP;      /* leave the gain alone */
if (mirisdr_tune(dev, 0, &tc, &tr) < 0)
    printf("refused\n");
```

The second argument is the tuner, from 0. `mirisdr_get_tuner_count()` says how many there are: 2 on the RSPduo, 1 on the others. Each tuner keeps its own tune, gain and front end. The older single setters, such as `mirisdr_set_center_freq()`, act on the first tuner the stream takes. See Two tuners below.

<h4>The tune config</h4>

  - `frequency`: the frequency to receive, in Hz.
  - `if_freq`: 0 for zero IF, or a low IF of 450000, 1620000 or 2048000.
  - `bandwidth`: the tuner's filter in Hz. 0 lets the library pick. At zero IF it takes the narrowest filter at least as wide as the sample rate, up to 8 MHz, and changes it when the rate changes. For example 2.048 Msps gets 5 MHz and 10 Msps gets 8 MHz. With a low IF it takes the widest the IF has. The filters are listed below.
  - `low_if_auto`: with a low IF, put the LO one IF above `frequency`, so `frequency` is what the tuner passes. Without it the LO sits at `frequency`.
  - `iq`: with a low IF, which tuner outputs run. `MIRISDR_IQ_BOTH`, or `MIRISDR_IQ_ONLY_I` or `MIRISDR_IQ_ONLY_Q` to switch one off.
  - `gain`: see the next section.
  - `frontend`: what sits before the tuner: the bias-T, the notches, the antenna, the pads, direct sampling, the board's flags and raw words, see below.
  - `dc_removal`: with a baseband stream, whether the library takes the DC offset off in software. `MIRISDR_DC_AUTO` (the default) does at a low IF, where the DC lies outside the band, and not at zero IF, where it is the band's centre. `MIRISDR_DC_ON` does at every IF; at zero IF a signal right at the LO goes with it. `MIRISDR_DC_OFF` never does. It tracks over 50 ms, so it follows the tuner's slow drift but not a signal.
  - `override` and `synth_thresh`: expert settings. They hold the tuner's calibration codes and fix the synthesizer's grid. Leave them at 0 unless you know you need them.
  - `strict`: 0 (the default) takes a value the hardware does not have exactly to the nearest one it has, and the tune result says what is in force. A bandwidth goes to the narrowest filter of the IF at least as wide, or its widest. The pads go to the nearest step. A gain stage goes onto its steps and into its range (an LNA of 2 is on, a baseband of 60 is 59), and a total into the band's range. A pad on the RSPduo's Hi-Z, which has none, is 0. With 1, each of these is refused instead. Either way a part or mode the board does not have, or a combination it cannot do, is refused: a notch, antenna, flag or pads on a board without them, an IF it has not, the Hi-Z above 30 MHz, one output at zero IF, and so on.

The filters each IF has:

| IF | Filters |
|---|---|
| Zero IF | 1536000, 5000000, 6000000, 7000000, 8000000, and 14000000 (no filter) |
| 450 kHz | 200000, 300000, 600000 |
| 1620 kHz and 2048 kHz | 600000, 1536000 |

A filter the IF does not have is taken to the nearest it has, or refused with `strict`. This matters when you change the IF. `mirisdr_get_tune()` gives the bandwidth back as you asked for it. A 0 stays 0 and works at every IF. A filter you chose yourself, for example 8 MHz at zero IF, may not exist at the new IF: the tune then takes the IF's nearest, or with `strict` you set `bandwidth` to 0, or to a filter of the new IF, when you change `if_freq`. The tune result always says which filter is in force.

<h4>The tune result</h4>

  - `lo`: the LO frequency the synthesizer reached.
  - `offset`: where `frequency` ends up in the stream. With a baseband stream this is 0, apart from the synthesizer's small error.
  - `inverted`: with one tuner output, whether the spectrum is mirrored.
  - `iq`: the outputs that run.
  - `bandwidth`: the filter in force.
  - `gain`: the gain of each stage and the total.
  - `frontend`: the front end in force: the bias-T, notches, antenna, pads, flags, direct sampling and the raw words.
  - `band`: the tuner band, such as `MIRISDR_BAND_VHF`.

`mirisdr_tune_check()` takes the same arguments and fills the same result, without tuning. Use it to find out what would work.

The older `mirisdr_set_center_freq()` sets the LO directly. It switches `low_if_auto` off. Under a baseband stream at a low IF it sets the frequency put at 0 Hz instead, with the LO the IF above it, as any tune there does.

<h4>The front end</h4>

`tc.frontend` holds what sits before the tuner. It goes out with the tune, in the same request.

  - `bias`: the bias-T, which powers the antenna input. On the RSPdx-R2 it powers port B. On the RSPduo it powers the port marked tuner 2, so only tuner 1 takes it.
  - `notch`: notch filters, on the RSP1B, RSPdx-R2 and RSPduo only. `MIRISDR_NOTCH_FM` covers 85 to 100 MHz, `MIRISDR_NOTCH_MW` medium wave from 0.4 to 1.6 MHz, and `MIRISDR_NOTCH_DAB` 165 to 230 MHz. Combine them with `|`.
  - `antenna`: the input, `MIRISDR_ANTENNA_A` (the default), `_B` or `_C`. Only the RSPdx-R2 has B and C. On the RSPduo, tuner 0 takes `MIRISDR_ANTENNA_HIZ` (the same as B), its Hi-Z input, with the LO below 30 MHz (at a low IF the frequency is an IF lower). It goes to the tuner's other AM input, past the filters, so its mixbuffer stage steps 0, 6, 12 and 18 dB instead of 0 and 24. A gain kept across the switch keeps its dB, the mixbuffer to the nearest step the input has. The MW notch has its own switch there.
  - `attenuator`: pads before the tuner, in dB, 0 (none) by default. It is apart from the gain: the total does not count it, and a `MIRISDR_GAIN_TOTAL` tune leaves it as the config has it. Use it against strong signals, since it comes before the tuner's LNA. `mirisdr_set_attenuator()` changes it alone.
    - RSPduo and RSP1B: 6, 13 or 19 dB, one pad at a time. The RSPduo's Hi-Z input does not pass through them: a pad asked there is 0 in force, or refused with `strict`.
    - RSPdx-R2: 3 to 36 dB in 3 dB steps. Its four pads are in series, so they add: within 1 dB of the step up to 900 MHz. At 1.5 GHz the larger steps give less, 31 dB at most.
  - `flags`: the board's switches, a bit for each that is on. A flag the board does not have is refused. They belong to the board, not to a tuner: a tune of either tuner sets them for both, and both tuners' configs and results then show them, the last tune's winning. `mirisdr_set_frontend_flags()` changes them alone.
    - `MIRISDR_FRONTEND_PORT_TIE`, RSPduo only: ties the two 50 ohm ports together, so both tuners hear both, as from one antenna. It works at HF, 50 to 200 MHz and 600 to 900 MHz, where the other tuner hears a port within 5 to 10 dB of its own tuner. It works much less at 300 to 433 MHz and in L band. The port's own tuner loses 0 to 5 dB at HF and 3 to 10 dB at 50 to 600 MHz.
    - `MIRISDR_FRONTEND_REF_OUT`, RSPduo only: the board's 24 MHz clock out of its reference connector, for another receiver. Off by default.
  - `direct_sampling`: RSPdx-R2 only. See Direct sampling below.
  - `raw`: the switch words themselves, in place of the band plan's. See Raw front end words below.

A notch, an antenna, pads or direct sampling on a board without them is refused. A pad value between steps is taken to the nearest, or refused with `strict`. The config keeps what you asked for, and the tune result reports what is in force.

`mirisdr_get_frontend_caps()` says what a tuner's front end has, so a program can offer only that: the notches, the antennas and which is a Hi-Z input, the bias-T, direct sampling, the amplifier (from which frequency, and its gain in its best band), the pad values, the flags and the raw parts. It does not change while the device is open. What depends on the frequency comes with a tune result, and `mirisdr_tune_check()` says what a given tune would get.

```c
mirisdr_frontend_caps_t caps;
mirisdr_get_frontend_caps(dev, 0, &caps);
for (int k = 0; k < caps.attenuator_count; k++)
    printf("%d dB\n", caps.attenuator[k]);
```

`mirisdr_set_bias()`, `mirisdr_set_notch()` and `mirisdr_set_antenna()` still work. They change this part of the tune in force and apply it. Like the other single setters they act on the stream's first tuner. On the RSPduo that is tuner 0 unless tuner 1 is streamed alone, and only tuner 1 has the bias-T, so there `mirisdr_set_bias(p, 1)` is refused by default: set `frontend.bias` in tuner 1's tune instead.

The RSP1B and RSPdx-R2 choose their filters from the frequency. The RSPdx-R2 uses its 500 kHz low pass only when the tuner's whole filter band fits under 500 kHz, so with a low IF and a narrow filter. Otherwise it uses the 2 MHz low pass.

<h4>Raw front end words</h4>

A tune works out what the board's switches get from its band plan and the fields above: register 8's GPIO bits and, on SDRplay boards, the words of their I/O expanders. `frontend.raw` can replace any of them, for a board that is not mapped yet or a path the plan does not choose. `raw.fields` names the parts replaced:

  - `MIRISDR_FRONTEND_RAW_GPIO`: `gpio_dir` and `gpio_level`, bit n for GPIO_n: an output, and high.
  - `MIRISDR_FRONTEND_RAW_EXP1`: `expander[0]`, the expander of the RSP1B and RSPdx-R2, or the RSPduo's first.
  - `MIRISDR_FRONTEND_RAW_EXP2`: `expander[1]`, the RSPduo's second expander.

An expander word is the MCP23S18's direction word, port B in the high byte. A 0 bit pulls that pin low and a 1 leaves it to the board. A part replaced takes the word whole. The fields above no longer reach it: a raw GPIO word decides the RSPdx-R2's direct sampling switch (GPIO_2) and a plain board's bias-T (GPIO_3) too. Pins held with `mirisdr_set_gpio_direction()` and the other GPIO calls stay held. A part the board does not have, or a GPIO past 3, is refused. The words go out with the tune, before the tuner's own, and scans take them per hop.

The tune result's `frontend.raw` reports the words in force, whether replaced or not. Its `fields` names every part the board has.

<h4>Direct sampling</h4>

The RSPdx-R2 can take the antenna past the tuner, through its 2 MHz low pass, straight to the I converter. `frontend.direct_sampling` says when:

  - `MIRISDR_DS_OFF` (0): never.
  - `MIRISDR_DS_ON` (1): every tune at 2 MHz (`MIRISDR_DS_MAX_HZ`) or below.
  - `MIRISDR_DS_AUTO` (3): every tune whose band fits under 2 MHz: with a baseband stream the frequency plus half the rate, without one the frequency alone. A rate whose band stops fitting moves the tune to the tuner, and back.

Other tunes use the tuner as usual. The tune result's `frontend.direct_sampling` says which is in force, so a program can sweep across 2 MHz without doing anything itself. `mirisdr_set_direct_sampling(dev, 1)` sets it on the tune in force, as in librtlsdr, and also takes `MIRISDR_DS_AUTO`. 2 (librtlsdr's Q converter) is refused, as the RSPdx-R2 has it on I only.

  - **The stream.** Direct sampling is like a single tuner output on I whose IF is the frequency. A stream that follows the tune, or a baseband one, switches to I by itself. With baseband the frequency comes out at 0 Hz, the right way round. The converters run at 4.8 Msps whatever the frequency, so a retune only moves the NCO and the stream does not restart. When a baseband stream's band fits under 500 kHz, the 500 kHz low pass is used instead, with the converters at 1.3 Msps. Rates go from about 9.4 ksps (2.5 ksps with the 500 kHz low pass) up. Without baseband the I samples are the band itself, from 0 Hz to half the rate.
  - **The gain.** The tuner is not in the path. The only gain is an amplifier of about 37 dB (`MIRISDR_DS_LNA_DB`), on by default. With `MIRISDR_GAIN_STAGES` it is `gain.lna`. With `MIRISDR_GAIN_TOTAL` it is on from 19 dB. `mirisdr_set_lna_gain()` and `mirisdr_set_tuner_gain()` switch it the same way. The tune result then reports it alone. Leave it on unless the signals are very strong: it raises the noise floor only 4 to 5 dB.
  - **What stays.** The tune's IF, bandwidth and `iq` are kept for tunes above 2 MHz. The tune result shows `lo` 0, `iq` `MIRISDR_IQ_ONLY_I`, and the low pass's edge as `bandwidth`. Scans refuse direct sampling.
  - **Response.** Flat from about 30 kHz to 1.5 MHz, 15 dB down at 2.5 MHz. Below 30 kHz it rolls off, 8 dB down at 10 kHz.


<h3>Gain</h3>

The gain goes with the tune, in `tc.gain`. Its `mode` decides what happens.

  - `MIRISDR_GAIN_KEEP`: leave the gain as it is. The stages adapt to a new band by themselves.
  - `MIRISDR_GAIN_TOTAL`: set `total` in dB. The library splits it over the stages in the way that works best, with the front end first so noise stays low. It does this again for every band. A total above what the band can do gives the most it can do.
  - `MIRISDR_GAIN_STAGES`: set each stage yourself. The stages keep these values through later tunes.

The stages:

  - `amp_bypass`: the board's amplifier before the tuner, on the RSP1B, RSPdx-R2 and RSPduo. 0 keeps it in, 1 bypasses it. Its gain in the band, up to about 25 dB, counts in the total, and `mirisdr_get_frontend_caps()` says from which frequency it is there (30 MHz on the RSP1B and RSPdx-R2, 55 MHz on the RSPduo, not on its Hi-Z). `MIRISDR_GAIN_TOTAL` turns it on first, and bypasses it only for a total below its own gain. Elsewhere there is none, and the result reports 0.
  - `lna`: on or off. It adds 24 dB on most bands, 7 dB in band IV/V and about 4 dB in L band. The AM inputs have none.
  - `mixer`: on or off. It adds 19 dB.
  - `mixbuffer`: only on the AM inputs. 0, 6, 12 or 18 dB on AM1, 0 or 24 dB on AM2.
  - `baseband`: 0 to 59 dB.

The total therefore goes from 0 to 102 dB on most bands, plus the board's amplifier where there is one. `mirisdr_set_tuner_gain()` switches that amplifier too. The tune result reports the stages and the total in force. There is no automatic gain control.


<h3>The stream</h3>

The stream config says how fast to sample, how samples cross the USB and what the program gets.

```c
mirisdr_stream_config_t sc;
mirisdr_stream_result_t sr;

mirisdr_get_stream(dev, &sc, NULL);
sc.rate = 8000000;
sc.transfer = "BULK";
if (mirisdr_set_stream(dev, &sc, &sr) < 0)
    printf("refused\n");
printf("%u samples/s as %s, %u bytes/s on the USB\n", sr.rate, sr.format, sr.usb_bytes);
```

`mirisdr_stream_config_default()` gives a fresh config at 2 Msps with everything automatic.

<h4>The stream config</h4>

  - `rate`: samples per second.
  - `format`: how samples are packed on the USB. NULL means `"AUTO"`. See below.
  - `transfer`: `"BULK"`, `"ISOC"`, `"ISOC2"` or `"ISOC1"`. NULL is the platform's default, isochronous on Linux and other Unix systems and bulk on Windows.
  - `decimation_bypass`: `"AUTO"`, `"ON"` or `"OFF"`. NULL means `"AUTO"`. See below.
  - `swap_iq`: swaps I and Q, which mirrors the spectrum. With a real format it picks the Q converter instead of I.
  - `gap_fill`: puts zeros where samples were lost, so the timing of the stream stays right.
  - `follow_tune`: the stream switches between complex and real by itself when a tune switches a tuner output off or on.
  - `format_single`: with `follow_tune`, the real format to use for one output. NULL means `"AUTO_REAL"`.
  - `usb_capacity`: the bytes per second the automatic format may plan on. 0 is the default.
  - `baseband`: deliver complex float with the tune's frequency at 0 Hz, at any IF. This has its own section below.
  - `tuners`: the tuners to stream, bit n for tuner n. 0 means tuner 0 alone. The RSPduo also takes 2 (tuner 1 alone) and 3 (both). See Two tuners below.
  - `strict`: 0 (the default) takes a rate out of reach to the nearest one there is, and the stream result says what is in force. That covers a rate outside the converters' range for the decimation setting, past the receiver's limit for the format, or outside the span a baseband IF takes. With 1 such a rate is refused instead. Either way a combination the receiver cannot do is refused: a rate past an isochronous mode's reservation, an unknown name, two tuners without baseband, and so on. A tune that changes the IF of a baseband stream still takes the rate to the nearest the new IF has, as below.

<h4>Sample formats</h4>

Samples travel in 1 kB USB packets. A format packs a fixed number of samples in each packet. More samples per packet means fewer bits per sample but less USB data.

| Format | Samples per packet | Kind |
|---|---|---|
| `252_S16` | 252 | complex, 14 bits |
| `336_S16` | 336 | complex, 12 bits |
| `384_S16` | 384 | complex, 10 bits plus 2 |
| `504_S16` | 504 | complex, 8 bits, delivered as int16 |
| `504_S8` | 504 | complex, 8 bits, delivered as int8 |
| `504_REAL_S16` | 504 | real, one converter |
| `672_REAL_S16` | 672 | real, one converter |
| `768_REAL_S16` | 768 | real, one converter |

The USB data rate is `rate x 1024 / samples per packet` bytes a second. Complex samples arrive as I, Q pairs. Real samples come from a single converter, chosen with `swap_iq`. They only make sense with a low IF.

`"AUTO"` picks the complex format with the most bits that fits the USB. `"AUTO_REAL"` does the same among the real formats.

<h4>USB transfers and the highest rates</h4>

| Transfer | Capacity | Highest rate with 8-bit samples |
|---|---|---|
| `ISOC` | 24.576 MB/s | about 12.1 Msps |
| `ISOC2` | 16.384 MB/s | about 8 Msps |
| `ISOC1` | 8.192 MB/s | about 4 Msps |
| `BULK` | what the computer manages | about 27.5 Msps |

Isochronous transfers reserve their bandwidth on the USB, so their capacity is a hard limit. A rate that does not fit is refused, with or without `strict`.

Bulk transfers take whatever the computer gives them. The automatic format plans on 32 MB/s in bulk, which every computer tried so far reaches. Above that it still accepts the rate and uses the 8-bit format. If the computer cannot keep up, samples go missing and show up as gaps. Only the receiver's own limit of about 56 MB/s applies: a rate past it is taken down to it, or refused with `strict`. Set `usb_capacity` if you know your computer does better or worse than 32 MB/s.

<h4>Decimation bypass</h4>

The receiver normally samples at twice the rate and filters down. Its clock reaches about 15 Msps that way. Above that the bypass skips the filter and samples at the rate itself. The program then gets the full width without the receiver's own filtering and has to filter itself.

  - `"AUTO"`: bypass only above 14.5 Msps.
  - `"OFF"`: never bypass, 1.3 to 15 Msps.
  - `"ON"`: always bypass, 2.6 to 30 Msps.

<h4>The stream result</h4>

  - `rate`: the rate the sample clock really reached.
  - `format`: the format running, never an automatic one.
  - `adc`: what is sampled, `MIRISDR_IQ_BOTH` for complex or `MIRISDR_IQ_ONLY_I` or `_ONLY_Q` for real.
  - `decimation_bypassed`: whether the bypass is on.
  - `adc_clock`: the converters' clock in Hz, twice `adc_rate`, or `adc_rate` itself with the bypass on. Up to `MIRISDR_ADC_CLOCK_BEST` (15 MHz, so 7.5 Msps without the bypass) the converters are at their best. Above it their effective resolution drops, gradually, not at once. The library goes past it only when the rate asks for it, and the bundled tools and the SoapySDR module say so.
  - `bits`: the bits a sample keeps in the running format: 14, 12, 10 or 8. This is the USB limit, separate from the converter clock.
  - `usb_bytes`: bytes per second on the USB.
  - `usb_capacity`: what the automatic choice planned on.
  - `gap_fill`: whether gap fill is in effect: as set, and always under baseband.
  - `baseband`, `adc_rate` and `decimation`: see the baseband section.

`mirisdr_stream_check()` gives the result without changing anything.

<h4>Changing the stream while it runs</h4>

You can apply a new stream config while samples are flowing. The stream restarts with a short gap. The buffer being filled at that moment is dropped, so a buffer never mixes two formats.


<h3>Reading samples</h3>

<h4>The callback</h4>

`mirisdr_read_async(dev, callback, ctx, num, len)` streams until it is cancelled. It calls `callback(buf, len, ctx)` from its own thread for every buffer.

  - `num` is the number of USB transfers in flight. 0 means 32.
  - `len` is the buffer length in bytes. 0 means the buffers come as the USB delivers them, which is the best choice for most programs. A fixed length gathers samples into buffers of exactly that size.

Call `mirisdr_reset_buffer()` before you start. Stop with `mirisdr_cancel_async()`, from the callback or from another thread. A cancel counts from the moment `mirisdr_read_async()` is entered, even before the first transfer runs. A cancel made before that is lost, so keep a flag of your own and check it in the callback.

Keep the callback short. If it blocks, the USB transfers run dry and samples are lost.

<h4>What a buffer holds</h4>

Inside the callback, `mirisdr_get_buffer_info()` describes the buffer. It only works from inside the callback.

  - `type`: how each value is stored. `MIRISDR_SAMPLE_S16` is int16 with full scale 32768. `MIRISDR_SAMPLE_S8` is int8 with full scale 128. `MIRISDR_SAMPLE_F32` is float with full scale 1.0.
  - `adc`: `MIRISDR_IQ_BOTH` for complex I, Q pairs, or `MIRISDR_IQ_ONLY_I` or `_ONLY_Q` for real samples.
  - `rate`: samples per second.
  - `sample`: how many samples were delivered before this buffer.
  - `index`: the position of the first sample in time, counting lost samples too.
  - `gap_samples`, `gaps_len` and `gaps`: where samples are missing inside this buffer and how many.
  - `tuner`: the tuner the samples are from. With two tuners streamed, buffers come in pairs: tuner 0's, then tuner 1's for the same stretch of time.

Each gap has an `offset` (the samples of the buffer before it), `samples` (how many are missing) and `filled` (how many of those gap fill replaced with zeros).

The difference between `index` and `sample` is the number of samples lost so far and not filled. Use `index` to put samples on a time line.

<h4>Gap fill</h4>

With `gap_fill` set in the stream config (or `mirisdr_set_gap_fill()`), the library puts zeros where samples went missing. The stream then keeps its timing, which matters for demodulators and filters. The gaps are still reported, with `filled` set.

Gap fill only covers short gaps. It puts in at most 16 USB packets of zeros per gap, which is 2 ms at 2 Msps with 14-bit samples and less at higher rates. A longer gap gets those zeros and the rest stays missing, so `filled` is less than `samples` and the time line jumps there.

<h4>Stream statistics</h4>

`mirisdr_get_stream_stats()` gives totals since the stream started: samples delivered, samples lost, gaps, samples filled, and a few error counters. Call it from the callback, where the numbers are consistent. Its `index` is the position of the current buffer's first sample.

<h4>Reading without a callback</h4>

`mirisdr_read_sync()` reads into your buffer and returns. It gives no buffer information, so you cannot see gaps. It does not do baseband, and it needs the `BULK` transfer, which is not the default on Linux. Use `mirisdr_read_async()` where you can.


<h3>Baseband: the band at 0 Hz, at any IF</h3>

Set `baseband` in the stream config and the callback gets complex float samples with the tune's `frequency` at 0 Hz. This works the same at zero IF and at a low IF, so the program does not need to know which IF runs.

<h4>Why a low IF</h4>

At zero IF the receiver's lowest rate is 1.3 Msps, and the LO and the DC offset sit in the middle of the band. A low IF moves the band away from the LO, gives access to narrower tuner filters, and lets the library filter down to much lower rates.

<h4>How it works</h4>

  - At zero IF the samples are only converted to float, and the DC offset taken off if the tune's `dc_removal` is `MIRISDR_DC_ON`.
  - With a low IF the LO goes one IF above `frequency`. The library shifts the band by a quarter of the converter rate and removes the DC offset (see `dc_removal` in the tune). With both converters that shift is just a change of sign, which costs almost nothing.
  - With one converter (`iq` set to `MIRISDR_IQ_ONLY_I` or `_ONLY_Q` in the tune) a real to I/Q converter makes complex samples at half the rate, shifted the same way. This halves the USB data and keeps more bits at high rates, but costs more CPU.
  - When the converters run at 4 times the IF, that shift puts the band at 0 Hz. At any other converter rate an NCO, a frequency shift in software, takes it the rest of the way. The NCO is skipped at 4 times the IF.
  - Half-band filters then halve the rate as many times as needed. The last is flat to 0.4 of its output rate and 63 dB down from 0.6. The ones before it only keep aliases out of that band, so they are shorter and cheaper.
  - Where halving does not reach the rate exactly, a resampler makes the rest, a ratio between 1 and 2. It is a windowed sinc, flat to 0.4 of the output rate and 63 dB down from 0.6, as the last half-band. It keeps exact time: output m lies exactly m × `decimation` converter samples after output 0. A setting that restarts the stream starts the filters and the resampler again: the output being filled is dropped and counted as lost, and from the restart on output m lies m × `decimation` converter samples after the first one after it, give or take one converter sample of the resampler's phase.

The image rejection is the tuner's own, about 37 dB, with one converter or both.

<h4>Which rates</h4>

At zero IF any normal rate works, from 1.3 Msps. With a low IF any rate works between the IF's lowest and what the converters reach. The library picks the converter rate like this:

  - 4 times the IF, if that is at most 7 Msps. Above about 7.5 Msps (a 15 MHz converter clock) the receiver's analog performance falls off. That is 1.8 Msps at 450 kHz and 6.48 Msps at 1620 kHz.
  - Else the least rate the IF takes: 2.4 times the IF plus half its widest filter, which keeps the band clear of the receiver's own filter edge. That is 6.7584 Msps at 2048 kHz, with the NCO.
  - The output rate itself, if that is higher (twice it with one converter), with the NCO.
  - At least 1.3 Msps, or 2.6 Msps with the decimation bypass `"ON"`, and at most 15 Msps with the bypass `"OFF"`, 30 Msps otherwise. `"AUTO"` uses the bypass above 14.5 Msps.

The total decimation stays under 512, so each IF has a lowest rate:

| IF | Converter rate | Lowest rate |
|---|---|---|
| 450 kHz | 1.8 Msps | 3516 |
| 1620 kHz | 6.48 Msps | 12657 |
| 2048 kHz | 6.7584 Msps | 13201 |

The rates below need neither the resampler nor the NCO, so they cost the least CPU. The converters then run at exactly 4 times the IF:

| IF | Converter rate | Rates with both converters |
|---|---|---|
| 450 kHz | 1.8 Msps | 1800000, 900000, 450000, 225000, 112500, 56250, 28125 |
| 1620 kHz | 6.48 Msps | 6480000, 3240000, 1620000, 810000, 405000, 202500, 101250, 50625 |

With one converter, the same rates divided by 2 at least. The 2048 kHz IF has none: 4 times it is 8.192 Msps, past the receiver's best, so it runs at 6.7584 Msps with the NCO. Only an output of 8192000 (4096000 with one converter) still takes the converters to 8.192 Msps, as nothing lower delivers it. The 1620 kHz IF has the same filters at a lower rate, so prefer it. For example 250 ksps at 450 kHz runs the converters at 1.8 Msps with a decimation of 7.2, and 1 Msps at 2048 kHz runs them at 6.7584 Msps with the NCO and a decimation of 6.7584. `adc_rate` in the stream result says what they run at.

The USB limits still apply to the converter rate. A rate outside the IF's span is taken to its nearest end. With `strict` it is refused, with a message that gives the span.

A tune that changes the IF while the stream runs keeps the rate if the new IF has it, and otherwise moves it to the nearest end of the new IF's span. Check `mirisdr_get_stream()` afterwards.

<h4>The stream result with baseband</h4>

  - `baseband`: `MIRISDR_BASEBAND_ZERO_IF`, `MIRISDR_BASEBAND_COMPLEX` (low IF, both converters) or `MIRISDR_BASEBAND_REAL` (low IF, one converter).
  - `rate`: the output rate the callback gets.
  - `adc_rate`: the converters' rate.
  - `decimation`: converter samples per output sample, a `double`. A fraction where the resampler runs.
  - `nco`: whether the NCO runs, because the converters are not at 4 times the IF.
  - `resampled`: whether the resampler runs, because the rate is not the converter rate divided by a power of 2. Either costs more CPU than the rates in the table above.
  - `adc_clock` and `bits`: see the stream result above.
  - `group_delay`: the filters' delay in converter samples, a `double`. Output sample m is centred on converter sample m × `decimation` − `group_delay`, so a time stamp counted in converter samples, such as a PPS, is output sample (sample + `group_delay`) / `decimation`. 0 at zero IF.

<h4>Things to know</h4>

  - The LO moves one IF above `frequency` when the stream starts. Set `low_if_auto` in the tune and it is there from the start, and the tune result shows the right LO.
  - Buffer information counts output samples. Gaps are placed to within the filters' delay. Stream statistics, stream events, PPS and scan reports still count converter samples. Add `group_delay` and divide by `decimation` to compare.
  - Gap fill is always on, because the filters need the timing. A gap longer than gap fill covers still makes the time line jump.
  - `swap_iq` still mirrors the spectrum with both outputs. With one output it picks the converter. Two tuners refuse it.
  - A buffer length given to `mirisdr_read_async()` is in output bytes, 8 bytes for each complex sample.
  - `mirisdr_read_sync()` refuses baseband.
  - The processing runs in the callback's thread. On a Raspberry Pi Compute Module 0 (one core at 1 GHz) the 450 kHz IF needs little. Rates off the table above add the resampler: 250 ksps at 450 kHz runs at 10 times real time, 1 Msps at 1620 kHz at 2.9 times and 3 Msps at 2 times. 1 Msps at 2048 kHz, with the NCO too, runs at about 1.9 times.
  - The same signal reads about 12 dB higher with a low IF than at zero IF. This is not explained yet, so do not compare absolute levels between the two.


<h3>Two tuners</h3>

The RSPduo has two tuners. Tuner 0 is the one on the 50 ohm input marked tuner 1 (with the Hi-Z input), tuner 1 the one marked tuner 2. Each takes its own tune: `mirisdr_tune(dev, 1, &tc, &tr)` tunes tuner 1 and leaves tuner 0 as it is.

The stream config's `tuners` picks what is streamed:

  - 0 or 1: tuner 0 alone, as on any receiver. Its I and Q outputs go to the two converters.
  - 2: tuner 1 alone, the same way. The single setters then act on tuner 1.
  - 3: both. Each tuner runs a low IF into a converter of its own, tuner 0 on I and tuner 1 on Q. This needs `baseband`, a low IF on both tunes and both outputs of each tuner (`iq` both). Otherwise it is refused.

With both, the library runs two baseband chains, one per converter, each the one output path. Each takes its tuner's IF to 0 Hz, so both deliver complex float at the stream's rate. The callback gets each tuner's buffers labelled in `tuner`: tuner 0's buffer, then tuner 1's for the same stretch, with the same `sample` positions. The stream result reports `MIRISDR_BASEBAND_DUAL` and `tuners` 3.

The two tuners may use different IFs. The converters then run at the rate the higher one needs, and each chain takes its own IF the rest of the way with the NCO. The rate has to suit both IFs: from the higher of their lowest rates (the table above) up to half the converters' most, as with one output. Above twice a tuner's IF its filter leaves nothing more to see (450 kHz IF: 900 ksps, 1620 kHz: 3.24 Msps). A rate outside that is taken to its nearest end, or refused with `strict`.

Things to know:

  - Each tuner has its own filters, amplifier, pads and notches. A tuner not streamed gets no filter path, which keeps its amplifier from passing its port to the other (up to 20 dB better isolation at 300 to 433 MHz). With both streamed, the ports are 44 to 63 dB apart up to 250 MHz and at 600 MHz, but only 23 to 30 dB at 300 to 433 MHz and 14 to 24 dB at 1.5 to 1.8 GHz. The bias-T powers the port marked tuner 2, so only tuner 1 takes it. The board can join the two 50 ohm ports, so both tuners hear one antenna: `MIRISDR_FRONTEND_PORT_TIE` in `frontend.flags`. They are apart by default. Tuner 0 also takes the Hi-Z input below 30 MHz (`antenna`, see Tuning).
  - Scans run on tuner 0 alone.
  - `mirisdr_get_tuner_status()` and `mirisdr_tune_learn()` take the tuner.

<h3>Stream events and the marker</h3>

Every USB packet carries the gain in force and bits that flip when a gain, tune or rate change takes effect. `mirisdr_set_stream_events(dev, cb, ctx)` calls `cb` for each packet where something changed, before its samples are delivered. The event says which change (`MIRISDR_EVENT_GAIN`, `_TUNE`, `_RATE`, `_MARK`, or `_MISSED` when packets were lost), where in the stream it happened, and the gain of each stage.

`mirisdr_set_stream_mark(dev, on)` sets a marker bit that travels with the samples. Use it to find the exact sample where something you did took effect.


<h3>Pulse per second</h3>

A pulse on GPIO 0 can be time stamped in the stream's own samples. This needs the library's firmware, which it loads by default, and a running stream.

  - `mirisdr_enable_pps(dev, 1)` starts it. This costs one damaged packet.
  - `mirisdr_get_pps()` gives the sample of the last rising edge, an edge counter, and two flags. Ignore the sample when `trusted` or `gapless` is 0. Both repair themselves.
  - `mirisdr_set_pps_source(dev, 1, divider)` takes the edges from the USB frame clock instead of GPIO 0.


<h3>Scanning</h3>

The receiver can hop through a list of tunes by itself, one hop every few USB packets, while the computer only keeps the list fed.

  1. `mirisdr_scan_compile()` turns an array of tune configs into a scan.
  2. `mirisdr_scan_start()` starts it and calls your function for every hop as it shows in the stream, with the exact sample where it started.
  3. `mirisdr_scan_feed()` keeps the list fed. Call it often from another thread.
  4. `mirisdr_scan_stop()` ends the scan and restores the tune from before.

`mirisdr_tune_learn()` tunes once, reads back the tuner's calibration and fills the config's `override` with it. Hops with held codes settle faster. L band hops need this.

The comments in `mirisdr.h` describe the timing in detail.


<h3>Board features</h3>

  - The bias-T, the notches, the pads, the antenna input and the RSPduo's port tie are part of the tune. See the front end under Tuning. `mirisdr_get_frontend_caps()` says which ones a board has.
  - GPIO: `mirisdr_set_gpio_direction()`, `mirisdr_set_gpio_output()`, `mirisdr_get_gpio_input()` and related calls, for 4 pins.
  - I2C on GPIO 1 and 2: `mirisdr_i2c_write()`, `mirisdr_i2c_read()` and `mirisdr_i2c_transfer()`.
  - Serial output on GPIO 2: `mirisdr_uart_write()`.
  - The infrared receiver on GPIO 3: `mirisdr_set_ir()` calls a function for every pulse.
  - The EEPROM: `mirisdr_read_eeprom()`, `mirisdr_write_eeprom()` and `mirisdr_eeprom_size()`.
  - Firmware: `mirisdr_get_fw_id()` reads the id of the running image. `mirisdr_fw_get()` and `mirisdr_fw_patch()` read and change the USB ids and serial inside an image.
  - `mirisdr_get_serial()`, `mirisdr_get_usb_ids()` and `mirisdr_get_usb_position()` say which device this is and where it sits on the USB.

<h4>For experts</h4>

`mirisdr_write_reg()`, `mirisdr_write_mem()`, `mirisdr_call()`, `mirisdr_load_list()` and `mirisdr_set_tuner_override()` reach the hardware directly. Writing the wrong value can hang the receiver until it is unplugged. `mirisdr_get_tuner_status()` and `mirisdr_read_reg()` only read and are safe.


<h3>Testing without a receiver</h3>

`mirisdr_open_null(&dev, format)` opens a fake receiver. `mirisdr_feed_bulk()` pushes your own USB packets through the library as if they came from a receiver. This is how the library's own tests check the sample handling, gap reports and baseband filters.


<h3>Threads</h3>

The library follows rtl-sdr's model. A device has no lock around its calls, so the threads that use it share the work as follows:

  - **One control thread.** Tuning, gain, rate, format and the other settings, scans, GPIO, EEPROM, UART, I2C and register access all come from one thread at a time. Several of them are made of a sequence of USB requests, and the library keeps the registers it last wrote in a cache. Two control threads at once interleave those requests and let the cache go out of date. If your program has more than one thread that changes settings, hold a lock of your own around these calls.
  - **The stream thread** is the one inside `mirisdr_read_async()`. It calls your callback. While a setting restarts the stream, the control thread waits for the transfers under way to come back, and the callback can then run on the control thread, inside that setting's call, for the buffers they carry. Do not hold a lock in the callback that the control thread holds around its settings, or take a recursive one.
  - **The stream event callback** runs under a lock of the library's: once `mirisdr_set_stream_events()` returns, the old callback no longer runs, and the callback and its context always go together.
  - **From the callback**, call only `mirisdr_cancel_async()` and the calls that read the stream: `mirisdr_get_buffer_info()`, `mirisdr_get_stream_stats()` and `mirisdr_get_stream()`. A setting changed from the callback would have to stop the stream, which waits for the callback that is asking it to.
  - **`mirisdr_cancel_async()`** may come from any thread at any time, also while the control thread changes a setting that restarts the stream.
  - **`mirisdr_scan_feed()`** runs in its own thread, next to the stream and the control thread, as described under Scanning.
  - **`mirisdr_close()`** comes last, from the control thread. It cancels the stream and waits for `mirisdr_read_async()` to return, so call it outside the callback.

Settings that change the stream may be made while it runs, from the control thread: the stream stops, takes the change and starts again, so the callback never sees a setting change under it.

<h3>Room to grow</h3>

Every config and result struct ends in a `reserved` array. The register list entries and `mirisdr_call_regs_t` mirror the firmware and have none. Later versions of the library add fields there, so the struct keeps its size and programs built against this version keep working. A new field always means the old behaviour when it is 0.

  - Start a config from its default call (such as `mirisdr_tune_config_default()`) or its get call (such as `mirisdr_get_tune()`). Both leave `reserved` at 0.
  - Do not write to `reserved`. A config with anything set there is refused, with a message that the program needs a newer libmirisdr. This way a program written for a newer library fails clearly on an older one, rather than having settings quietly ignored.
  - Results come back with `reserved` at 0.
  - The structs only handed to your callbacks (stream events, scan reports and IR pulses) belong to the library. They can grow at their end.


<h3>Return values</h3>

  - 0 or more means success.
  - A negative value means the call failed or was refused. The reason is printed on stderr. The receiver keeps what it had.
  - `MIRISDR_REOPEN` (-2) means the USB handle is no longer valid and the receiver has to be opened again. This only happens after `mirisdr_reboot()` or when it was opened by file descriptor.
