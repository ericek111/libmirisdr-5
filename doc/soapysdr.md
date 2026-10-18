Using libmirisdr receivers with SoapySDR
========================================

The `mirisdr` SoapySDR module lets programs that use SoapySDR work with libmirisdr receivers. Examples are SDR++, CubicSDR, GQRX, GNU Radio and your own programs. It supports boards with the MSi2500 and MSi001 chips, such as the SDRplay RSP1, RSP1B, RSPdx-R2 and RSPduo.

This guide explains how to install the module, how to pick it, and what each setting does.


<h3>Quick start</h3>

Check that SoapySDR finds the receiver:

    SoapySDRUtil --find="driver=mirisdr"
    SoapySDRUtil --probe="driver=mirisdr"

In a program, choose the device with the string `driver=mirisdr`. Then set a frequency, a sample rate and a gain as for any other receiver. The defaults work for most uses. The frequency you set is always at the centre of the samples you get.

To check that your computer keeps up at a rate:

    SoapySDRUtil --args="driver=mirisdr" --rate=8e6 --direction=RX


<h3>Installing</h3>

The module is built with the library when the SoapySDR development files are installed. On Debian and Raspberry Pi OS that is the package `libsoapysdr-dev`. Turn it off with `cmake -DENABLE_SOAPY=OFF`.

`make install` puts it in SoapySDR's module folder, for example `/usr/local/lib/SoapySDR/modules0.7`. A module only loads in the SoapySDR version it was built for, which is the version in the folder name. After a SoapySDR upgrade, build the module again. It builds and works with SoapySDR 0.7.2 and with the development version of October 2026.

Other modules can also drive these receivers. These are the Debian package `soapysdr0.7-module-mirisdr` (driver `miri`) and SoapyMiri (driver `soapyMiri`). If one of them is installed too, programs list the receiver once for each module. Pick this one with `driver=mirisdr`, or remove the others.


<h3>Choosing the device</h3>

The device string can hold these keys:

  - `driver=mirisdr`: this module.
  - `serial=...`: a receiver by its serial, when more than one is plugged in. `SoapySDRUtil --find` lists the serials.
  - `usb_port=...`: a receiver by the USB port it is in, such as `1:3`. `SoapySDRUtil --find` lists the port of every receiver.
  - `if_freq`, `converters`, `baseband` and `direct_sampling`: the settings of the same name, described below. They are here for programs that only let you type a device string. An example is `driver=mirisdr,if_freq=450000`.
  - `bandwidth`: the tuner filter in Hz, for programs such as GQRX that have no field for it. See Bandwidth below.
  - `gain_ranges`: `max`, the default, or `band`. See Gain below.
  - `report`: `true` logs what the receiver runs, each time it changes. See Seeing what the receiver runs below.

When opened, the receiver is at 100 MHz with a total gain of 40 dB and a rate of 2.048 Msps.

Found by `serial=` or `usb_port=`, a receiver is listed under those same keys, and it is opened by them too. A receiver plugged in or out in between does not take its place.

<h4>Receivers in use</h4>

Listing receivers and finding one does not open them, so a receiver another program is using is listed with its serial too, and that program is not disturbed. The module opens only the receiver it is asked to open. The serial is the copy the system keeps from when the receiver was plugged in. libusb 1.0.30 and later give it on Linux, Windows and macOS. With an older libusb it is read from `/sys/bus/usb/devices` on Linux. Elsewhere the module still opens each receiver for a moment to read its serial: one in use may then not open and is listed without one, and a program opening it at that same moment may fail. `usb_port=` opens nothing anywhere.

<h4>Receivers without firmware in their EEPROM</h4>

Most receivers start from the ROM in the chip. libmirisdr loads its own firmware into them when it opens them. The ROM does not report a serial, and the module does not open/update every receiver just to list it. So until a receiver has been opened once, `SoapySDRUtil --find` lists it with only its place on the USB:

    usb_port = 1:3

That is bus 1, port 3. Behind a hub more ports follow, for example `1:2.4`. `usb_port=1:3` picks the receiver for as long as it stays in that port.

After the first open the firmware keeps running until the receiver loses power. From then on its serial is listed too, whichever program opened it. A program that saved the device string with both keys, `serial=...,usb_port=1:3`, finds the receiver either way:

  - by its serial, in whatever port, while the firmware runs
  - after a power cycle, back on its ROM without a serial, as the receiver in that port. A receiver there that reports another serial is not taken in its place.

Older versions listed the port as `serial=1:3`. That still finds the receiver, also once it reports its serial.

To have a fixed serial from the moment it is plugged in, write the firmware into the receiver's EEPROM with a serial of its own. The receiver needs an EEPROM for this. Unplug it and plug it in again afterwards, so it starts from the new firmware. With more than one receiver plugged in, pick the one to write with `-d` and its index, for example `miri_eeprom -d 1 writefw ...`.

**Make a backup of the EEPROM first.** Writing the firmware replaces the original data, and without the old contents you may not be able to easily go back to vendor software. Replug the receiver, so it runs from its ROM, and read the EEPROM before any program opens it:

    miri_eeprom saferead backup.bin

Keep `backup.bin` somewhere safe. `saferead` does no do anything with the receiver but dumping the EEPROM. To put the backup back later:

    miri_eeprom write backup.bin

The firmware shows up with libmirisdr-6's own USB ids, 16d0:158c. These ids do not say which board it is, so the library tells from the serial:

  - **RSP1B**: the serial must be `B-` followed by the receiver's own 10 character SDRplay serial.

        miri_eeprom writefw default -s B-1234567890

  - **RSPdx-R2**: the same with `C-`.

  - **RSPduo**: the same with `D-`.

  - **Other boards**: any serial of up to 12 characters works. `random` makes one up.

        miri_eeprom writefw default -s random

After replugging the USB VID:PID will change, which may require retriggering udev to get access to the device.

<h3>Frequency</h3>

The module reports a range of 1 kHz to 2400 MHz, which is what the library accepts. The frequency you set is at 0 Hz in the samples, at every IF. You never need to add or subtract an IF yourself.

There is one tuning element, called `RF`, and one antenna, called `RX`. The RSPdx-R2 has three antennas instead, `A`, `B` and `C`, with `A` the default. The RSPduo's channel 0 also has `Hi-Z`, its Hi-Z input, below 30 MHz.


<h3>Sample rates</h3>

How a rate is made depends on whether it is above or below 1.3 Msps.

<h4>From 1.3 Msps up</h4>

The receiver samples directly, at zero IF. Any rate in the range works. A rate past its top gives the highest the receiver has, and the module logs it, except that an isochronous `transfer` refuses a rate it cannot carry.

  - The isochronous USB transfer, the default on Linux, reaches 12.096 Msps.
  - The bulk transfer reaches about 27.5 Msps. Above 15.75 Msps the samples cross the USB with 8 bits, and the computer has to take in up to 55 MB/s. If it cannot, samples go missing.
  - Above 14.5 Msps the receiver's own filter is bypassed, so the program gets the full width and has to filter itself.

<h4>Below 1.3 Msps</h4>

The receiver tunes to a low IF, and the library shifts the band back to the centre and filters it down in software. Any rate from about 3.5 ksps works, and you get exactly the rate you ask for.

These rates cost the least CPU, because the filters reach them by halving alone:

| IF | Rates |
|---|---|
| 450 kHz | 900000, 450000, 225000, 112500, 56250, 28125 |
| 1620 kHz | 810000, 405000, 202500, 101250, 50625 |

Any other rate adds a resampler in software. The module then logs a warning that names the nearest rates that need neither. The 2048 kHz IF always adds a frequency shift (an NCO) as well, because 4 times it would clock the receiver past its best, so it runs at 6.7584 Msps rather than 8.192. `auto` never picks it: the 1620 kHz IF has the same filters at a lower rate. It picks the IF like this:

  - A rate from the table gets its IF.
  - Up to 750 ksps, any other rate gets the 450 kHz IF, whose widest filter (600 kHz) still passes most of it.
  - Above that, it gets the 1620 kHz IF, with its 1536 kHz filter.

The module lists the table's rates, a few round ones (48k, 96k, 128k, 192k, 250k, 256k, 500k, 512k, 1M and 1.024M) and the usual zero IF rates. The rate range it reports runs from the lowest low IF rate to the top of the zero IF range.

With a fixed IF (see the `if_freq` setting) that IF takes any rate in its span:

| IF | Both converters | One converter |
|---|---|---|
| 450 kHz | 3516 to 1800000 | 3516 to 900000 |
| 1620 kHz | 12657 to 6480000 | 12657 to 3240000 |
| 2048 kHz | 13201 to 8192000 | 13201 to 4096000 |

The span stops at 4 times the IF, or 2 times with one converter, because above that the tuner's filter leaves nothing to see. A rate outside the span gives its nearest end, and the module logs which.

The module logs a warning when a rate clocks the receiver's converters past 15 MHz, which is above 7.5 Msps (15 Msps with the decimation bypass). Their effective resolution drops there, gradually. It also notes when the USB format keeps fewer than 14 bits a sample, and suggests the 1620 kHz IF when 2048 kHz is set.


<h3>Bandwidth</h3>

The bandwidths are the tuner's filters. A request picks the narrowest filter that is at least as wide as asked, or the widest if none is. The module remembers what you asked for, so a change of IF takes the nearest filter the new IF has.

Asking for 0, the default, lets the library choose:

  - At zero IF it takes the narrowest filter that is at least as wide as the sample rate, up to 8 MHz. It changes when the rate changes. For example 2.048 Msps gets 5 MHz, 6 Msps gets 6 MHz and 10 Msps gets 8 MHz. This passes the whole stream and keeps strong signals outside it away from the receiver's converters.
  - With a low IF it takes the widest filter the IF has. The software filters do the rest.

Some programs, have no field for the filter. For those, put it in the device string, for example `driver=mirisdr,bandwidth=1536000`. A program that asks for 0 then gets this filter.

| IF | Filters |
|---|---|
| Zero IF | 1.536, 5, 6, 7 and 8 MHz, and 14 MHz (no filter) |
| 450 kHz | 200, 300 and 600 kHz |
| 1620 kHz and 2048 kHz | 600 kHz and 1.536 MHz |


<h3>Gain</h3>

Set the gain as one total, from 0 to 102 dB on most bands, plus the board's amplifier on the RSP1B, RSPdx-R2 and RSPduo. The library splits it over the stages in the way that works best, with the front end first so the noise stays low. It does this again when you tune to another band.

The stages can also be set one by one:

  - `ATT`: RSP1B, RSPdx-R2 and RSPduo. The pads before the tuner, against strong signals, as a negative gain: 0, -6, -13 or -19 dB on the RSP1B and RSPduo, and 0 to -36 dB in steps of 3 on the RSPdx-R2. Another value takes the nearest pad. In L band they attenuate less, and `ATT` reads back what they give there. The RSPduo's Hi-Z has none. `ATT` is not part of the total: the total is the gain of the receiver's own chain, and setting it leaves the pads as they are. So with pads in, the stages add up to more than the total.
  - `AMP`: RSP1B, RSPdx-R2 and RSPduo. 0 or the full gain of the amplifier before the tuner in the band tuned, up to about 25 dB: from 30 MHz on the RSP1B and RSPdx-R2, from 55 MHz on the RSPduo (not on its Hi-Z). A total turns it on first, and bypasses it only below its own gain.
  - `LNA`: 0 or its full gain, which is 24 dB on most bands, 7 dB in band IV/V and about 4 dB in L band. The AM inputs have none.
  - `MIX`: 0 or 19 dB.
  - `MIXBUF`: only where the receiver uses an AM input: below 55 MHz, and from 255 to 420 MHz through the up-converter (on the RSP1 below 50 MHz only). 0 or 24 dB on AM input 2, which the 50 ohm ports use, and 0, 6, 12 or 18 dB on AM input 1, which only the RSPduo's Hi-Z uses.
  - `BB`: 0 to 59 dB.

The ranges the module reports are the most each stage has in any band, because programs such as GQRX read them only once, when the device opens. A value the band you are tuned to cannot take gives the nearest one it can, and a stage the band does not have stays at 0. For ranges that follow the band, add `gain_ranges=band` to the device string. On the RSPduo's channel 0 `MIXBUF` steps by 6: the Hi-Z takes 0, 6, 12 and 18, the 50 ohm port 0 and 24. The RSPdx-R2's `LNA` reaches 37 dB with its direct sampling amplifier, and 24 dB on the tuner. There is no automatic gain control.


<h3>Sample formats</h3>

  - `CF32`: the native format, complex float with full scale 1.0.
  - `CS16`: complex int16 with full scale 32768.
  - `CS8`: complex int8 with full scale 128.

With `baseband` off and one converter at a low IF, the samples are real instead, as `S16` or `F32`.


<h3>Settings</h3>

Programs show these as device settings. With SoapySDR's API they are `writeSetting()` and `readSetting()`.

| Setting | Values | Default | What it does |
|---|---|---|---|
| `if_freq` | `auto`, `0`, `450000`, `1620000`, `2048000` | `auto` | The tuner IF. `auto` uses zero IF from 1.3 Msps up and a low IF below, never 2048 kHz. The others fix the IF. |
| `if_in_use` | read only | | The IF in force, in Hz, 0 for zero IF: what `auto` picked. |
| `converters` | `both`, `I`, `Q` | `both` | With a low IF, use both converters or only one. |
| `baseband` | `true`, `false` | `true` | Put the frequency you set at the centre of the stream, at any IF. |
| `format` | `AUTO`, `252_S16`, `336_S16`, `384_S16`, `504_S16`, `504_S8` | `AUTO` | How samples cross the USB. `AUTO` picks the most bits that fit. |
| `transfer` | `ISOC`, `ISOC1`, `ISOC2`, `BULK` | `ISOC` on Linux, `BULK` on Windows | The USB transfer type. |
| `decimation_bypass` | `AUTO`, `ON`, `OFF` | `AUTO` | Skip the receiver's own filter. `AUTO` does this above 14.5 Msps. |
| `gap_fill` | `true`, `false` | `false` | Put zeros where samples were lost, for gaps of up to 16 USB packets. Always on with `baseband`, and then read back as `true`. |
| `biastee` | `true`, `false` | `false` | RSP1B, RSPdx-R2 and RSPduo only. Power on the antenna input: port B on the RSPdx-R2, the port marked tuner 2 (channel 1) on the RSPduo. |
| `fm_notch` | `true`, `false` | `false` | RSP1B, RSPdx-R2 and RSPduo only. Notch for FM broadcast (85 to 100 MHz). This is channel 0's. Each channel also has the notches as channel settings. |
| `mw_notch` | `true`, `false` | `false` | RSP1B, RSPdx-R2 and RSPduo only. Notch for medium wave (0.4 to 1.6 MHz). |
| `dab_notch` | `true`, `false` | `false` | RSP1B, RSPdx-R2 and RSPduo only. Notch for DAB (165 to 230 MHz). |
| `ref_out` | `true`, `false` | `false` | RSPduo only. The board's 24 MHz clock out of its reference connector, for another receiver. |
| `port_tie` | `true`, `false` | `false` | RSPduo only. Ties the two 50 ohm ports together, so both channels hear both, as from one antenna. It works at HF, 50 to 200 MHz and 600 to 900 MHz, and much less at 300 to 433 MHz and in L band. |
| `direct_sampling` | `OFF`, `ON`, `AUTO` | `AUTO` | RSPdx-R2 only. Sample the antenna directly instead of through the tuner: `ON` whenever the frequency is 2 MHz or below, `AUTO` only where the whole band (the frequency plus half the rate) fits under 2 MHz. Otherwise the tuner is used as usual. `true` and `false` work as `ON` and `OFF`. The gain is then only the LNA, about 37 dB, on from a total of 19 dB. |

A setting the receiver cannot do is refused and logged. The receiver keeps what it had. That includes a `format`, `transfer` or `decimation_bypass` that the current rate does not fit, for example the bypass at 2 Msps (it starts at 2.6) or `ISOC1` at 8 Msps: the module does not move the rate for a setting.

<h4>DC offset</h4>

With `baseband` on, the library takes the DC offset off in software: by default at a low IF, where it lies outside the band, and not at zero IF, where it sits at the band's centre. `setDCOffsetMode()` turns it on or off at every IF, and `getDCOffsetMode()` says whether it is on now. At zero IF it also removes a signal right at the centre frequency. Programs such as GQRX can do their own DC removal instead.

<h4>More about the IF settings</h4>

  - **Zero IF** needs no processing on the computer. The LO and its DC offset sit in the middle of the band.
  - **A low IF** keeps the LO and DC away from the band and has the narrow filters. The tuner rejects the other side of its LO by about 37 dB. A strong signal at the other side can leak into the stream that much weaker.
  - **Both converters** cost very little CPU. This is the best choice in most cases.
  - **One converter** (`I` or `Q`) halves the USB data and keeps 14 bits at the highest low IF rates. It needs more CPU and halves the rates on offer. The image rejection stays the same, because it comes from the tuner.
  - **`baseband` off** gives the samples as they come from the receiver, as older versions did. The tuner's LO is then at the centre, and with a low IF the band you want lies one IF below it. Use this only if your program does its own IF handling.

<h4>How much CPU</h4>

Measured on a Raspberry Pi Compute Module 0 (one core at 1 GHz), the 450 kHz IF needs little processing. A rate that is not in the table above adds the resampler: 250 ksps at 450 kHz still runs at 10 times real time, 1 Msps at 1620 kHz at 2.9 times and 3 Msps at 2 times. The 2048 kHz IF also runs the NCO, at about 1.9 times real time, so use 1620 kHz there on such a computer. Zero IF needs no processing.


<h4>Seeing what the receiver runs</h4>

The rules above decide a lot on their own. A rate below 1.3 Msps takes a low IF, the RSPdx-R2 switches to direct sampling below 2 MHz, and a rate a low IF cannot reach is taken to the nearest one it can. To see where they took the receiver, add `report=true` to the device string, for example `driver=mirisdr,report=true`. The module then logs what runs when the device opens, and again after every call that changed it. Only the lines that changed are logged, so a new gain logs one line and the same value set again logs nothing. The values come from the library, so they are what the hardware runs and not what was asked.

For example, an RSPdx-R2 set to 1 Msps at 100 MHz, and then tuned to 1 MHz:

    [INFO] mirisdr: stream: 1000000 S/s, complex float from a low IF on both converters
    [INFO] mirisdr: converters: 6480000 S/s at a 12.960 MHz clock, decimation bypass off, format 336_S16 with 12 bits, transfer ISOC, 19748571 B/s over USB
    [INFO] mirisdr: baseband: decimation 6.48 with the resampler, no NCO: the converters at 4 x IF, shifted by a quarter of their rate
    [INFO] mirisdr: tuner 0: 100.000000 MHz, LO 101.620000 MHz, low IF 1620 kHz, both outputs, filter 1.536 MHz, band VHF, in the stream at 0 Hz, DC removal on

    [INFO] mirisdr: stream: 1000000 S/s, complex float from converter I alone
    [INFO] mirisdr: converters: 4800000 S/s at a 9.600 MHz clock, decimation bypass off, format 504_REAL_S16 with 14 bits, transfer ISOC, 9752380 B/s over USB
    [INFO] mirisdr: baseband: decimation 4.8 with the resampler, NCO in use
    [INFO] mirisdr: tuner 0: 1.000000 MHz by direct sampling: the antenna through the 2 MHz low pass to converter I, no LO, in the stream at 0 Hz, DC removal on
    [INFO] mirisdr: tuner 0 gain: 37 dB, the direct sampling amplifier on

The rate below 1.3 Msps took the 1620 kHz IF. Below 2 MHz the receiver switched to direct sampling, which always needs the NCO.

The lines are:

  - `stream`: the rate the program gets and what kind of samples they are.
  - `converters`: the rate and clock of the converters, the decimation bypass, and the USB format and transfer.
  - `baseband`: with `baseband` on, how the samples are brought to 0 Hz. It gives the decimation and whether the resampler and the NCO run. At a low IF without the NCO it says why none is needed.
  - `tuner 0` (and `tuner 1` with two channels): the frequency, the LO, the IF, the outputs and the filter, or direct sampling, and where the frequency lands in the stream.
  - `tuner 0 gain`: the total and each stage.
  - `tuner 0 front end`: the antenna, the pads, the notches, the bias-T and the RSPduo's switches, on boards that have them.

These are SoapySDR log messages at the info level. `SoapySDRUtil` and most programs print them in the terminal they run in.


<h3>Stream options</h3>

These go in the arguments of `setupStream()`:

  - `buffers`: how many USB buffers the module holds for the program. The default is 256. More buffers ride out longer pauses in the consumer.
  - `transfers`: USB transfers in flight. 0, the default, uses the library's choice.


<h3>Time stamps and lost samples</h3>

Every read carries a time stamp. It counts samples from the start of the stream and turns them into nanoseconds. A stream activated again carries on from where the last one ended. `setHardwareTime()` moves the time stamps to a time you choose.

Samples can go missing in two places.

  - **On the USB**, when the computer does not take the data fast enough. With `baseband` on, the default, or with `gap_fill` on, short gaps are replaced with zeros. The stream keeps its timing and no overflow is reported. Gap fill covers up to 16 USB packets per gap, which is 2 ms at 2 Msps and less at higher rates. Where more is missing, and always with both settings off, the read stops at the gap and returns an overflow. The next read's time stamp shows how many samples are missing.
  - **In the module**, when the program reads too slowly and all its buffers are full. The read returns an overflow and SoapySDR prints `O`. The time stamp of the next read shows the jump.

If you see overflows, try a lower rate, more `buffers`, or the isochronous transfer. Bulk transfers at high rates depend most on the computer.


<h3>A short example in C++</h3>

```cpp
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <complex>
#include <cstdio>
#include <vector>

int main()
{
    SoapySDR::Device *sdr = SoapySDR::Device::make("driver=mirisdr");

    sdr->setSampleRate(SOAPY_SDR_RX, 0, 256000);
    sdr->setFrequency(SOAPY_SDR_RX, 0, 433.5e6);
    sdr->setGain(SOAPY_SDR_RX, 0, 10);

    SoapySDR::Stream *rx = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    sdr->activateStream(rx);

    std::vector<std::complex<float>> buf(4096);
    void *buffs[] = { buf.data() };
    long long total = 0;
    while (total < 256000) {
        int flags;
        long long timeNs;
        int r = sdr->readStream(rx, buffs, buf.size(), flags, timeNs, 1000000);
        if (r == SOAPY_SDR_OVERFLOW) printf("samples lost\n");
        else if (r < 0) break;
        else total += r;
    }

    sdr->deactivateStream(rx);
    sdr->closeStream(rx);
    SoapySDR::Device::unmake(sdr);
    return 0;
}
```

Build it with `g++ example.cpp -lSoapySDR`. At 256000 samples a second the module picks the 450 kHz IF by itself, and 433.5 MHz is at 0 Hz in `buf`.


<h3>Two channels</h3>

The RSPduo reports two channels, one for each tuner. Channel 0 is the tuner on the 50 ohm input marked tuner 1, channel 1 the one marked tuner 2. Frequency, gain, bandwidth and the other per channel calls act on their own tuner. The device settings (`if_freq`, the notches and the rest) act on channel 0, except `biastee`, which powers channel 1's port, and `port_tie` and `ref_out`, which are the board's. Each channel has its own pads, the `ATT` gain (`setGain(SOAPY_SDR_RX, 1, "ATT", -13)`), and its own notches, the channel settings `fm_notch`, `mw_notch` and `dab_notch`.

A channel not streamed gets no filter path, so an antenna on its port leaks less into the other. With both streamed, the ports are 44 to 63 dB apart up to 250 MHz and at 600 MHz, but only 23 to 30 dB at 300 to 433 MHz and 14 to 24 dB at 1.5 to 1.8 GHz.

`setupStream()` takes the channels to stream:

  - `{0}` or `{1}`: one tuner, as on any receiver, at the full rates.
  - `{0, 1}`: both. Each tuner runs a low IF into its own converter, and `readStream()` fills one buffer for each channel, in the order given, with the same number of samples and one time stamp. The rate is shared. `auto` picks a low IF for both: 450 kHz up to 750 ksps and 1620 kHz above, which reaches 3.24 Msps.

With both, the rate the module reports and takes is the one rate of both channels.

<h3>What it does not do</h3>

  - Transmit.
  - More than one channel, except on the RSPduo (see Two channels).
  - Automatic gain control.
  - Frequency correction.
  - Time from a PPS input. The library can do this, but it is not exposed in this module as you need custom hardware.
