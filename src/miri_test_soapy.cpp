/*
 * miri_test's soapy group: the SoapySDR module, loaded and used as an
 * application would. C++ because SoapySDR's C API differs between 0.7 and 0.8.
 */

#include "miri_test_soapy.h"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Modules.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

typedef SoapySDR::Device Dev;

static double now(void)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* quiet unless verbose: the module and library log refusals on purpose */
static void quietLog(const SoapySDRLogLevel, const char *) {}

static Dev *openDev(const char *module, const char *serial, const char *gainRanges, soapy_say_t say,
                    const SoapySDR::Kwargs &extra = SoapySDR::Kwargs())
{
    static std::string loaded;
    SoapySDR::Kwargs args;
    std::string err;

    if (loaded != module)
    {
        err = SoapySDR::loadModule(module);
        if (!err.empty() && err.find("already loaded") == std::string::npos)
        {
            say("cannot load %s: %s", module, err.c_str());
            return nullptr;
        }
        loaded = module;
    }

    args["driver"] = "mirisdr";
    if (serial) args["serial"] = serial;
    if (gainRanges) args["gain_ranges"] = gainRanges;
    for (const auto &kv : extra) args[kv.first] = kv.second;
    try { return Dev::make(args); }
    catch (const std::exception &e) { say("make failed: %s", e.what()); }
    return nullptr;
}

static bool has(const std::vector<std::string> &v, const char *s)
{
    return std::find(v.begin(), v.end(), s) != v.end();
}

/* a channel setting's options, -1 when the channel has not got it */
static int chanOptions(Dev *d, size_t ch, const std::string &key)
{
    for (const auto &a : d->getSettingInfo(SOAPY_SDR_RX, ch))
        if (a.key == key) return (int) a.options.size();
    return -1;
}

/* names, formats, ranges and the gain stages a band has */
static int probe(Dev *d, soapy_say_t say, soapy_say_t note)
{
    std::vector<std::string> f = d->getStreamFormats(SOAPY_SDR_RX, 0), g;
    SoapySDR::RangeList fr = d->getFrequencyRange(SOAPY_SDR_RX, 0), rr = d->getSampleRateRange(SOAPY_SDR_RX, 0);
    std::vector<double> bw;
    double top;
    double full;
    int bad = 0;

    if (d->getDriverKey() != "mirisdr") { say("driver %s", d->getDriverKey().c_str()); bad++; }
    if (d->getNumChannels(SOAPY_SDR_RX) != (d->getHardwareKey() == "RSPduo" ? 2u : 1u) || d->getNumChannels(SOAPY_SDR_TX) != 0)
    { say("channels wrong: %zu", d->getNumChannels(SOAPY_SDR_RX)); bad++; }
    if (!has(f, SOAPY_SDR_CS16) || !has(f, SOAPY_SDR_CF32) || !has(f, SOAPY_SDR_CS8)) { say("formats missing"); bad++; }
    if (d->getNativeStreamFormat(SOAPY_SDR_RX, 0, full) != SOAPY_SDR_CF32 || full != 1.0) { say("native format not CF32 in baseband"); bad++; }
    if (fr.empty() || fr[0].minimum() > 1e6 || fr[0].maximum() < 2e9) { say("frequency range wrong"); bad++; }
    /* one span: a low IF's from a few ksps, zero IF's up to the top */
    if (rr.size() != 1 || rr[0].minimum() > 10e3 || rr[0].maximum() < 8e6)
    { say("rate range wrong: %zu spans, %.0f to %.0f", rr.size(), rr.empty() ? 0 : rr[0].minimum(),
          rr.empty() ? 0 : rr.back().maximum()); bad++; }
    else {
        /* the list: low IF rates needing no NCO, the usual round ones, zero IF's */
        std::vector<double> lr = d->listSampleRates(SOAPY_SDR_RX, 0);
        auto in = [&lr](double r) { return std::find(lr.begin(), lr.end(), r) != lr.end(); };

        if (!in(900000) || !in(256000) || !in(250000) || !in(2048000))
        { say("rates 900k, 256k, 250k and 2048k not all listed"); bad++; }
        note("frequency %.3f-%.0f MHz, rates %.0f sps to %.2f Msps, %zu listed", fr[0].minimum() / 1e6,
             fr[0].maximum() / 1e6, rr[0].minimum(), rr[0].maximum() / 1e6, lr.size());
    }

    /* the board's amplifier before the tuner: the RSPduo's and RSP1B's 25 dB in band II, their
       most, and the RSPdx-R2's 24 there, 25 at most (band III) */
    const std::string hw = d->getHardwareKey();
    const bool duo = hw == "RSPduo", dxr2 = hw == "RSPdxR2", rsp1b = hw == "RSP1B";
    const double amp = (duo || rsp1b) ? 25 : dxr2 ? 24 : 0, ampMax = (duo || dxr2 || rsp1b) ? 25 : 0;

    /* the pads first, as ATT, where the board has them: a negative gain, apart from the total.
       No attenuator setting besides */
    const bool pads = duo || dxr2 || rsp1b;
    g = d->listGains(SOAPY_SDR_RX, 0);
    if (g.size() != (amp ? 5u : 4u) + (pads ? 1u : 0u) || !has(g, "LNA") || !has(g, "MIX") || !has(g, "MIXBUF") ||
        !has(g, "BB") || (amp && !has(g, "AMP")) || (pads && g[0] != "ATT") || (!pads && has(g, "ATT")))
    { say("gain stages wrong"); bad++; }
    if (pads && (d->getGainRange(SOAPY_SDR_RX, 0, "ATT").minimum() != (dxr2 ? -36 : -19) ||
                 d->getGainRange(SOAPY_SDR_RX, 0, "ATT").maximum() != 0 ||
                 d->getGainRange(SOAPY_SDR_RX, 0, "ATT").step() != (dxr2 ? 3 : 1)))
    { say("ATT range %.0f to %.0f by %.0f", d->getGainRange(SOAPY_SDR_RX, 0, "ATT").minimum(),
          d->getGainRange(SOAPY_SDR_RX, 0, "ATT").maximum(), d->getGainRange(SOAPY_SDR_RX, 0, "ATT").step()); bad++; }
    if (chanOptions(d, 0, "attenuator") >= 0) { say("an attenuator channel setting is still there"); bad++; }
    for (const auto &a : d->getSettingInfo())
        if (a.key == "attenuator") { say("an attenuator device setting is still there"); bad++; }

    /* the most of any band, the same at VHF and HF: GQRX reads them once. The RSPduo's
       mixbuffer steps by 6 (its Hi-Z is on AM1), the RSPdx-R2's LNA reaches 37 (its direct
       sampling amplifier) */
    const double lnaMax = hw == "RSPdxR2" ? 37 : 24, mixbufStep = duo ? 6 : 24;
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    if (d->getGainRange(SOAPY_SDR_RX, 0).maximum() != 102 + ampMax ||
        d->getGainRange(SOAPY_SDR_RX, 0, "AMP").maximum() != ampMax ||
        d->getGainRange(SOAPY_SDR_RX, 0, "LNA").maximum() != lnaMax ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIX").maximum() != 19 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").maximum() != 24 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").step() != mixbufStep ||
        d->getGainRange(SOAPY_SDR_RX, 0, "BB").maximum() != 59)
    { say("VHF gain ranges wrong, total %.0f", d->getGainRange(SOAPY_SDR_RX, 0).maximum()); bad++; }

    /* VHF has no mixbuffer: asking for one leaves it at 0 */
    d->setGain(SOAPY_SDR_RX, 0, "MIXBUF", 24);
    if (d->getGain(SOAPY_SDR_RX, 0, "MIXBUF") != 0) { say("MIXBUF at VHF gave %.0f", d->getGain(SOAPY_SDR_RX, 0, "MIXBUF")); bad++; }

    /* HF goes through AM2, with a 24 dB mixbuffer, set with the ranges read at VHF */
    d->setFrequency(SOAPY_SDR_RX, 0, 10e6);
    if (d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").maximum() != 24 || d->getGainRange(SOAPY_SDR_RX, 0, "LNA").maximum() != lnaMax)
    { say("HF gain ranges wrong"); bad++; }
    d->setGain(SOAPY_SDR_RX, 0, "MIXBUF", 24);
    if (d->getGain(SOAPY_SDR_RX, 0, "MIXBUF") != 24) { say("MIXBUF 24 at HF gave %.0f", d->getGain(SOAPY_SDR_RX, 0, "MIXBUF")); bad++; }
    d->setGain(SOAPY_SDR_RX, 0, "MIXBUF", 0);

    /* a total, then one stage: the others stay */
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    d->setGain(SOAPY_SDR_RX, 0, 40);
    if (d->getGain(SOAPY_SDR_RX, 0) != 40) { say("total gain 40 gave %.0f", d->getGain(SOAPY_SDR_RX, 0)); bad++; }
    d->setGain(SOAPY_SDR_RX, 0, "BB", 10);
    d->setGain(SOAPY_SDR_RX, 0, "LNA", 24);
    d->setGain(SOAPY_SDR_RX, 0, "MIX", 19);
    if (d->getGain(SOAPY_SDR_RX, 0, "BB") != 10 || d->getGain(SOAPY_SDR_RX, 0, "LNA") != 24 ||
        d->getGain(SOAPY_SDR_RX, 0) != 53 + amp)
    { say("stages BB 10, LNA and MIX on gave %.0f dB", d->getGain(SOAPY_SDR_RX, 0)); bad++; }

    /* the amplifier off and on, the others staying */
    if (amp)
    {
        d->setGain(SOAPY_SDR_RX, 0, "AMP", 0);
        if (d->getGain(SOAPY_SDR_RX, 0, "AMP") != 0 || d->getGain(SOAPY_SDR_RX, 0) != 53)
        { say("AMP off gave %.0f dB", d->getGain(SOAPY_SDR_RX, 0)); bad++; }
        d->setGain(SOAPY_SDR_RX, 0, "AMP", amp);
        if (d->getGain(SOAPY_SDR_RX, 0, "AMP") != amp || d->getGain(SOAPY_SDR_RX, 0) != 53 + amp)
        { say("AMP on gave %.0f dB", d->getGain(SOAPY_SDR_RX, 0)); bad++; }
    }

    /* RSP1B: its pads one at a time, 0, 6, 13 or 19 dB, the nearest taken, apart from the total */
    if (rsp1b)
    {
        d->setGain(SOAPY_SDR_RX, 0, "ATT", -12);
        if (d->getGain(SOAPY_SDR_RX, 0, "ATT") != -13 || d->getGain(SOAPY_SDR_RX, 0) != 53 + amp)
        { say("ATT -12 gave %.0f", d->getGain(SOAPY_SDR_RX, 0, "ATT")); bad++; }
        d->setGain(SOAPY_SDR_RX, 0, "ATT", 0);
    }

    /* the notches: each channel its own, the device's channel 0's */
    if (rsp1b || dxr2 || duo)
    {
        const size_t last = duo ? 1 : 0;

        d->writeSetting("fm_notch", "true");
        if (chanOptions(d, 0, "fm_notch") < 0 || d->readSetting(SOAPY_SDR_RX, 0, "fm_notch") != "true" ||
            d->readSetting("fm_notch") != "true" || (duo && d->readSetting(SOAPY_SDR_RX, 1, "fm_notch") != "false"))
        { say("fm notch: device %s, channel 0 %s", d->readSetting("fm_notch").c_str(),
              d->readSetting(SOAPY_SDR_RX, 0, "fm_notch").c_str()); bad++; }
        d->writeSetting(SOAPY_SDR_RX, last, "dab_notch", "true");
        if (d->readSetting(SOAPY_SDR_RX, last, "dab_notch") != "true")
        { say("dab notch on channel %zu: %s", last, d->readSetting(SOAPY_SDR_RX, last, "dab_notch").c_str()); bad++; }
        d->writeSetting(SOAPY_SDR_RX, last, "dab_notch", "false");
        d->writeSetting("fm_notch", "false");
        if (d->readSetting("fm_notch") != "false") { say("fm notch left on"); bad++; }
    }

    /* RSPdx-R2: its pads in 3 dB steps, the nearest taken, apart from the total */
    if (dxr2)
    {
        d->setGain(SOAPY_SDR_RX, 0, "ATT", -20);
        if (d->getGain(SOAPY_SDR_RX, 0, "ATT") != -21 || d->getGain(SOAPY_SDR_RX, 0) != 53 + amp)
        { say("ATT -20 gave %.0f", d->getGain(SOAPY_SDR_RX, 0, "ATT")); bad++; }
        d->setGain(SOAPY_SDR_RX, 0, "ATT", 0);
    }

    /* RSPduo: the pads apart from the total, each channel its own, the nearest taken, and
       the port tie */
    if (duo)
    {
        d->setGain(SOAPY_SDR_RX, 1, "ATT", -12);
        d->setGain(SOAPY_SDR_RX, 0, "ATT", -6);
        if (d->getGain(SOAPY_SDR_RX, 1, "ATT") != -13 || d->getGain(SOAPY_SDR_RX, 0, "ATT") != -6 ||
            d->getGain(SOAPY_SDR_RX, 0) != 53 + amp || d->getGainRange(SOAPY_SDR_RX, 1, "ATT").minimum() != -19)
        { say("ATT: channel 0 %.0f, channel 1 %.0f", d->getGain(SOAPY_SDR_RX, 0, "ATT"),
              d->getGain(SOAPY_SDR_RX, 1, "ATT")); bad++; }
        d->setGain(SOAPY_SDR_RX, 1, "ATT", 0);
        d->setGain(SOAPY_SDR_RX, 0, "ATT", 0);

        d->writeSetting("port_tie", "true");
        if (d->readSetting("port_tie") != "true") { say("port tie: %s", d->readSetting("port_tie").c_str()); bad++; }
        d->writeSetting("port_tie", "false");
        if (d->readSetting("port_tie") != "false") { say("port tie left on"); bad++; }
        d->writeSetting("ref_out", "true");
        if (d->readSetting("ref_out") != "true" || d->readSetting("port_tie") != "false") { say("reference output: %s", d->readSetting("ref_out").c_str()); bad++; }
        d->writeSetting("ref_out", "false");
        if (d->readSetting("ref_out") != "false") { say("reference output left on"); bad++; }
    }

    /* the filters follow the IF; a request takes the narrowest that passes it */
    bw = d->listBandwidths(SOAPY_SDR_RX, 0);
    if (std::find(bw.begin(), bw.end(), 8e6) == bw.end()) { say("no 8 MHz filter at zero IF"); bad++; }
    d->setBandwidth(SOAPY_SDR_RX, 0, 5.5e6);
    if (d->getBandwidth(SOAPY_SDR_RX, 0) != 6e6) { say("5.5 MHz gave %.0f", d->getBandwidth(SOAPY_SDR_RX, 0)); bad++; }
    d->writeSetting("if_freq", "450000");
    bw = d->listBandwidths(SOAPY_SDR_RX, 0);
    if (bw.size() != 3 || bw[0] != 200e3 || bw[2] != 600e3) { say("low IF filters wrong"); bad++; }
    d->writeSetting("if_freq", "auto");

    /* past what an isochronous transfer carries, refused: logged, and the rate stays */
    std::string transfer = d->readSetting("transfer");
    d->writeSetting("transfer", "ISOC");
    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    d->setSampleRate(SOAPY_SDR_RX, 0, 100e6);
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 2e6) { say("a refused rate changed it"); bad++; }
    /* in bulk the receiver's most instead, 504 samples a packet past its 56 MB/s */
    d->writeSetting("transfer", "BULK");
    d->setSampleRate(SOAPY_SDR_RX, 0, 100e6);
    if (std::fabs(d->getSampleRate(SOAPY_SDR_RX, 0) - 27562500) > 16)
    { say("100 Msps in bulk: %.0f", d->getSampleRate(SOAPY_SDR_RX, 0)); bad++; }
    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    d->writeSetting("transfer", transfer);

    /* a low rate takes a low IF, the frequency still the centre */
    d->setFrequency(SOAPY_SDR_RX, 0, 433.5e6);
    d->setSampleRate(SOAPY_SDR_RX, 0, 900000);
    bw = d->listBandwidths(SOAPY_SDR_RX, 0);
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 900000 || d->getFrequency(SOAPY_SDR_RX, 0) != 433.5e6 || bw.back() != 600e3)
    { say("900 ksps: %.0f sps at %.0f Hz, widest filter %.0f", d->getSampleRate(SOAPY_SDR_RX, 0),
          d->getFrequency(SOAPY_SDR_RX, 0), bw.back()); bad++; }
    /* the DC removal: on at a low IF by default, set on or off at any IF */
    if (!d->hasDCOffsetMode(SOAPY_SDR_RX, 0) || !d->getDCOffsetMode(SOAPY_SDR_RX, 0)) { say("no DC removal at a low IF"); bad++; }
    d->setDCOffsetMode(SOAPY_SDR_RX, 0, false);
    if (d->getDCOffsetMode(SOAPY_SDR_RX, 0)) { say("DC removal stayed on"); bad++; }
    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    d->setDCOffsetMode(SOAPY_SDR_RX, 0, true);
    if (!d->getDCOffsetMode(SOAPY_SDR_RX, 0)) { say("no DC removal at zero IF when set"); bad++; }
    d->setSampleRate(SOAPY_SDR_RX, 0, 900000);

    /* auto never takes 2048 kHz: 1620 kHz has the same filters at a lower converter rate */
    for (double sr : { 1024000.0, 512000.0, 256000.0, 128000.0, 1000000.0, 1200000.0, 700000.0, 64000.0 })
    {
        d->setSampleRate(SOAPY_SDR_RX, 0, sr);
        if (d->readSetting("if_in_use") == "2048000") { say("auto took 2048 kHz at %.0f S/s", sr); bad++; }
    }
    d->setSampleRate(SOAPY_SDR_RX, 0, 900000);

    /* any other low rate as asked, the NCO taking the band the rest of the way */
    d->setSampleRate(SOAPY_SDR_RX, 0, 250000);
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 250000) { say("250 ksps gave %.0f", d->getSampleRate(SOAPY_SDR_RX, 0)); bad++; }

    /* a fixed IF has a span of rates; one converter halves its top */
    d->writeSetting("if_freq", "2048000");
    d->setSampleRate(SOAPY_SDR_RX, 0, 8192000);
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 8192000) { say("both converters at 2048 kHz: %.0f", d->getSampleRate(SOAPY_SDR_RX, 0)); bad++; }
    top = d->getSampleRateRange(SOAPY_SDR_RX, 0).back().maximum();
    d->writeSetting("converters", "I");
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != std::min(8192000.0, std::floor(top / 2)) ||
        d->getSampleRateRange(SOAPY_SDR_RX, 0).back().maximum() != std::floor(top / 2))
    { say("one converter at 2048 kHz: %.0f, the span's top %.0f of %.0f", d->getSampleRate(SOAPY_SDR_RX, 0),
          d->getSampleRateRange(SOAPY_SDR_RX, 0).back().maximum(), top); bad++; }

    /* raw: one converter gives real samples, the LO at the centre */
    d->writeSetting("baseband", "false");
    f = d->getStreamFormats(SOAPY_SDR_RX, 0);
    if (d->getNativeStreamFormat(SOAPY_SDR_RX, 0, full) != SOAPY_SDR_S16 || !has(f, SOAPY_SDR_F32) || has(f, SOAPY_SDR_CF32) ||
        d->getFrequency(SOAPY_SDR_RX, 0) != 433.5e6)
    { say("raw with one converter: native %s, LO %.0f", d->getNativeStreamFormat(SOAPY_SDR_RX, 0, full).c_str(),
          d->getFrequency(SOAPY_SDR_RX, 0)); bad++; }
    d->writeSetting("converters", "both");
    d->writeSetting("if_freq", "auto");
    if (d->getNativeStreamFormat(SOAPY_SDR_RX, 0, full) != SOAPY_SDR_CS16 || full != 32768) { say("raw native not CS16"); bad++; }
    /* gap fill reads back as it runs: as set when raw, on under baseband */
    d->writeSetting("gap_fill", "false");
    if (d->readSetting("gap_fill") != "false") { say("raw gap fill off reads %s", d->readSetting("gap_fill").c_str()); bad++; }
    d->writeSetting("baseband", "true");
    if (d->readSetting("gap_fill") != "true") { say("baseband gap fill reads %s", d->readSetting("gap_fill").c_str()); bad++; }
    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);

    if (!bad) say("ranges, gain stages, filters, a low IF below 1.3 Msps, fixed IFs, one converter, raw, gap fill");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* each format at 2 Msps, and 8 Msps over 8 bit USB samples: rate, time stamps, data */
static int stream(Dev *d, soapy_say_t say, soapy_say_t note)
{
    struct Case { const char *fmt; double rate; const char *usb; const char *conv; };
    const Case cases[] = { { SOAPY_SDR_CS16, 2e6, "AUTO", "both" }, { SOAPY_SDR_CF32, 2e6, "AUTO", "both" },
                           { SOAPY_SDR_CS8, 2e6, "AUTO", "both" }, { SOAPY_SDR_CS16, 8e6, "504_S8", "both" },
                           { SOAPY_SDR_CF32, 900000, "AUTO", "both" }, { SOAPY_SDR_CS16, 256000, "AUTO", "both" },
                           { SOAPY_SDR_CF32, 1024000, "AUTO", "I" } };
    int bad = 0;

    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    d->setGain(SOAPY_SDR_RX, 0, 40);

    for (const Case &c : cases)
    {
        d->writeSetting("format", c.usb);
        d->writeSetting("converters", c.conv);
        d->setSampleRate(SOAPY_SDR_RX, 0, c.rate);

        SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, c.fmt);
        size_t mtu = d->getStreamMTU(s);
        std::vector<int16_t> buf(4 * mtu);
        void *b = buf.data();
        long long total = 0, want = (long long) c.rate, next = -1, jumps = 0, ovf = 0, err = 0, notime = 0;
        double sum = 0, t0, dt;

        d->activateStream(s);
        /* the first buffers carry the start up */
        for (int i = 0; i < 20; i++) { int fl; long long t; d->readStream(s, &b, mtu, fl, t, 1000000); }
        t0 = now();
        while (total < want && !err)
        {
            int fl = 0; long long t = 0;
            int n = d->readStream(s, &b, mtu, fl, t, 1000000);

            if (n == SOAPY_SDR_OVERFLOW) { ovf++; next = -1; continue; }
            if (n < 0) { err = n; break; }
            if (!(fl & SOAPY_SDR_HAS_TIME)) notime++;
            if (next >= 0 && std::llabs(t - next) > 2) jumps++;
            next = t + (long long) std::llround(n * 1e9 / c.rate);

            /* the power, in the format's own scale, from I and Q alike (one alone can sit
               under 8 bits' LSB on a quiet input) */
            for (int i = 0; i < 2 * n; i += 63)
            {
                double v = c.fmt == std::string(SOAPY_SDR_CF32) ? ((float *) b)[i] :
                           c.fmt == std::string(SOAPY_SDR_CS8) ? ((int8_t *) b)[i] / 128.0 : ((int16_t *) b)[i] / 32768.0;
                sum += v * v;
            }
            total += n;
        }
        dt = now() - t0;
        d->deactivateStream(s);
        d->closeStream(s);

        double sps = total / dt, rms = std::sqrt(sum / (total / 32.0 + 1));
        note("%s at %.3f Msps over %s, %s converters: %.0f sps, rms %.4f, %lld overflows, %lld time jumps",
             c.fmt, c.rate / 1e6, c.usb, c.conv, sps, rms, ovf, jumps);
        if (err || ovf || jumps || notime || std::fabs(sps / c.rate - 1) > 0.05 || rms < 1e-4)
        {
            say("%s at %.0f Msps: %.0f sps, rms %.5f, %lld overflows, %lld jumps, error %lld",
                c.fmt, c.rate / 1e6, sps, rms, ovf, jumps, err);
            bad++;
        }
    }
    d->writeSetting("format", "AUTO");
    d->writeSetting("converters", "both");

    if (!bad) say("CS16, CF32, CS8 at 2 and 8 Msps, low IF at 900k, 256k and one converter at 1024k: full rate, "
                  "time stamps continuous");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* the RSPduo: channels 0 and 1 apart, then together, each buffer filled with as many
   samples and one time stamp, and tuner 1 alone too */
static int channels(Dev *d, soapy_say_t say, soapy_say_t note)
{
    int bad = 0;

    if (d->getNumChannels(SOAPY_SDR_RX) < 2) { say("one channel"); return SOAPY_T_SKIP; }

    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    d->setFrequency(SOAPY_SDR_RX, 1, 433e6);
    if (d->getFrequency(SOAPY_SDR_RX, 0) != 100e6 || d->getFrequency(SOAPY_SDR_RX, 1) != 433e6)
    { say("frequencies %.0f and %.0f", d->getFrequency(SOAPY_SDR_RX, 0), d->getFrequency(SOAPY_SDR_RX, 1)); bad++; }
    d->setGain(SOAPY_SDR_RX, 1, 30);
    d->setSampleRate(SOAPY_SDR_RX, 0, 250000);

    for (int mode = 0; mode < 2; mode++)
    {
        std::vector<size_t> ch = mode ? std::vector<size_t>{ 0, 1 } : std::vector<size_t>{ 1 };
        SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, ch);
        size_t mtu = d->getStreamMTU(s);
        std::vector<float> b0(2 * mtu), b1(2 * mtu);
        void *bp[2] = { b0.data(), b1.data() };
        long long total = 0, next = -1, jumps = 0, err = 0;
        double sum[2] = { 0, 0 }, t0, rate = d->getSampleRate(SOAPY_SDR_RX, 0);

        d->activateStream(s);
        for (int i = 0; i < 20; i++) { int fl; long long t; d->readStream(s, bp, mtu, fl, t, 1000000); }
        t0 = now();
        while (total < (long long) rate && !err)
        {
            int fl = 0; long long t = 0;
            int n = d->readStream(s, bp, mtu, fl, t, 1000000);

            if (n == SOAPY_SDR_OVERFLOW) { next = -1; continue; }
            if (n < 0) { err = n; break; }
            if (next >= 0 && std::llabs(t - next) > 2) jumps++;
            next = t + (long long) std::llround(n * 1e9 / rate);
            for (int k = 0; k < 2 * n; k += 16) { sum[0] += b0[k] * b0[k]; if (mode) sum[1] += b1[k] * b1[k]; }
            total += n;
        }
        double sps = total / (now() - t0);
        d->deactivateStream(s);
        d->closeStream(s);

        note("%s: %.0f sps, rms %.5f%s%.5f, %lld time jumps, error %lld", mode ? "channels 0 and 1" : "channel 1 alone",
             sps, std::sqrt(sum[0] / (total / 8.0 + 1)), mode ? " and " : "", mode ? std::sqrt(sum[1] / (total / 8.0 + 1)) : 0.0,
             jumps, err);
        if (err || jumps || std::fabs(sps / rate - 1) > 0.05 || sum[0] <= 0 || (mode && sum[1] <= 0))
        { say("%s: %.0f sps, %lld jumps, error %lld", mode ? "two channels" : "channel 1", sps, jumps, err); bad++; }
    }

    /* back to channel 0 alone */
    SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, { 0 });
    d->closeStream(s);

    if (!bad) say("frequencies apart, channel 1 alone, both together at the rate with one time stamp");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* start and stop straight away, half of them after one read */
static int cycles(Dev *d, soapy_say_t say, soapy_say_t note)
{
    const int n = 100;
    int slow = 0, i;
    double worst = 0;

    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
    std::vector<int16_t> buf(2 * d->getStreamMTU(s));
    void *b = buf.data();

    for (i = 0; i < n; i++)
    {
        double t0 = now();
        d->activateStream(s);
        if (i % 2) { int fl; long long t; d->readStream(s, &b, buf.size() / 2, fl, t, 1000000); }
        d->deactivateStream(s);
        worst = std::max(worst, now() - t0);
        if (now() - t0 > 2) { slow++; note("cycle %d took %.2f s", i, now() - t0); }
    }
    d->closeStream(s);

    say("%d start and stop cycles, %d slow, the longest %.0f ms", n, slow, worst * 1e3);
    return slow ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* retunes while streaming: no overflow, time stamps continuous; then rate changes
 * between zero IF and low IF ones: the stream restarts and flows at the last rate */
static int retune(Dev *d, soapy_say_t say, soapy_say_t note)
{
    double t0, last = 0, rate;
    long long total = 0, ovf = 0, err = 0, jumps = 0, next = -1, tunes = 0, ticks = 0, rates = 0;
    int bad = 0;

    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    std::vector<float> buf(2 * d->getStreamMTU(s));
    void *b = buf.data();

    d->activateStream(s);
    for (int phase = 0; phase < 2; phase++)
    {
        t0 = now();
        while (now() - t0 < 4)
        {
            int fl = 0; long long t = 0;
            int n = d->readStream(s, &b, buf.size() / 2, fl, t, 1000000);

            if (n == SOAPY_SDR_OVERFLOW) { ovf += !phase; next = -1; continue; }
            if (n < 0) { err++; continue; }
            if (!phase && next >= 0 && std::llabs(t - next) > 2) jumps++;
            rate = d->getSampleRate(SOAPY_SDR_RX, 0);
            next = t + (long long) std::llround(n * 1e9 / rate);
            total += n;

            if (now() - last > 0.05)
            {
                last = now();
                if (!phase) { d->setFrequency(SOAPY_SDR_RX, 0, 90e6 + (tunes % 20) * 1e6); tunes++; }
                else if ((ticks++ % 4) == 0)
                {
                    /* through zero IF and low IF rates, which switch the IF while streaming */
                    static const double cyc[] = { 8e6, 900000, 2e6, 256000 };
                    d->setSampleRate(SOAPY_SDR_RX, 0, cyc[rates % 4]);
                    rates++;
                    next = -1;
                }
            }
        }
    }

    /* and it streams at the rate set last */
    long long got = 0;
    t0 = now();
    while (now() - t0 < 0.5)
    {
        int fl; long long t;
        int n = d->readStream(s, &b, buf.size() / 2, fl, t, 1000000);
        if (n > 0) got += n;
    }
    rate = got / (now() - t0);
    d->deactivateStream(s);
    d->closeStream(s);

    note("%lld retunes: %lld overflows, %lld time jumps; %lld rate changes, then %.0f sps at %.0f",
         tunes, ovf, jumps, rates, rate, d->getSampleRate(SOAPY_SDR_RX, 0));
    if (ovf || jumps || err || std::fabs(rate / d->getSampleRate(SOAPY_SDR_RX, 0) - 1) > 0.1) bad++;

    say("%lld retunes: %lld overflows, %lld time jumps; %lld rate changes, then %.0f sps; %lld errors",
        tunes, ovf, jumps, rates, rate, err);
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* gain_ranges=band: only what the band tuned has */
static int bandRanges(Dev *d, soapy_say_t say, soapy_say_t note)
{
    int bad = 0;

    /* VHF: LNA 24, mixer 19, no mixbuffer, baseband 59, and the amplifier, the RSPduo's and RSP1B's
       25, the RSPdx-R2's 24 */
    const double amp = (d->getHardwareKey() == "RSPduo" || d->getHardwareKey() == "RSP1B") ? 25 :
                       d->getHardwareKey() == "RSPdxR2" ? 24 : 0;
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    if (d->getGainRange(SOAPY_SDR_RX, 0).maximum() != 102 + amp ||
        d->getGainRange(SOAPY_SDR_RX, 0, "AMP").maximum() != amp ||
        d->getGainRange(SOAPY_SDR_RX, 0, "LNA").maximum() != 24 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIX").maximum() != 19 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").maximum() != 0 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "BB").maximum() != 59)
    { say("VHF gain ranges wrong, total %.0f", d->getGainRange(SOAPY_SDR_RX, 0).maximum()); bad++; }

    /* HF goes through AM2: a 24 dB mixbuffer and no LNA gain */
    d->setFrequency(SOAPY_SDR_RX, 0, 10e6);
    if (d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").maximum() != 24 || d->getGainRange(SOAPY_SDR_RX, 0, "LNA").maximum() != 0 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "AMP").maximum() != 0)
    { say("HF gain ranges wrong"); bad++; }

    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* reads for a while, returning the samples and the time stamps' end; err counts errors, and
   jumps time stamps more than 10 ms back or 100 ms on from where the last read ended (a
   restart takes a few ms; a buffer of the old rate read at the new is 3 ms out at most) */
static long long readFor(Dev *d, SoapySDR::Stream *s, std::vector<float> &buf, double secs, long long &end,
                         long long &err, long long *jumps = nullptr)
{
    void *b = buf.data();
    long long got = 0;
    double t0 = now();

    while (now() - t0 < secs)
    {
        int fl = 0; long long t = 0;
        int n = d->readStream(s, &b, buf.size() / 2, fl, t, 1000000);

        if (n == SOAPY_SDR_OVERFLOW) continue;
        if (n < 0) { err++; continue; }
        if (jumps && end && (t < end - 10000000 || t > end + 100000000)) (*jumps)++;
        end = t + (long long) std::llround(n * 1e9 / d->getSampleRate(SOAPY_SDR_RX, 0));
        got += n;
    }
    return got;
}

/* what the review of 2026-10 found, each once */
static int regress(Dev *d, const char *module, const char *serial, soapy_say_t say, soapy_say_t note)
{
    int bad = 0;

    /* an IF given in any other way than the module lists: refused, and nothing throws later */
    for (const char *v : { "AUTO", "", "450k", "Zero IF" })
    {
        try {
            d->writeSetting("if_freq", v);
            d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
            if (d->listSampleRates(SOAPY_SDR_RX, 0).empty() || d->getSampleRateRange(SOAPY_SDR_RX, 0).empty())
            { say("if_freq \"%s\": no rates left", v); bad++; }
        } catch (const std::exception &e) { say("if_freq \"%s\" threw: %s", v, e.what()); bad++; }
        if (d->readSetting("if_freq") != "auto") { say("if_freq \"%s\" left %s", v, d->readSetting("if_freq").c_str()); bad++; }
    }
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 2e6) { say("2 Msps after the IF settings: %.0f", d->getSampleRate(SOAPY_SDR_RX, 0)); bad++; }

    /* a refused decimation bypass is not kept */
    d->writeSetting("decimation_bypass", "ON");
    if (d->readSetting("decimation_bypass") != "AUTO") { say("refused bypass kept: %s", d->readSetting("decimation_bypass").c_str()); bad++; }
    d->setSampleRate(SOAPY_SDR_RX, 0, 2.048e6);
    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 2e6) { say("2 Msps after a refused bypass: %.0f", d->getSampleRate(SOAPY_SDR_RX, 0)); bad++; }

    /* no phantom mixbuffer outside the AM bands */
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    d->setGain(SOAPY_SDR_RX, 0, 90);
    if (d->getGain(SOAPY_SDR_RX, 0, "MIXBUF") != 0) { say("MIXBUF %.0f dB at VHF", d->getGain(SOAPY_SDR_RX, 0, "MIXBUF")); bad++; }

    SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    std::vector<float> buf(2 * d->getStreamMTU(s));
    void *b = buf.data();
    long long end = 0, err = 0, jumps = 0;

    /* the time goes on across a rate change, at each part's rate */
    d->setSampleRate(SOAPY_SDR_RX, 0, 4.096e6);
    d->activateStream(s);
    readFor(d, s, buf, 0.3, end, err);
    d->setSampleRate(SOAPY_SDR_RX, 0, 2.048e6);
    readFor(d, s, buf, 0.3, end, err, &jumps);
    d->setSampleRate(SOAPY_SDR_RX, 0, 4.096e6);
    readFor(d, s, buf, 0.3, end, err, &jumps);
    d->setSampleRate(SOAPY_SDR_RX, 0, 2.048e6);
    readFor(d, s, buf, 0.3, end, err, &jumps);
    /* rescaled at the new rate, the time so far would double or halve */
    if (jumps) { say("%lld time jumps over rate changes, ending at %.3f s", jumps, end / 1e9); bad++; }

    /* the transfer changed while streaming: it flows on */
    for (const char *t : { "BULK", "ISOC1", "ISOC" })
    {
        long long e = 0, got;

        d->writeSetting("transfer", t);
        got = readFor(d, s, buf, 0.5, end, e);
        note("%s while streaming: %.0f sps", t, got / 0.5);
        if (d->readSetting("transfer") != t || e || got < 0.8 * 0.5 * 2.048e6)
        { say("transfer %s while streaming: %s, %.0f sps, %lld errors", t, d->readSetting("transfer").c_str(), got / 0.5, e); bad++; }
    }

    /* stopped: a read waits its timeout rather than spinning */
    d->deactivateStream(s);
    {
        int fl; long long t;
        double t0 = now();
        int n, k = 0;

        while ((n = d->readStream(s, &b, buf.size() / 2, fl, t, 100000)) > 0 && k++ < 1000);
        t0 = now();
        n = d->readStream(s, &b, buf.size() / 2, fl, t, 100000);
        if (n != SOAPY_SDR_TIMEOUT || now() - t0 < 0.08) { say("read when stopped: %d after %.0f ms", n, (now() - t0) * 1e3); bad++; }
    }

    /* started again: the time carries on from where it stopped, it does not go back to 0 */
    {
        int fl = 0, n = 0, k = 0;
        long long t = 0, before = d->getHardwareTime();

        d->activateStream(s);
        while ((n = d->readStream(s, &b, buf.size() / 2, fl, t, 200000)) <= 0 && k++ < 20);
        if (n <= 0 || t < before || d->getHardwareTime() < before)
        { say("started again: %d, the time %.3f s after %.3f s", n, t / 1e9, before / 1e9); bad++; }
        d->deactivateStream(s);
    }

    /* after closing, reads and starts are refused */
    d->closeStream(s);
    {
        int fl; long long t;
        int n = d->readStream(s, &b, buf.size() / 2, fl, t, 1000);

        if (n >= 0 || n == SOAPY_SDR_TIMEOUT) { say("read after close: %d", n); bad++; }
        if (d->activateStream(s) == 0) { say("start after close accepted"); bad++; d->deactivateStream(s); }
    }

    if (err) { say("%lld read errors", err); bad++; }
    if (!bad) say("IF strings, a refused bypass, MIXBUF at VHF, time over a rate change and a restart, "
                  "transfers while streaming, reads when stopped and closed");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* report=true: the module's lines on what the receiver runs, kept here */
static std::vector<std::string> logged;

static void keepLog(const SoapySDRLogLevel, const char *m)
{
    for (const char *k : { "mirisdr: stream:", "mirisdr: converters:", "mirisdr: baseband:", "mirisdr: tuner " })
        if (!strncmp(m, k, strlen(k))) { logged.push_back(m + strlen("mirisdr: ")); return; }
}

/* a line logged that starts with prefix and has part in it */
static bool logLine(const char *prefix, const char *part = "")
{
    for (const std::string &l : logged)
        if (!l.compare(0, strlen(prefix), prefix) && l.find(part) != std::string::npos) return true;
    return false;
}

/* each line once at open, then only the ones a call changed: a rate below the converters'
   least takes a low IF, said on the stream and the tuner's lines, the same rate again logs
   nothing, a retune the tuner's line and not the stream's, a gain its own line alone.
   Without report=true nothing */
static int report(const char *module, const char *serial, soapy_say_t say, soapy_say_t note)
{
    SoapySDR::Kwargs extra;
    Dev *d;
    int bad = 0;
    char want[64];

    extra["report"] = "true";
    logged.clear();
    SoapySDR::registerLogHandler(keepLog);
    if (!(d = openDev(module, serial, nullptr, say, extra))) return SOAPY_T_FAIL;

    try
    {
        for (const std::string &l : logged) note("open: %s", l.c_str());
        if (!logLine("stream: 2048000 S/s, complex float at zero IF") || !logLine("converters: ") ||
            !logLine("baseband: decimation") || !logLine("tuner 0: 100.000000 MHz", "zero IF") || !logLine("tuner 0 gain: "))
        { say("at open %zu lines, not each part", logged.size()); bad++; }

        logged.clear();
        d->setSampleRate(SOAPY_SDR_RX, 0, 1e6);
        for (const std::string &l : logged) note("1 Msps: %s", l.c_str());
        snprintf(want, sizeof want, "stream: %.0f S/s, complex float from a low IF", d->getSampleRate(SOAPY_SDR_RX, 0));
        if (!logLine(want) || !logLine("tuner 0: ", "low IF") || !logLine("baseband: decimation"))
        { say("1 Msps: no low IF in the %zu lines", logged.size()); bad++; }

        logged.clear();
        d->setSampleRate(SOAPY_SDR_RX, 0, 1e6);
        if (!logged.empty()) { say("the same rate again logged %s", logged[0].c_str()); bad++; }

        logged.clear();
        d->setFrequency(SOAPY_SDR_RX, 0, 101e6);
        for (const std::string &l : logged) note("101 MHz: %s", l.c_str());
        if (!logLine("tuner 0: 101.000000 MHz") || logLine("stream:"))
        { say("a retune logged %zu lines, not the tuner's alone", logged.size()); bad++; }

        logged.clear();
        d->setGain(SOAPY_SDR_RX, 0, 30);
        for (const std::string &l : logged) note("30 dB: %s", l.c_str());
        if (logged.size() != 1 || !logLine("tuner 0 gain: 30 dB"))
        { say("a gain logged %zu lines, not its own alone", logged.size()); bad++; }

        /* the pads, a gain apart from the total: the front end's line alone */
        if (d->getGainRange(SOAPY_SDR_RX, 0, "ATT").minimum() < 0)
        {
            logged.clear();
            d->setGain(SOAPY_SDR_RX, 0, "ATT", -6);
            for (const std::string &l : logged) note("ATT -6: %s", l.c_str());
            if (logged.size() != 1 || !logLine("tuner 0 front end: ", "pads 6 dB"))
            { say("ATT logged %zu lines, not the front end's alone", logged.size()); bad++; }
            d->setGain(SOAPY_SDR_RX, 0, "ATT", 0);
        }

        logged.clear();
        d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
        if (!logLine("stream: 2000000 S/s, complex float at zero IF") || !logLine("tuner 0: ", "zero IF"))
        { say("2 Msps: not back at zero IF in the %zu lines", logged.size()); bad++; }

        /* the RSPdx-R2 under 2 MHz: direct sampling, which always runs the NCO */
        if (d->getHardwareKey() == "RSPdxR2")
        {
            d->setSampleRate(SOAPY_SDR_RX, 0, 1e6);
            logged.clear();
            d->setFrequency(SOAPY_SDR_RX, 0, 1e6);
            for (const std::string &l : logged) note("1 MHz: %s", l.c_str());
            if (!logLine("tuner 0: 1.000000 MHz by direct sampling") || !logLine("baseband: ", "NCO in use"))
            { say("1 MHz on the RSPdx-R2: no direct sampling in the %zu lines", logged.size()); bad++; }
        }
    }
    catch (const std::exception &e) { say("%s", e.what()); bad++; }
    Dev::unmake(d);

    logged.clear();
    if (!(d = openDev(module, serial, nullptr, say))) return SOAPY_T_FAIL;
    d->setSampleRate(SOAPY_SDR_RX, 0, 1e6);
    Dev::unmake(d);
    if (!logged.empty()) { say("without report=true: %s", logged[0].c_str()); bad++; }

    if (!bad) say("each part at open, then what a rate, a retune and a gain changed, and nothing without it");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

extern "C" int soapy_test(const char *which, const char *module, const char *serial, int verbose,
                          soapy_say_t say, soapy_say_t note)
{
    Dev *d;
    int r = SOAPY_T_FAIL;

    if (!verbose) SoapySDR::registerLogHandler(quietLog);

    /* the device string's IF: one not listed is refused at open, not thrown */
    if (!strcmp(which, "regress"))
    {
        SoapySDR::Kwargs extra;

        extra["if_freq"] = "AUTO";
        if (!(d = openDev(module, serial, nullptr, say, extra))) return SOAPY_T_FAIL;
        Dev::unmake(d);
        extra["if_freq"] = "bogus";
        if (!(d = openDev(module, serial, nullptr, say, extra))) return SOAPY_T_FAIL;
        Dev::unmake(d);

        /* every receiver listed with its port, and the serial only of one that declares it */
        SoapySDR::Kwargs find, by;
        std::string port, own;
        bool listed = false;

        find["driver"] = "mirisdr";
        auto found = Dev::enumerate(find);
        for (auto &f : found)
            if (f["usb_port"].find(':') == std::string::npos || (f.count("serial") && f["serial"].empty()))
            { say("listed usb_port \"%s\", serial \"%s\"", f["usb_port"].c_str(), f["serial"].c_str()); return SOAPY_T_FAIL; }
        if (!(d = openDev(module, serial, nullptr, say))) return SOAPY_T_FAIL;
        port = d->getHardwareInfo()["usb_port"];
        own = d->getHardwareInfo()["serial"];
        Dev::unmake(d);
        for (auto &f : found) listed |= (f["usb_port"] == port) && (f["serial"] == own);
        if (!listed) { say("opened usb_port \"%s\" serial \"%s\", not as listed", port.c_str(), own.c_str()); return SOAPY_T_FAIL; }

        /* opened by its port, alone and with its serial */
        by["usb_port"] = port;
        for (int k = 0; k < 2; k++)
        {
            if (!(d = openDev(module, k ? own.c_str() : nullptr, nullptr, say, by))) return SOAPY_T_FAIL;
            std::string got = d->getHardwareInfo()["usb_port"];
            Dev::unmake(d);
            if (got != port) { say("usb_port=%s opened %s", port.c_str(), got.c_str()); return SOAPY_T_FAIL; }
        }

        /* found by port, the result carries it, which the open matches again; a port whose
           receiver declares another serial than asked is not it */
        find["usb_port"] = port;
        found = Dev::enumerate(find);
        if (found.size() != 1 || found[0]["usb_port"] != port || found[0].count("serial"))
        { say("finding usb_port %s gave %zu results", port.c_str(), found.size()); return SOAPY_T_FAIL; }
        find["serial"] = "no such serial";
        if (!own.empty() && !Dev::enumerate(find).empty())
        { say("usb_port %s found for another serial", port.c_str()); return SOAPY_T_FAIL; }
    }

    if (!strcmp(which, "report"))
    {
        r = report(module, serial, say, note);
        SoapySDR::registerLogHandler(nullptr);
        return r;
    }

    if (!(d = openDev(module, serial, strcmp(which, "bandranges") ? nullptr : "band", say))) return SOAPY_T_FAIL;

    try
    {
        if (!strcmp(which, "probe")) r = probe(d, say, note);
        else if (!strcmp(which, "stream")) r = stream(d, say, note);
        else if (!strcmp(which, "channels")) r = channels(d, say, note);
        else if (!strcmp(which, "cycles")) r = cycles(d, say, note);
        else if (!strcmp(which, "retune")) r = retune(d, say, note);
        else if (!strcmp(which, "bandranges")) r = bandRanges(d, say, note);
        else if (!strcmp(which, "regress")) r = regress(d, module, serial, say, note);
    }
    catch (const std::exception &e) { say("%s", e.what()); r = SOAPY_T_FAIL; }

    Dev::unmake(d);
    SoapySDR::registerLogHandler(nullptr);
    return r;
}
