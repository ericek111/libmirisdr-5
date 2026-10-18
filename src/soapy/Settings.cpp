/*
 * SoapySDR module for libmirisdr: device, tuning and settings.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "SoapyMiriSDR.hpp"

#include <SoapySDR/Logger.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

static const char *FORMATS[] = { "AUTO", "252_S16", "336_S16", "384_S16", "504_S16", "504_S8" };
static const char *TRANSFERS[] = { "BULK", "ISOC", "ISOC1", "ISOC2" };
static const char *BYPASS[] = { "AUTO", "ON", "OFF" };
static const uint32_t IFS[] = { 0, 450000, 1620000, 2048000 };
static const char *CONVERTERS[] = { "both", "I", "Q" };

/* every filter the tuner has; which ones an IF takes is asked from the library */
static const uint32_t BANDWIDTHS[] = { 200000, 300000, 600000, 1536000, 5000000, 6000000,
                                       7000000, 8000000, 14000000 };

static bool isTrue(const std::string &v)
{
    return v == "true" || v == "1" || v == "on" || v == "yes";
}

/* A direct sampling setting: off, on or auto (true and false as on and off), -1 otherwise */
static int parseDs(const std::string &v)
{
    if (v == "auto" || v == "AUTO") return MIRISDR_DS_AUTO;
    if (v == "ON" || isTrue(v)) return MIRISDR_DS_ON;
    if (v == "off" || v == "OFF" || v == "false" || v == "0" || v == "no") return MIRISDR_DS_OFF;
    return -1;
}

/* An IF setting as "auto" or the Hz of one the tuner has, anything else refused */
static bool parseIf(const std::string &v, std::string &mode)
{
    std::string l = v;
    char *end;
    unsigned long hz;

    std::transform(l.begin(), l.end(), l.begin(), [](unsigned char c) { return (char) std::tolower(c); });
    if (l == "auto")
    {
        mode = "auto";
        return true;
    }

    if (v.empty() || !std::isdigit((unsigned char) v[0])) return false;
    hz = std::strtoul(v.c_str(), &end, 10);
    if (*end || std::find(std::begin(IFS), std::end(IFS), (uint32_t) hz) == std::end(IFS)) return false;

    mode = std::to_string(hz);
    return true;
}

/* the IF asked for, 0 for automatic (or zero IF) */
uint32_t SoapyMiriSDR::fixedIf(void) const
{
    return ifMode == "auto" ? 0 : (uint32_t) std::strtoul(ifMode.c_str(), NULL, 10);
}

SoapyMiriSDR::SoapyMiriSDR(const SoapySDR::Kwargs &args):
    dev(nullptr), flavour(MIRISDR_HW_DEFAULT), ifFreq(0), ifMode("auto"), converters(MIRISDR_IQ_BOTH),
    baseband(true), wantRate(2048000), wantBw(0), argBw(0), bandGainRanges(false),
    reportOn(false), reportDepth(0), freqMin(0), freqMax(0), rateMin(0), rateMax(0),
    stopping(false), rxDone(true), rxEnded(false), rxResult(0), outFormat(0),
    ringHead(0), ringTail(0), ringCount(0), dropPending(false), pairOpen(false), slotPos(0), gapPos(0),
    timeOffsetNs(0), lastIndex(0), lastRate(0), timeBaseNs(0), timeBaseIndex(0), timeBaseRate(0),
    asyncBuffers(0), chanOrder{ 0, -1 }, tunerSel(0)
{
    mirisdr_open_config_t cfg;
    mirisdr_tune_config_t tc;
    char ser[256];

    /* the library finds the serial or port in the list it opens from */
    mirisdr_open_config_default(&cfg);
    if (args.count("serial")) cfg.serial = args.at("serial").c_str();
    if (args.count("usb_port")) cfg.usb_port = args.at("usb_port").c_str();

    if (mirisdr_open_ex(&dev, &cfg) < 0 || !dev)
        throw std::runtime_error("mirisdr: could not open the device");

    /* the device is ours from here: closed again if anything below throws */
    try {

    flavour = mirisdr_get_hw_flavour(dev);
    /* a tuner the board has not: no part of a front end, and no Hi-Z */
    std::memset(caps, 0, sizeof caps);
    caps[0].antenna_hiz = caps[1].antenna_hiz = -1;
    for (int t = 0; t < mirisdr_get_tuner_count(dev) && t < 2; t++) mirisdr_get_frontend_caps(dev, t, &caps[t]);
    if (mirisdr_get_serial(dev, ser, sizeof ser) > 0) serial = ser;
    /* "1-3" here is "1:3" in the list */
    if ((mirisdr_get_usb_position(dev, NULL, NULL, ser, sizeof ser) == 0) && std::strchr(ser, '-'))
    {
        *std::strchr(ser, '-') = ':';
        usbPort = ser;
    }

    /* the IF settings may come with the device, for programs that only take a string */
    if (args.count("if_freq") && !parseIf(args.at("if_freq"), ifMode))
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no IF %s, using auto", args.at("if_freq").c_str());
    if (args.count("converters"))
        converters = args.at("converters") == "I" ? MIRISDR_IQ_ONLY_I :
                     args.at("converters") == "Q" ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_BOTH;
    if (args.count("baseband")) baseband = isTrue(args.at("baseband"));
    /* the filter, for programs that have no field for it */
    if (args.count("bandwidth"))
    {
        try { argBw = (uint32_t) std::stoul(args.at("bandwidth")); }
        catch (const std::exception &) { SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: bandwidth %s is not a number of Hz", args.at("bandwidth").c_str()); }
        wantBw = argBw;
    }
    /* the gain ranges: "max", the default, stay the same in every band, as programs
       such as GQRX read them once at open; "band" has only what the band tuned has */
    if (args.count("gain_ranges"))
    {
        if (args.at("gain_ranges") == "band") bandGainRanges = true;
        else if (args.at("gain_ranges") != "max")
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: gain_ranges %s is not max or band", args.at("gain_ranges").c_str());
    }
    if (args.count("report")) reportOn = isTrue(args.at("report"));

    /* the library's defaults, its filter and a known gain, the ranges at zero IF */
    mirisdr_get_stream(dev, &streamCfg, NULL);
    streamCfg.rate = 2048000;
    streamCfg.baseband = 0;
    applyStream(streamCfg);

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.frequency = 100000000;
    tc.if_freq = 0;
    tc.bandwidth = 0;
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = 40;
    /* the RSPdx-R2 samples the antenna directly where the band fits under its 2 MHz low
       pass, unless the device string says otherwise */
    if (caps[0].direct_sampling)
    {
        int ds = args.count("direct_sampling") ? parseDs(args.at("direct_sampling")) : MIRISDR_DS_AUTO;
        if (ds < 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: direct_sampling %s is not off, on or auto", args.at("direct_sampling").c_str());
            ds = MIRISDR_DS_AUTO;
        }
        tc.frontend.direct_sampling = ds;
    }
    applyTune(tc);

    probeRanges();
    configure(wantRate);

    } catch (...) {
        mirisdr_close(dev);
        dev = nullptr;
        throw;
    }

    SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: opened %s at %s", serial.empty() ? "a receiver without serial" : serial.c_str(),
                  usbPort.empty() ? "an unknown port" : usbPort.c_str());
    if (reportOn) report();
}

SoapyMiriSDR::~SoapyMiriSDR(void)
{
    if (rxThread.joinable()) deactivateStream(nullptr, 0, 0);
    mirisdr_close(dev);
}

/*******************************************************************
 * Applying the config: refused combinations are logged, not thrown,
 * since applications rarely catch, and the device keeps what it had
 ******************************************************************/

/* strict: a rate out of reach refused rather than taken to the nearest, so a setting
   that would move the rate is refused instead */
bool SoapyMiriSDR::applyStream(const mirisdr_stream_config_t &cfg, bool strict)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_config_t c = cfg;

    c.format = formatSetting.empty() ? NULL : formatSetting.c_str();
    c.transfer = transferSetting.empty() ? NULL : transferSetting.c_str();
    c.decimation_bypass = bypassSetting.empty() ? NULL : bypassSetting.c_str();
    /* raw, a single converter gives real samples */
    c.follow_tune = !c.baseband;
    c.strict = strict ? 1 : 0;

    if (mirisdr_set_stream(dev, &c, NULL) < 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: stream refused (%u S/s, format %s, transfer %s)",
                      c.rate, c.format ? c.format : "AUTO", c.transfer ? c.transfer : "default");
        return false;
    }
    streamCfg = c;
    streamCfg.format = streamCfg.transfer = streamCfg.decimation_bypass = NULL;
    streamCfg.strict = 0;
    return true;
}

void SoapyMiriSDR::applyTune(const mirisdr_tune_config_t &cfg)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    if (mirisdr_tune(dev, tunerSel, &cfg, NULL) < 0)
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: tune refused (%u Hz, IF %u, bandwidth %u)",
                      cfg.frequency, cfg.if_freq, cfg.bandwidth);
}

/* what the library accepts, found once by asking it */
void SoapyMiriSDR::probeRanges(void)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t sc;
    double lo, hi, mid;
    int i;

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.gain.mode = MIRISDR_GAIN_KEEP;

    for (freqMin = 1000; freqMin < 100e6; freqMin *= 2)
    {
        tc.frequency = (uint32_t) freqMin;
        if (mirisdr_tune_check(dev, tunerSel, &tc, NULL) == 0) break;
    }
    lo = 1e9; hi = 4e9;
    for (i = 0; i < 40; i++)
    {
        mid = std::floor((lo + hi) / 2);
        tc.frequency = (uint32_t) mid;
        if (mirisdr_tune_check(dev, tunerSel, &tc, NULL) == 0) lo = mid; else hi = mid;
    }
    freqMax = lo;

    mirisdr_get_stream(dev, &sc, NULL);
    sc.baseband = 0;
    sc.follow_tune = 0;
    sc.strict = 1;      /* refused out of reach, not snapped */
    lo = 1; hi = 4e6;
    for (i = 0; i < 40; i++)
    {
        mid = std::floor((lo + hi) / 2);
        sc.rate = (uint32_t) mid;
        if (mirisdr_stream_check(dev, &sc, NULL) == 0) hi = mid; else lo = mid;
    }
    rateMin = hi;
    lo = 4e6; hi = 100e6;
    for (i = 0; i < 40; i++)
    {
        mid = std::floor((lo + hi) / 2);
        sc.rate = (uint32_t) mid;
        if (mirisdr_stream_check(dev, &sc, NULL) == 0) lo = mid; else hi = mid;
    }
    rateMax = lo;
}

/*******************************************************************
 * The IF and the rate. Baseband (the default) delivers the band at 0 Hz
 * whatever the IF. "auto" keeps zero IF from rateMin up, as without
 * baseband, and below it takes a low IF. A low IF takes any rate in a
 * span (see the library's baseband rates): the converters at 4 x IF up
 * to 7 Msps, else at the least the IF takes, and a resampler for what
 * halving does not reach. The cheapest are 4 x IF divided down, which
 * need neither the resampler nor the NCO
 ******************************************************************/

/* the rates of a low IF that need no resampler or NCO: 4 x IF divided by a power of 2, by
   2 at least for a single converter. None where 4 x IF is past 7 Msps (2048 kHz): the
   library runs those converters lower, with the NCO */
std::vector<uint32_t> SoapyMiriSDR::lowIfRates(uint32_t ifHz) const
{
    std::vector<uint32_t> rates;
    int one = oneOut(), k;
    uint64_t base = (uint64_t) 4 * ifHz / (one ? 2 : 1);

    if ((uint64_t) 4 * ifHz > 7000000) return rates;

    for (k = 0; k <= 8 - one && !(base % ((uint64_t) 1 << k)); k++) rates.push_back((uint32_t) (base >> k));
    return rates;
}

/* the converter rate a low IF runs at, as the library has it: 4 x IF up to 7 Msps, else
   the least the IF takes, 2.4 x (IF + half its widest filter) */
static uint64_t convPref(uint32_t ifHz)
{
    const uint64_t widest = ifHz > 450000 ? 1536000 : 600000;
    const uint64_t least = std::max<uint64_t>(((2 * (uint64_t) ifHz + widest) * 6 + 4) / 5, 1300000);

    return 4 * (uint64_t) ifHz <= 7000000 ? 4 * (uint64_t) ifHz : least;
}

/* the rates a low IF offers: from just above that over 512 (8 halvings and the resampler's
   2 at most), to 4 x IF (2 x IF with one converter), past which the tuner's filter leaves
   nothing. The library takes more, up to the converters' most */
void SoapyMiriSDR::lowIfRange(uint32_t ifHz, uint32_t &lo, uint32_t &hi) const
{
    const uint64_t one = oneOut() ? 1 : 0;

    lo = (uint32_t) (convPref(ifHz) / 512 + 1);
    hi = (uint32_t) std::min<uint64_t>((uint64_t) rateMax, 4 * (uint64_t) ifHz) / (one ? 2 : 1);
}

/* "auto" below rateMin: an IF that has this rate free, as before; else the one with the
   least converter rate whose widest filter passes most of it. Never 2048 kHz: 1620 kHz has
   the same filters at a lower converter rate */
uint32_t SoapyMiriSDR::pickLowIf(uint32_t rate) const
{
    const int one = oneOut();
    uint32_t pick = 0, lo, hi;
    uint64_t best = 0;

    for (uint32_t f : IFS)
    {
        std::vector<uint32_t> r = lowIfRates(f);
        if (f && f != 2048000 && std::find(r.begin(), r.end(), rate) != r.end()) return f;
    }

    for (uint32_t f : IFS)
    {
        uint64_t widest = f > 450000 ? 1536000 : 600000, conv;

        lowIfRange(f, lo, hi);
        if (!f || f == 2048000 || rate < lo || rate > hi || widest < 0.8 * rate) continue;
        conv = std::max<uint64_t>(convPref(f), (uint64_t) rate << one);
        if (!pick || conv < best) { pick = f; best = conv; }
    }

    /* none passes most of it (two tuners above 1.2 Msps): the widest filter that has the rate, or if
       none has it: the one whose span ends nearest, so the rate is taken to that end */
    if (!pick)
    {
        uint64_t off, least = 0;

        for (uint32_t f : { 1620000u, 450000u })
        {
            lowIfRange(f, lo, hi);
            if (rate >= lo && rate <= hi) return f;
            off = rate > hi ? rate - hi : lo - rate;
            if (!pick || off < least) { pick = f; least = off; }
        }
    }

    return pick;
}

bool SoapyMiriSDR::realOut(void) const
{
    return !baseband && ifFreq && converters != MIRISDR_IQ_BOTH;
}

bool SoapyMiriSDR::configure(uint32_t rate)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t c = streamCfg;
    uint32_t ifHz = 0, out = rate;

    ifHz = fixedIf();

    /* below the converters own minimum only a low IF gets there, and two tuners always take one */
    if (baseband && ifMode == "auto" && (rate < rateMin || streamTuners() == 3)) ifHz = pickLowIf(rate);

    /* a low IF has a span of rates: the nearest end outside it */
    if (baseband && ifHz)
    {
        uint32_t lo, hi;

        lowIfRange(ifHz, lo, hi);
        out = std::min(std::max(rate, lo), hi);
    }

    /* the tune of each tuner streamed: the IF, one converter or both (two tuners take both
       outputs, a converter each), the filter nearest to what was asked */
    for (int t = 0; t < 2; t++)
    {
        if (!(streamTuners() & (1 << t))) continue;
        tunerSel = t;
        mirisdr_get_tune(dev, tunerSel, &tc, NULL);
        tc.if_freq = ifHz;
        tc.iq = (ifHz && streamTuners() != 3) ? converters : MIRISDR_IQ_BOTH;
        tc.low_if_auto = 0;
        tc.bandwidth = pickBandwidth(ifHz);
        tc.gain.mode = MIRISDR_GAIN_KEEP;
        if (mirisdr_tune(dev, tunerSel, &tc, NULL) < 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: IF %u refused on tuner %d", ifHz, t);
            return false;
        }
    }
    tunerSel = (streamTuners() & 1) ? 0 : 1;
    ifFreq = ifHz;

    c.rate = out;
    c.baseband = baseband;
    if (!applyStream(c)) return false;
    if (out != rate) SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: %u S/s, the nearest to %u with a %u Hz IF", out, rate, ifHz);

    mirisdr_stream_result_t r;
    mirisdr_tune_result_t tr;
    if (mirisdr_get_stream(dev, NULL, &r) < 0) return true;

    /* the library takes a rate out of its reach to the nearest it has */
    if ((r.rate + 16 < out) || (r.rate > out + 16))
        SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: %u S/s, the nearest the receiver has to %u", r.rate, out);

    /* the converters past their best clock, the format short of 14 bits, or a 2048 kHz IF
       where 1620 kHz has the same filters for less */
    if (r.adc_clock > MIRISDR_ADC_CLOCK_BEST)
        SoapySDR_logf(SOAPY_SDR_WARNING, "mirisdr: the converters clock at %.1f MHz, past their best %.0f MHz "
                      "(7.5 MS/s, or 15 MS/s with the decimation bypass): their resolution drops",
                      r.adc_clock / 1e6, MIRISDR_ADC_CLOCK_BEST / 1e6);
    if (r.bits && r.bits < 14)
        SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: %u S/s over USB fits the %s format, %d bits a sample",
                      r.adc_rate, r.format, r.bits);
    if (ifHz == 2048000)
        SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: a 1620 kHz IF has the same filters as 2048 kHz at a lower "
                      "converter rate");

    /* a rate that is not the converters' halved, or with them off 4 x IF: say which
       nearby ones need neither. Direct sampling always runs the NCO */
    if (mirisdr_get_tune(dev, tunerSel, NULL, &tr) == 0 && tr.frontend.direct_sampling) return true;
    if (r.nco || r.resampled)
    {
        uint32_t below = 0, above = 0;

        for (uint32_t f : lowIfRates(ifHz))
        {
            if (f < r.rate && f > below) below = f;
            if (f > r.rate && (!above || f < above)) above = f;
        }
        SoapySDR_logf(SOAPY_SDR_WARNING, "mirisdr: %u S/s with a %u Hz IF needs the %s, the receiver at %u S/s: "
                      "more CPU than %u%s%.0u S/s, which need neither", r.rate, ifHz,
                      r.nco && r.resampled ? "resampler and the NCO" : r.nco ? "NCO" : "resampler", r.adc_rate,
                      below ? below : above, below && above ? " or " : "", below && above ? above : 0);
    }

    return true;
}

/*******************************************************************
 * Identification
 ******************************************************************/

std::string SoapyMiriSDR::getDriverKey(void) const
{
    return SOAPY_MIRISDR_DRIVER;
}

std::string SoapyMiriSDR::getHardwareKey(void) const
{
    switch (flavour)
    {
    case MIRISDR_HW_RSP1B:   return "RSP1B";
    case MIRISDR_HW_RSPDXR2: return "RSPdxR2";
    case MIRISDR_HW_RSPDUO:  return "RSPduo";
    case MIRISDR_HW_SDRPLAY: return "SDRplay";
    default:                 return "MSi2500";
    }
}

SoapySDR::Kwargs SoapyMiriSDR::getHardwareInfo(void) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    SoapySDR::Kwargs info;
    uint8_t id[MIRISDR_FW_ID_LEN];
    uint16_t vid, pid;
    char s[64];
    int i;

    if (!serial.empty()) info["serial"] = serial;
    if (!usbPort.empty()) info["usb_port"] = usbPort;
    if (mirisdr_get_usb_ids(dev, &vid, &pid) == 0)
    {
        snprintf(s, sizeof s, "%04x:%04x", vid, pid);
        info["usb_id"] = s;
    }
    if (mirisdr_running_from_rom(dev) == 1) info["firmware"] = "ROM";
    else if (mirisdr_get_fw_id(dev, id, sizeof id) == (int) sizeof id)
    {
        for (i = 0; i < (int) sizeof id; i++) snprintf(s + 2 * i, 3, "%02x", id[i]);
        info["firmware"] = s;
    }
    return info;
}

/* the RSPduo's two tuners are channels 0 and 1, streamed alone or together */
int SoapyMiriSDR::chanOf(size_t channel) const
{
    if (channel >= (size_t) std::max(1, mirisdr_get_tuner_count(dev)))
        throw std::runtime_error("mirisdr: no channel " + std::to_string(channel));
    return (int) channel;
}

size_t SoapyMiriSDR::getNumChannels(const int direction) const
{
    return direction == SOAPY_SDR_RX ? (size_t) std::max(1, mirisdr_get_tuner_count(dev)) : 0;
}

/*******************************************************************
 * Antenna
 ******************************************************************/

/* The channel's inputs as the library lists them: with a Hi-Z, "RX" and "Hi-Z" (the
   RSPduo's channel 0), with several ports their letters (the RSPdx-R2's A, B and C),
   otherwise the one "RX" */
static std::string antennaName(const mirisdr_frontend_caps_t &c, int a)
{
    if (c.antenna_hiz >= 0) return a == c.antenna_hiz ? "Hi-Z" : "RX";
    if (c.antennas & ~(1u << MIRISDR_ANTENNA_A)) return std::string(1, (char) ('A' + a - MIRISDR_ANTENNA_A));
    return "RX";
}

std::vector<std::string> SoapyMiriSDR::listAntennas(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::vector<std::string> list;

    for (int a = 0; a < 32; a++)
        if (cap().antennas & (1u << a)) list.push_back(antennaName(cap(), a));
    return list;
}

void SoapyMiriSDR::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    for (int a = 0; a < 32; a++)
    {
        if (!(cap().antennas & (1u << a)) || antennaName(cap(), a) != name) continue;
        /* the channel's own tune: the Hi-Z ends at 30 MHz, refused above. The one in use
           already: nothing to do (programs set it again with every start) */
        mirisdr_get_tune(dev, tunerSel, &tc, NULL);
        if (tc.frontend.antenna == a) return;
        tc.frontend.antenna = a;
        tc.gain.mode = MIRISDR_GAIN_KEEP;
        applyTune(tc);
        return;
    }
    SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no antenna %s", name.c_str());
}

std::string SoapyMiriSDR::getAntenna(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    if (mirisdr_get_tune(dev, tunerSel, &tc, NULL) < 0) return "RX";
    return antennaName(cap(), tc.frontend.antenna);
}

/*******************************************************************
 * DC offset: the library takes it off in software under baseband, by
 * default at a low IF and not at zero IF, where it is the band's
 * centre. Setting the mode turns it on or off at every IF
 ******************************************************************/

bool SoapyMiriSDR::hasDCOffsetMode(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    return true;
}

void SoapyMiriSDR::setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.dc_removal = automatic ? MIRISDR_DC_ON : MIRISDR_DC_OFF;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

bool SoapyMiriSDR::getDCOffsetMode(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    if (!baseband || tc.dc_removal == MIRISDR_DC_OFF) return false;
    return tc.dc_removal == MIRISDR_DC_ON || ifFreq != 0;
}

/*******************************************************************
 * Gain: the total split by the library, or each stage. The stages a
 * band has and their steps are asked from the library
 ******************************************************************/

/* ATT, the board's pads, comes first as they are first in the signal's way. It is a negative
   gain, so more is more signal as on the other sliders, and apart from the total as in the
   library: the total is the tuner's chain, and its split would never take a pad */
std::vector<std::string> SoapyMiriSDR::listGains(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::vector<std::string> list;

    if (cap().attenuator_count > 1) list.push_back("ATT");
    /* the board's amplifier before the tuner, where it can be bypassed */
    if (cap().amp_bypass_hz) list.push_back("AMP");
    for (const char *g : { "LNA", "MIX", "MIXBUF", "BB" }) list.push_back(g);
    return list;
}

/* the pad nearest db, in the selected tuner's tune */
void SoapyMiriSDR::setPads(int db)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    int best = 0;

    for (int k = 0; k < cap().attenuator_count; k++)
        if (std::abs(cap().attenuator[k] - db) < std::abs(best - db)) best = cap().attenuator[k];

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    if (tc.frontend.attenuator == best) return;
    tc.frontend.attenuator = best;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

/* ATT's range: down to the board's most, in its step where the pads are evenly spaced
   (the RSPdx-R2's 3 dB), else by 1 dB, each value taking the nearest pad */
SoapySDR::Range SoapyMiriSDR::attRange(void) const
{
    const mirisdr_frontend_caps_t &c = cap();
    int step = c.attenuator_count > 1 ? c.attenuator[1] : 1;

    if (c.attenuator_count < 2) return SoapySDR::Range(0, 0);
    for (int k = 2; k < c.attenuator_count; k++)
        if (c.attenuator[k] - c.attenuator[k - 1] != step) step = 1;
    return SoapySDR::Range(-c.attenuator[c.attenuator_count - 1], 0, step);
}

SoapyMiriSDR::StageRanges SoapyMiriSDR::stageRanges(void) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_tune_result_t r;
    StageRanges sr = { 0, 0, 0, 0, { 0 } };
    int base, v;

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.gain.mode = MIRISDR_GAIN_STAGES;
    tc.gain.lna = tc.gain.mixer = tc.gain.mixbuffer = tc.gain.baseband = 0;
    tc.gain.amp_bypass = 1;
    if (mirisdr_tune_check(dev, tunerSel, &tc, &r) < 0) return sr;
    base = r.gain.total;

    tc.gain.amp_bypass = 0;
    if (mirisdr_tune_check(dev, tunerSel, &tc, &r) == 0) sr.amp = r.gain.total - base;
    tc.gain.amp_bypass = 1;

    tc.gain.lna = 1;
    if (mirisdr_tune_check(dev, tunerSel, &tc, &r) == 0) sr.lna = r.gain.total - base;
    tc.gain.lna = 0;

    tc.gain.mixer = 1;
    if (mirisdr_tune_check(dev, tunerSel, &tc, &r) == 0) sr.mixer = r.gain.total - base;
    tc.gain.mixer = 0;

    /* only the values that add gain: bands without the stage take any */
    for (v = 1; v <= 24; v++)
    {
        tc.gain.mixbuffer = v;
        if (mirisdr_tune_check(dev, tunerSel, &tc, &r) == 0 && r.gain.total - base == v) sr.mixbuffer.push_back(v);
    }
    tc.gain.mixbuffer = 0;

    for (v = 59; v > 0; v--)
    {
        tc.gain.baseband = v;
        if (mirisdr_tune_check(dev, tunerSel, &tc, &r) == 0) break;
    }
    /* none where it adds nothing: under direct sampling only the LNA counts */
    sr.baseband = (v > 0 && r.gain.total == base) ? 0 : v;

    return sr;
}

void SoapyMiriSDR::setGain(const int direction, const size_t channel, const double value)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = std::max(0, (int) std::lround(value));
    applyTune(tc);
}

void SoapyMiriSDR::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_tune_result_t r;
    int v = (int) std::lround(value), best, d;

    if (name == "ATT" && cap().attenuator_count > 1)
    {
        setPads(-v);
        return;
    }

    StageRanges sr = stageRanges();

    /* the stages in force, then this one changed */
    mirisdr_get_tune(dev, tunerSel, &tc, &r);
    tc.gain = r.gain;
    tc.gain.mode = MIRISDR_GAIN_STAGES;

    if (name == "AMP") tc.gain.amp_bypass = (sr.amp > 0 && 2 * v >= sr.amp) ? 0 : 1;
    else if (name == "LNA") tc.gain.lna = (sr.lna > 0 && 2 * v >= sr.lna) ? 1 : 0;
    else if (name == "MIX") tc.gain.mixer = (sr.mixer > 0 && 2 * v >= sr.mixer) ? 1 : 0;
    else if (name == "BB") tc.gain.baseband = std::min(std::max(v, 0), sr.baseband);
    else if (name == "MIXBUF")
    {
        best = 0;
        for (int m : sr.mixbuffer)
        {
            d = std::abs(m - v);
            if (d < std::abs(best - v)) best = m;
        }
        tc.gain.mixbuffer = best;
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no gain %s", name.c_str());
        return;
    }
    applyTune(tc);
}

double SoapyMiriSDR::getGain(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;

    if (mirisdr_get_tune(dev, tunerSel, NULL, &r) < 0) return 0;
    return r.gain.total;
}

double SoapyMiriSDR::getGain(const int direction, const size_t channel, const std::string &name) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;
    StageRanges sr;

    if (mirisdr_get_tune(dev, tunerSel, NULL, &r) < 0) return 0;
    if (name == "ATT") return -r.frontend.attenuator;
    if (name == "MIXBUF") return r.gain.mixbuffer;
    if (name == "BB") return r.gain.baseband;

    sr = stageRanges();
    if (name == "AMP") return r.gain.amp_bypass ? 0 : sr.amp;
    if (name == "LNA") return r.gain.lna ? sr.lna : 0;
    if (name == "MIX") return r.gain.mixer ? sr.mixer : 0;
    return 0;
}

/* The most each stage reaches in any band: the LNA below band IV/V, the mixbuffer on
   AM2, which never run together. The mixbuffer is 0 or 24 dB on AM2. The RSPduo's Hi-Z
   runs on AM1, which steps it by 6 up to 18, so a channel with a Hi-Z steps by 6. The board's
   amplifier, its best band's, comes from the library */
#define MAX_LNA                 24
#define MAX_MIX                 19
#define MAX_MIXBUF              24
#define MAX_BB                  59

SoapySDR::Range SoapyMiriSDR::getGainRange(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    StageRanges sr;

    if (!bandGainRanges)
        return SoapySDR::Range(0, cap().amp_max_db + std::max(MAX_LNA, MAX_MIXBUF) +
                               MAX_MIX + MAX_BB, 1);

    sr = stageRanges();
    return SoapySDR::Range(0, sr.amp + sr.lna + sr.mixer + sr.mixbuffer.back() + sr.baseband, 1);
}

SoapySDR::Range SoapyMiriSDR::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    StageRanges sr;

    if (name == "ATT") return attRange();
    if (!bandGainRanges)
    {
        /* the RSPdx-R2's direct sampling amplifier counts as its LNA */
        if (name == "LNA")
        {
            int lna = cap().direct_sampling ? MIRISDR_DS_LNA_DB : MAX_LNA;
            return SoapySDR::Range(0, lna, lna);
        }
        if (name == "AMP" && cap().amp_bypass_hz) return SoapySDR::Range(0, cap().amp_max_db, cap().amp_max_db);
        if (name == "MIX") return SoapySDR::Range(0, MAX_MIX, MAX_MIX);
        if (name == "MIXBUF") return SoapySDR::Range(0, MAX_MIXBUF, cap().antenna_hiz >= 0 ? 6 : MAX_MIXBUF);
        if (name == "BB") return SoapySDR::Range(0, MAX_BB, 1);
        return SoapySDR::Range(0, 0);
    }

    sr = stageRanges();

    if (name == "AMP") return SoapySDR::Range(0, sr.amp, sr.amp ? sr.amp : 1);
    if (name == "LNA") return SoapySDR::Range(0, sr.lna, sr.lna ? sr.lna : 1);
    if (name == "MIX") return SoapySDR::Range(0, sr.mixer, sr.mixer ? sr.mixer : 1);
    if (name == "BB") return SoapySDR::Range(0, sr.baseband, 1);
    if (name == "MIXBUF")
        return SoapySDR::Range(0, sr.mixbuffer.back(), sr.mixbuffer.size() > 1 ? sr.mixbuffer[1] : 1);
    return SoapySDR::Range(0, 0);
}

/*******************************************************************
 * Frequency: the LO, which is the centre of the stream. With a low IF
 * the band received lies the IF below it
 ******************************************************************/

void SoapyMiriSDR::setFrequency(const int direction, const size_t channel, const double frequency,
                                const SoapySDR::Kwargs &args)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    setFrequency(direction, channel, "RF", frequency, args);
}

void SoapyMiriSDR::setFrequency(const int direction, const size_t channel, const std::string &name,
                                const double frequency, const SoapySDR::Kwargs &args)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    if (name != "RF")
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no frequency %s", name.c_str());
        return;
    }
    if (frequency < 0 || frequency > 4294967295.0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: cannot tune to %.0f Hz", frequency);
        return;
    }

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.frequency = (uint32_t) std::llround(frequency);
    tc.low_if_auto = 0;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

double SoapyMiriSDR::getFrequency(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    return getFrequency(direction, channel, "RF");
}

double SoapyMiriSDR::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;

    mirisdr_tune_config_t c;

    if (name != "RF" || mirisdr_get_tune(dev, tunerSel, &c, &r) < 0) return 0;
    /* baseband: the band asked for is the centre; raw, the LO is */
    return baseband ? c.frequency : r.lo;
}

std::vector<std::string> SoapyMiriSDR::listFrequencies(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    return { "RF" };
}

SoapySDR::RangeList SoapyMiriSDR::getFrequencyRange(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    return getFrequencyRange(direction, channel, "RF");
}

SoapySDR::RangeList SoapyMiriSDR::getFrequencyRange(const int direction, const size_t channel,
                                                    const std::string &name) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    return { SoapySDR::Range(freqMin, freqMax) };
}

/*******************************************************************
 * Sample rate and bandwidth
 ******************************************************************/

void SoapyMiriSDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    wantRate = (uint32_t) std::llround(rate);
    configure(wantRate);
}

double SoapyMiriSDR::getSampleRate(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_result_t r;

    if (mirisdr_get_stream(dev, NULL, &r) < 0) return 0;
    return r.rate;
}

/* rates within a span, once each */
static void addRates(std::vector<double> &v, const std::vector<double> &rates, double lo, double hi)
{
    for (double r : rates)
        if (r >= lo && r <= hi && std::find(v.begin(), v.end(), r) == v.end()) v.push_back(r);
}

/* A list of the usual ones. With a low IF those needing no NCO, from 25 ksps, within
   its span */
std::vector<double> SoapyMiriSDR::listSampleRates(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    static const std::vector<double> common = { 1.536e6, 2e6, 2.048e6, 2.4e6, 3e6, 4e6, 5e6, 6e6, 7e6,
                                                8e6, 9e6, 10e6, 12e6, 14e6 };
    static const std::vector<double> low = { 48e3, 96e3, 128e3, 192e3, 250e3, 256e3, 500e3, 512e3, 1e6, 1.024e6 };
    std::vector<double> rates;
    uint32_t fixed = fixedIf(), lo, hi;

    /* a low IF alone: the IF asked for, or with two tuners those auto picks from */
    if (baseband && (fixed || streamTuners() == 3))
    {
        for (uint32_t f : fixed ? std::vector<uint32_t>{ fixed } : std::vector<uint32_t>{ 450000, 1620000 })
        {
            lowIfRange(f, lo, hi);
            for (uint32_t r : lowIfRates(f)) addRates(rates, { (double) r }, std::max<double>(lo, 25000), hi);
            addRates(rates, low, lo, hi);
            addRates(rates, common, lo, hi);
        }
    }
    else
    {
        if (baseband)
            for (uint32_t f : IFS)
                if (f)
                {
                    lowIfRange(f, lo, hi);
                    for (uint32_t r : lowIfRates(f)) addRates(rates, { (double) r }, std::max<double>(lo, 25000), rateMin - 1);
                    addRates(rates, low, lo, rateMin - 1);
                }
        addRates(rates, common, rateMin, rateMax);
    }
    std::sort(rates.begin(), rates.end());
    return rates;
}

/* Any rate in a span: a low IF's, or with "auto" the lowest of them up to zero IF's most */
SoapySDR::RangeList SoapyMiriSDR::getSampleRateRange(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    SoapySDR::RangeList out;
    uint32_t fixed = fixedIf(), lo, hi;

    if (baseband && fixed)
    {
        lowIfRange(fixed, lo, hi);
        out.push_back(SoapySDR::Range(lo, hi));
    }
    else if (baseband && streamTuners() == 3)
    {
        /* two tuners: the 450 kHz IF's least to the 1620 kHz IF's most */
        uint32_t lo2, hi2;

        lowIfRange(450000, lo, hi);
        lowIfRange(1620000, lo2, hi2);
        out.push_back(SoapySDR::Range(std::min(lo, lo2), std::max(hi, hi2)));
    }
    else if (baseband)
    {
        lowIfRange(IFS[1], lo, hi);
        out.push_back(SoapySDR::Range(std::min<double>(lo, rateMin), rateMax));
    }
    else out.push_back(SoapySDR::Range(rateMin, rateMax));
    return out;
}

std::vector<uint32_t> SoapyMiriSDR::bandwidthsFor(uint32_t ifHz) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    std::vector<uint32_t> bws;
    mirisdr_tune_config_t tc;

    /* the filters this IF has, as configure() would tune it: strict, so one it has not is refused
       rather than taken to the nearest */
    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.if_freq = ifHz;
    tc.iq = (ifHz && streamTuners() != 3) ? converters : MIRISDR_IQ_BOTH;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    tc.strict = 1;
    for (uint32_t bw : BANDWIDTHS)
    {
        tc.bandwidth = bw;
        if (mirisdr_tune_check(dev, tunerSel, &tc, NULL) == 0) bws.push_back(bw);
    }
    return bws;
}

/* the narrowest filter of the IF that passes wantBw, else its widest. 0 when
   nothing is asked: the library picks */
uint32_t SoapyMiriSDR::pickBandwidth(uint32_t ifHz) const
{
    std::vector<uint32_t> bws = bandwidthsFor(ifHz);

    if (!wantBw || bws.empty()) return 0;
    for (uint32_t b : bws)
        if (b >= wantBw) return b;
    return bws.back();
}

void SoapyMiriSDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    /* remembered, so a change of IF takes the nearest it has. 0 is the device
       string's, or else the library's choice */
    wantBw = bw > 0 ? (uint32_t) bw : argBw;

    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.bandwidth = pickBandwidth(ifFreq);
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

double SoapyMiriSDR::getBandwidth(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;

    if (mirisdr_get_tune(dev, tunerSel, NULL, &r) < 0) return 0;
    return r.bandwidth;
}

std::vector<double> SoapyMiriSDR::listBandwidths(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    std::vector<double> out;

    for (uint32_t b : bandwidthsFor(ifFreq)) out.push_back(b);
    return out;
}

SoapySDR::RangeList SoapyMiriSDR::getBandwidthRange(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    SoapySDR::RangeList out;

    for (uint32_t b : bandwidthsFor(ifFreq)) out.push_back(SoapySDR::Range(b, b));
    return out;
}

/*******************************************************************
 * Settings
 ******************************************************************/

SoapySDR::ArgInfoList SoapyMiriSDR::getSettingInfo(void) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    SoapySDR::ArgInfoList list;
    SoapySDR::ArgInfo a;

    /* the device's settings are channel 0's where a channel has its own */
    tunerSel = 0;

    a = SoapySDR::ArgInfo();
    a.key = "if_freq";
    a.name = "IF";
    a.description = "Tuner IF in Hz. auto: zero IF from 1.3 Msps up, a low IF below. A low IF has its own "
                    "sample rates and filters.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "auto";
    a.options.push_back("auto");
    for (uint32_t f : IFS) a.options.push_back(std::to_string(f));
    a.optionNames = { "Auto", "Zero IF", "450 kHz", "1620 kHz", "2048 kHz" };
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "converters";
    a.name = "Converters";
    a.description = "With a low IF: both, or one (I or Q), which halves the USB data and keeps more bits "
                    "at high rates, using more CPU.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "both";
    for (const char *c : CONVERTERS) a.options.push_back(c);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "baseband";
    a.name = "Baseband";
    a.description = "The frequency set at the centre of the stream at any IF. Off: the stream "
                    "unmodified, the LO at the centre, real samples from a single converter.";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = "true";
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "format";
    a.name = "USB sample format";
    a.description = "AUTO picks a format that fits automatically.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "AUTO";
    for (const char *f : FORMATS) a.options.push_back(f);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "transfer";
    a.name = "USB transfer";
    a.description = "Bulk, or isochronous with 3, 1 or 2 packets per microframe.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = readSetting("transfer");
    for (const char *t : TRANSFERS) a.options.push_back(t);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "decimation_bypass";
    a.name = "Decimation bypass";
    a.description = "Run the ADC at the sample rate without the internal filter. AUTO above 14.5 Msps.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "AUTO";
    for (const char *b : BYPASS) a.options.push_back(b);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "gap_fill";
    a.name = "Gap fill";
    a.description = "Put zeros where samples were lost on the USB, so the timing stays right. Always on with baseband, the default.";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = "false";
    list.push_back(a);

    if (biasTuner() >= 0)
    {
        a = SoapySDR::ArgInfo();
        a.key = "biastee";
        a.name = "Bias-T";
        a.description = flavour == MIRISDR_HW_RSPDXR2 ? "Power on antenna port B." :
                        flavour == MIRISDR_HW_RSPDUO ? "Power on the port marked tuner 2 (channel 1)." : "Power on the antenna input.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }

    notchInfo(list, true);

    if (caps[0].flags & MIRISDR_FRONTEND_PORT_TIE)
    {
        a = SoapySDR::ArgInfo();
        a.key = "port_tie";
        a.name = "Port tie";
        a.description = "Tie the two 50 ohm ports together, so both channels hear both, as from one antenna. "
                        "Works at HF, 50-200 MHz and 600-900 MHz; much less at 300-433 MHz and in L band.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }

    if (caps[0].flags & MIRISDR_FRONTEND_REF_OUT)
    {
        a = SoapySDR::ArgInfo();
        a.key = "ref_out";
        a.name = "Reference output";
        a.description = "The board's 24 MHz clock out of its reference connector, for another receiver.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }

    if (caps[0].direct_sampling)
    {
        a = SoapySDR::ArgInfo();
        a.key = "direct_sampling";
        a.name = "Direct sampling";
        a.description = "Sample the antenna directly instead of through the tuner: ON at 2 MHz and below, "
                        "AUTO where the whole band fits under 2 MHz. The gain is then its LNA alone, about 37 dB.";
        a.type = SoapySDR::ArgInfo::STRING;
        a.value = "AUTO";
        a.options = { "OFF", "ON", "AUTO" };
        list.push_back(a);
    }

    return list;
}

static int notchBit(const std::string &key)
{
    return key == "fm_notch" ? MIRISDR_NOTCH_FM : key == "mw_notch" ? MIRISDR_NOTCH_MW :
           key == "dab_notch" ? MIRISDR_NOTCH_DAB : 0;
}

void SoapyMiriSDR::writeSetting(const std::string &key, const std::string &value)
{
    ReportScope report_(*this);
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    tunerSel = 0;

    if (key == "if_freq")
    {
        if (!parseIf(value, ifMode))
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no IF %s", value.c_str());
            return;
        }
        configure(wantRate);
    }
    else if (key == "converters")
    {
        converters = value == "I" ? MIRISDR_IQ_ONLY_I : value == "Q" ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_BOTH;
        configure(wantRate);
    }
    else if (key == "baseband")
    {
        baseband = isTrue(value);
        configure(wantRate);
    }
    else if (key == "format" || key == "transfer" || key == "decimation_bypass")
    {
        std::string &s = key == "format" ? formatSetting : key == "transfer" ? transferSetting : bypassSetting;
        std::string was = s;

        s = (value == "AUTO" && key != "transfer") ? "" : value;
        applyStream(streamCfg, true);

        /* refused: keep what runs */
        mirisdr_stream_config_t now;
        mirisdr_get_stream(dev, &now, NULL);
        if (key == "format" && !s.empty() && (!now.format || s != now.format)) s = was;
        if (key == "transfer" && !s.empty() && (!now.transfer || s != now.transfer)) s = was;
        if (key == "decimation_bypass" && !s.empty() && (!now.decimation_bypass || s != now.decimation_bypass)) s = was;

        /* the rates a format and transfer reach differ */
        probeRanges();
        configure(wantRate);
    }
    else if (key == "gap_fill")
    {
        mirisdr_stream_config_t c = streamCfg;
        c.gap_fill = isTrue(value);
        applyStream(c);
    }
    else if (key == "biastee" && biasTuner() >= 0)
    {
        /* the tune of the tuner whose input has it: the RSPduo's channel 1 */
        mirisdr_tune_config_t tc;
        int keep = tunerSel;
        tunerSel = biasTuner();
        mirisdr_get_tune(dev, tunerSel, &tc, NULL);
        tc.frontend.bias = isTrue(value) ? 1 : 0;
        tc.gain.mode = MIRISDR_GAIN_KEEP;
        applyTune(tc);
        tunerSel = keep;
    }
    else if ((key == "port_tie" && (caps[0].flags & MIRISDR_FRONTEND_PORT_TIE)) ||
             (key == "ref_out" && (caps[0].flags & MIRISDR_FRONTEND_REF_OUT)))
    {
        /* the board's: the streamed tuner's tune carries it to both (channel 1's when it
           streams alone, so the tuner not streamed keeps no filter path) */
        mirisdr_tune_config_t tc;
        uint32_t bit = key == "port_tie" ? MIRISDR_FRONTEND_PORT_TIE : MIRISDR_FRONTEND_REF_OUT;
        tunerSel = (streamTuners() & 1) ? 0 : 1;
        mirisdr_get_tune(dev, tunerSel, &tc, NULL);
        tc.frontend.flags = isTrue(value) ? (tc.frontend.flags | bit) : (tc.frontend.flags & ~bit);
        tc.gain.mode = MIRISDR_GAIN_KEEP;
        applyTune(tc);
        tunerSel = 0;
    }
    else if (key == "direct_sampling")
    {
        int ds = parseDs(value);
        if (ds < 0 || mirisdr_set_direct_sampling(dev, ds) < 0)
            SoapySDR_log(SOAPY_SDR_ERROR, "mirisdr: direct sampling refused");
    }
    else if (notchBit(key)) writeSetting(SOAPY_SDR_RX, 0, key, value);
    else SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no setting %s", key.c_str());
}

std::string SoapyMiriSDR::readSetting(const std::string &key) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_config_t c;

    tunerSel = 0;

    if (key == "if_freq") return ifMode;
    if (key == "if_in_use") return std::to_string(ifFreq);
    if (key == "converters") return converters == MIRISDR_IQ_ONLY_I ? "I" : converters == MIRISDR_IQ_ONLY_Q ? "Q" : "both";
    if (key == "baseband") return baseband ? "true" : "false";
    if (key == "format") return formatSetting.empty() ? "AUTO" : formatSetting;
    if (key == "transfer")
    {
        mirisdr_get_stream(dev, &c, NULL);
        return c.transfer ? c.transfer : "";
    }
    if (key == "decimation_bypass") return bypassSetting.empty() ? "AUTO" : bypassSetting;
    if (key == "gap_fill")
    {
        mirisdr_stream_result_t r;
        mirisdr_get_stream(dev, NULL, &r);
        return r.gap_fill ? "true" : "false";
    }
    if (key == "biastee")
    {
        mirisdr_tune_result_t r;
        return (biasTuner() >= 0 && mirisdr_get_tune(dev, biasTuner(), NULL, &r) == 0 && r.frontend.bias) ? "true" : "false";
    }
    if (key == "port_tie" || key == "ref_out")
    {
        int f = mirisdr_get_frontend_flags(dev);
        return (f > 0 && (f & (key == "port_tie" ? MIRISDR_FRONTEND_PORT_TIE : MIRISDR_FRONTEND_REF_OUT))) ? "true" : "false";
    }
    if (key == "direct_sampling")
    {
        int ds = mirisdr_get_direct_sampling(dev);
        return ds == MIRISDR_DS_AUTO ? "AUTO" : ds == MIRISDR_DS_ON ? "ON" : "OFF";
    }
    if (notchBit(key)) return readSetting(SOAPY_SDR_RX, 0, key);
    return "";
}

/*******************************************************************
 * Channel settings: the pads and notches before each tuner, as the library lists them, and
 * the tuner with the bias-T
 ******************************************************************/

int SoapyMiriSDR::biasTuner(void) const
{
    for (int t = 0; t < mirisdr_get_tuner_count(dev) && t < 2; t++)
        if (caps[t].bias) return t;
    return -1;
}

/* the notches the selected tuner's input has. With device: the device settings, channel 0's */
void SoapyMiriSDR::notchInfo(SoapySDR::ArgInfoList &list, bool device) const
{
    static const struct { int bit; const char *key, *name, *band; } notches[] = {
        { MIRISDR_NOTCH_FM, "fm_notch", "FM notch", "the FM band (85-100 MHz)" },
        { MIRISDR_NOTCH_MW, "mw_notch", "MW notch", "medium wave (0.4-1.6 MHz)" },
        { MIRISDR_NOTCH_DAB, "dab_notch", "DAB notch", "DAB (165-230 MHz)" } };

    for (const auto &n : notches)
    {
        SoapySDR::ArgInfo a;

        if (!(cap().notches & n.bit)) continue;
        a.key = n.key;
        a.name = n.name;
        a.description = std::string("Notch for ") + n.band + ".";
        if (device && mirisdr_get_tuner_count(dev) > 1) a.description += " Channel 0's here; each channel has its own as a channel setting.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }
}

SoapySDR::ArgInfoList SoapyMiriSDR::getSettingInfo(const int direction, const size_t channel) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    SoapySDR::ArgInfoList list;

    notchInfo(list, false);
    return list;
}

void SoapyMiriSDR::writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value)
{
    ReportScope report_(*this);
    SOAPY_MIRISDR_CHANNEL(channel);
    mirisdr_tune_config_t tc;
    int bit = notchBit(key);

    /* a notch, in this channel's tune: the only channel settings, the pads are the ATT gain */
    if (!bit)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no channel setting %s", key.c_str());
        return;
    }
    if (!(cap().notches & bit))
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no %s on channel %zu", key.c_str(), channel);
        return;
    }
    mirisdr_get_tune(dev, tunerSel, &tc, NULL);
    tc.frontend.notch = isTrue(value) ? (tc.frontend.notch | bit) : (tc.frontend.notch & ~bit);
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

std::string SoapyMiriSDR::readSetting(const int direction, const size_t channel, const std::string &key) const
{
    SOAPY_MIRISDR_CHANNEL(channel);
    mirisdr_tune_result_t r;
    int bit = notchBit(key);

    if (!bit || mirisdr_get_tune(dev, tunerSel, NULL, &r) < 0) return "";
    return (r.frontend.notch & bit) ? "true" : "false";
}

/*******************************************************************
 * Time: the stream's sample count, from when it started
 ******************************************************************/

static long long spanNs(uint64_t n, uint32_t rate)
{
    if (!rate) return 0;
    return (long long) ((n / rate) * 1000000000ULL + ((n % rate) * 1000000000ULL) / rate);
}

/* From the last rate change on at its rate, before it at the rates before */
long long SoapyMiriSDR::indexToNs(uint64_t index, uint32_t rate) const
{
    uint64_t from = (uint64_t) timeBaseIndex.load();

    return timeOffsetNs + timeBaseNs + spanNs(index > from ? index - from : 0, rate);
}

/* the rate changes at index: the time so far is kept, at the rate it ran at */
void SoapyMiriSDR::timeRebase(uint64_t index, uint32_t rate)
{
    uint64_t from = (uint64_t) timeBaseIndex.load();

    if (rate == timeBaseRate) return;
    if (timeBaseRate) timeBaseNs += spanNs(index > from ? index - from : 0, timeBaseRate);
    timeBaseIndex = (long long) index;
    timeBaseRate = rate;
}

bool SoapyMiriSDR::hasHardwareTime(const std::string &what) const
{
    return what.empty();
}

long long SoapyMiriSDR::getHardwareTime(const std::string &what) const
{
    return indexToNs(lastIndex, lastRate);
}

void SoapyMiriSDR::setHardwareTime(const long long timeNs, const std::string &what)
{
    timeOffsetNs += timeNs - getHardwareTime(what);
}

/*******************************************************************
 * report=true: what the receiver runs, from the library's results
 * rather than from what was asked, so a program can see where the
 * rules took it. A line is logged when it differs from the last ones
 ******************************************************************/

static std::string strf(const char *f, ...)
{
    char s[512];
    va_list ap;

    va_start(ap, f);
    vsnprintf(s, sizeof s, f, ap);
    va_end(ap);
    return s;
}

static const char *bandName(mirisdr_band_t b)
{
    switch (b)
    {
    case MIRISDR_BAND_AM1: return "AM1";
    case MIRISDR_BAND_AM2: return "AM2";
    case MIRISDR_BAND_VHF: return "VHF";
    case MIRISDR_BAND_3:   return "III";
    case MIRISDR_BAND_45:  return "IV/V";
    case MIRISDR_BAND_L:   return "L";
    }
    return "unknown";
}

/* a filter's width: kHz below 1 MHz, MHz from there */
static std::string widthName(uint32_t hz)
{
    return hz < 1000000 ? strf("%g kHz", hz / 1e3) : strf("%g MHz", hz / 1e6);
}

static std::string notchNames(int n)
{
    std::string s;

    if (n & MIRISDR_NOTCH_FM) s += "FM";
    if (n & MIRISDR_NOTCH_MW) s += s.empty() ? "MW" : " and MW";
    if (n & MIRISDR_NOTCH_DAB) s += s.empty() ? "DAB" : " and DAB";
    return s.empty() ? "none" : s;
}

std::vector<std::string> SoapyMiriSDR::reportLines(void) const
{
    std::vector<std::string> lines;
    mirisdr_stream_result_t r;
    const char *transfer = mirisdr_get_transfer(dev);
    std::string what;

    if (mirisdr_get_stream(dev, NULL, &r) < 0) return lines;

    const char *conv = r.adc == MIRISDR_IQ_ONLY_I ? "converter I" : r.adc == MIRISDR_IQ_ONLY_Q ? "converter Q" : "both converters";
    switch (r.baseband)
    {
    case MIRISDR_BASEBAND_ZERO_IF: what = "complex float at zero IF"; break;
    case MIRISDR_BASEBAND_COMPLEX: what = "complex float from a low IF on both converters"; break;
    case MIRISDR_BASEBAND_REAL:    what = strf("complex float from %s alone", conv); break;
    case MIRISDR_BASEBAND_DUAL:    what = "two tuners, each a low IF on a converter of its own"; break;
    default:                       what = r.adc == MIRISDR_IQ_BOTH ? "raw complex samples" : strf("raw real samples from %s", conv);
    }
    lines.push_back(strf("stream: %u S/s, %s", r.rate, what.c_str()));
    lines.push_back(strf("converters: %u S/s at a %.3f MHz clock%s, decimation bypass %s, format %s with %d bits, "
                         "transfer %s, %u B/s over USB", r.adc_rate, r.adc_clock / 1e6,
                         r.adc_clock > MIRISDR_ADC_CLOCK_BEST ? " (past their best)" : "",
                         r.decimation_bypassed ? "on" : "off", r.format ? r.format : "unknown", r.bits,
                         transfer ? transfer : "unknown", r.usb_bytes));

    for (int t = 0; t < 2; t++)
    {
        mirisdr_tune_config_t tc;
        mirisdr_tune_result_t tr;
        const mirisdr_frontend_caps_t &c = caps[t];
        std::string s, fe;

        if (!(r.tuners & (1 << t)) || mirisdr_get_tune(dev, t, &tc, &tr) < 0) continue;

        /* where it listens, and where that lands in the stream */
        if (tr.frontend.direct_sampling)
            s = strf("tuner %d: %.6f MHz by direct sampling: the antenna through the %s low pass to "
                     "converter I, no LO", t, tc.frequency / 1e6, widthName(tr.bandwidth).c_str());
        else
            s = strf("tuner %d: %.6f MHz, LO %.6f MHz, %s, %s, filter %s, band %s", t, tc.frequency / 1e6,
                     tr.lo / 1e6, tc.if_freq ? strf("low IF %.0f kHz", tc.if_freq / 1e3).c_str() : "zero IF",
                     tr.iq == MIRISDR_IQ_BOTH ? "both outputs" : tr.iq == MIRISDR_IQ_ONLY_I ? "output I only" : "output Q only",
                     widthName(tr.bandwidth).c_str(), bandName(tr.band));
        s += tr.offset ? strf(", in the stream at %+d Hz", tr.offset) : std::string(", in the stream at 0 Hz");
        if (tr.inverted) s += " and mirrored";
        if (r.baseband != MIRISDR_BASEBAND_OFF)
        {
            bool dc = tc.dc_removal == MIRISDR_DC_ON || (tc.dc_removal == MIRISDR_DC_AUTO && tc.if_freq);
            s += strf(", DC removal %s", dc ? "on" : "off");
        }
        lines.push_back(s);

        /* the gain, stage by stage: the LNA is not on the AM ports, the mixbuffer only there */
        const bool am = tr.band == MIRISDR_BAND_AM1 || tr.band == MIRISDR_BAND_AM2;
        if (tr.frontend.direct_sampling)
            s = strf("tuner %d gain: %d dB, the direct sampling amplifier %s", t, tr.gain.total, tr.gain.lna ? "on" : "off");
        else
        {
            s = strf("tuner %d gain: %d dB in total", t, tr.gain.total);
            if (c.amp_bypass_hz && tc.frequency >= c.amp_bypass_hz)
                s += strf(", the board's amplifier %s", tr.gain.amp_bypass ? "bypassed" : "in");
            if (!am) s += strf(", LNA %s", tr.gain.lna ? "on" : "off");
            s += strf(", mixer %s", tr.gain.mixer ? "on" : "off");
            if (am) s += strf(", mixbuffer %d dB", tr.gain.mixbuffer);
            s += strf(", baseband %d dB", tr.gain.baseband);
        }
        lines.push_back(s);

        /* the front end, only what the board has */
        if (c.antennas & ~(1u << MIRISDR_ANTENNA_A)) fe += ", antenna " + antennaName(c, tr.frontend.antenna);
        if (c.attenuator_count > 1) fe += strf(", pads %d dB", tr.frontend.attenuator);
        if (c.notches) fe += ", notches " + notchNames(tr.frontend.notch);
        if (c.bias) fe += strf(", bias-T %s", tr.frontend.bias ? "on" : "off");
        if (c.flags & MIRISDR_FRONTEND_PORT_TIE)
            fe += strf(", port tie %s", (tr.frontend.flags & MIRISDR_FRONTEND_PORT_TIE) ? "on" : "off");
        if (c.flags & MIRISDR_FRONTEND_REF_OUT)
            fe += strf(", reference out %s", (tr.frontend.flags & MIRISDR_FRONTEND_REF_OUT) ? "on" : "off");
        if (!fe.empty()) lines.push_back(strf("tuner %d front end:", t) + fe.substr(1));
    }

    /* the samples' way from the converters to the output */
    if (r.baseband != MIRISDR_BASEBAND_OFF)
    {
        std::string s = strf("baseband: decimation %g%s", r.decimation, r.resampled ? " with the resampler" : ", no resampler");
        if (r.nco) s += ", NCO in use";
        else if (r.baseband == MIRISDR_BASEBAND_ZERO_IF) s += ", no NCO";
        else s += ", no NCO: the converters at 4 x IF, shifted by a quarter of their rate";
        lines.insert(lines.begin() + 2, s);
    }

    return lines;
}

/* the lines that changed since the last report */
void SoapyMiriSDR::report(void)
{
    try {
        std::vector<std::string> now = reportLines();

        for (const std::string &l : now)
            if (std::find(reported.begin(), reported.end(), l) == reported.end())
                SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: %s", l.c_str());
        reported = now;
    } catch (...) {
    }
}
