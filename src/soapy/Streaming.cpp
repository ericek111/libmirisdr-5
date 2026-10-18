/*
 * SoapySDR module for libmirisdr: streaming.
 *
 * The library's stream thread copies each buffer into a ring of slots, with
 * where its samples sit in time and where samples are missing. readStream()
 * converts from the ring, stops at every gap and reports it as an overflow,
 * so the time stamp after it shows how much is missing.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "SoapyMiriSDR.hpp"

#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>

#include <chrono>
#include <cstring>
#include <stdexcept>

#include <algorithm>
#include <cmath>

#define OUT_CS16 0
#define OUT_CF32 1
#define OUT_CS8  2
#define OUT_S16  3      /* real, a single converter without baseband */
#define OUT_F32  4

/* complex, or real when one converter runs without baseband */
std::vector<std::string> SoapyMiriSDR::getStreamFormats(const int direction, const size_t channel) const
{
    if (realOut()) return { SOAPY_SDR_S16, SOAPY_SDR_F32 };
    return { SOAPY_SDR_CF32, SOAPY_SDR_CS16, SOAPY_SDR_CS8 };
}

std::string SoapyMiriSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
    if (baseband)
    {
        fullScale = 1.0;
        return SOAPY_SDR_CF32;
    }
    fullScale = 32768;
    return realOut() ? SOAPY_SDR_S16 : SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyMiriSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList list;
    SoapySDR::ArgInfo a;

    a.key = "buffers";
    a.name = "Ring buffers";
    a.description = "USB buffers the module holds for the application.";
    a.type = SoapySDR::ArgInfo::INT;
    a.value = std::to_string(SOAPY_MIRISDR_SLOTS);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "transfers";
    a.name = "USB transfers";
    a.description = "Transfers in flight, 0 for the library's default.";
    a.type = SoapySDR::ArgInfo::INT;
    a.value = "0";
    list.push_back(a);

    return list;
}

SoapySDR::Stream *SoapyMiriSDR::setupStream(const int direction, const std::string &format,
                                            const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{
    ReportScope report_(*this);
    size_t slots = SOAPY_MIRISDR_SLOTS;

    if (direction != SOAPY_SDR_RX) throw std::runtime_error("mirisdr: receive only");
    if (rxThread.joinable()) throw std::runtime_error("mirisdr: a stream is already set up");

    /* the channels: one tuner, or on the RSPduo both, each a low IF on its converter */
    {
        std::lock_guard<std::recursive_mutex> lock(devMutex);
        const size_t nch = getNumChannels(SOAPY_SDR_RX);
        int tuners = 0;

        if (channels.size() > nch) throw std::runtime_error("mirisdr: at most " + std::to_string(nch) + " channels");
        chanOrder[0] = channels.empty() ? 0 : (int) channels[0];
        chanOrder[1] = channels.size() > 1 ? (int) channels[1] : -1;
        for (int i = 0; i < 2; i++)
        {
            if (chanOrder[i] < 0) continue;
            if ((size_t) chanOrder[i] >= nch || (tuners & (1 << chanOrder[i])))
                throw std::runtime_error("mirisdr: channel " + std::to_string(chanOrder[i]) + " not available");
            tuners |= 1 << chanOrder[i];
        }
        if (streamTuners() != tuners)
        {
            const mirisdr_stream_config_t was = streamCfg;
            mirisdr_stream_config_t lib;

            /* what the library took, not what was asked: a refused one is put back */
            streamCfg.tuners = tuners == 1 ? 0 : tuners;
            if (!configure(wantRate) || (mirisdr_get_stream(dev, &lib, NULL) < 0) ||
                ((lib.tuners ? lib.tuners : 1) != tuners))
            {
                streamCfg = was;
                configure(wantRate);
                throw std::runtime_error("mirisdr: these channels together refused");
            }
        }
    }

    if (format == SOAPY_SDR_CS16) outFormat = OUT_CS16;
    else if (format == SOAPY_SDR_CF32) outFormat = OUT_CF32;
    else if (format == SOAPY_SDR_CS8) outFormat = OUT_CS8;
    else if (format == SOAPY_SDR_S16) outFormat = OUT_S16;
    else if (format == SOAPY_SDR_F32) outFormat = OUT_F32;
    else throw std::runtime_error("mirisdr: format " + format + " not supported, use CF32, CS16 or CS8 (S16 or F32 "
                                  "real, from one converter without baseband)");

    if (args.count("buffers")) slots = std::max(4, std::stoi(args.at("buffers")));
    asyncBuffers = args.count("transfers") ? (uint32_t) std::max(0, std::stoi(args.at("transfers"))) : 0;

    ring.assign(slots, Slot());
    ringHead = ringTail = ringCount = 0;

    return (SoapySDR::Stream *) this;
}

void SoapyMiriSDR::closeStream(SoapySDR::Stream *stream)
{
    deactivateStream(stream, 0, 0);

    std::lock_guard<std::mutex> rl(ringMutex);
    ring.clear();
    ringHead = ringTail = ringCount = 0;
    slotPos = gapPos = 0;
}

static inline int16_t clip16(float v)
{
    v = v * 32768.0f;
    return (int16_t) (v >= 32767.0f ? 32767 : v <= -32768.0f ? -32768 : std::lrint(v));
}

static inline int8_t clip8(float v)
{
    v = v * 128.0f;
    return (int8_t) (v >= 127.0f ? 127 : v <= -128.0f ? -128 : std::lrint(v));
}

/* n samples of a slot's kind into the stream format. A slot of the other kind (real
   or complex) than the format, after a setting changed the stream, gives zeros for
   the part it has not */
static void convert(int kind, const uint8_t *src, int out, void *dst, size_t n)
{
    size_t k;

    switch (kind)
    {
    case SoapyMiriSDR::K_CF32: {
        const float *in = (const float *) src;
        if (out == OUT_CF32) std::memcpy(dst, in, n * 8);
        else if (out == OUT_CS16) for (k = 0; k < 2 * n; k++) ((int16_t *) dst)[k] = clip16(in[k]);
        else if (out == OUT_CS8) for (k = 0; k < 2 * n; k++) ((int8_t *) dst)[k] = clip8(in[k]);
        else if (out == OUT_S16) for (k = 0; k < n; k++) ((int16_t *) dst)[k] = clip16(in[2 * k]);
        else for (k = 0; k < n; k++) ((float *) dst)[k] = in[2 * k];
        break;
    }
    case SoapyMiriSDR::K_CS16: {
        const int16_t *in = (const int16_t *) src;
        if (out == OUT_CS16) std::memcpy(dst, in, n * 4);
        else if (out == OUT_CF32) for (k = 0; k < 2 * n; k++) ((float *) dst)[k] = in[k] * (1.0f / 32768.0f);
        else if (out == OUT_CS8) for (k = 0; k < 2 * n; k++) ((int8_t *) dst)[k] = (int8_t) (in[k] >> 8);
        else if (out == OUT_S16) for (k = 0; k < n; k++) ((int16_t *) dst)[k] = in[2 * k];
        else for (k = 0; k < n; k++) ((float *) dst)[k] = in[2 * k] * (1.0f / 32768.0f);
        break;
    }
    case SoapyMiriSDR::K_CS8: {
        const int8_t *in = (const int8_t *) src;
        if (out == OUT_CS8) std::memcpy(dst, in, n * 2);
        else if (out == OUT_CF32) for (k = 0; k < 2 * n; k++) ((float *) dst)[k] = in[k] * (1.0f / 128.0f);
        else if (out == OUT_CS16) for (k = 0; k < 2 * n; k++) ((int16_t *) dst)[k] = (int16_t) (in[k] * 256);
        else if (out == OUT_S16) for (k = 0; k < n; k++) ((int16_t *) dst)[k] = (int16_t) (in[2 * k] * 256);
        else for (k = 0; k < n; k++) ((float *) dst)[k] = in[2 * k] * (1.0f / 128.0f);
        break;
    }
    default: {
        const int16_t *in = (const int16_t *) src;
        if (out == OUT_S16) std::memcpy(dst, in, n * 2);
        else if (out == OUT_F32) for (k = 0; k < n; k++) ((float *) dst)[k] = in[k] * (1.0f / 32768.0f);
        else if (out == OUT_CF32) for (k = 0; k < n; k++) { ((float *) dst)[2 * k] = in[k] * (1.0f / 32768.0f); ((float *) dst)[2 * k + 1] = 0; }
        else if (out == OUT_CS16) for (k = 0; k < n; k++) { ((int16_t *) dst)[2 * k] = in[k]; ((int16_t *) dst)[2 * k + 1] = 0; }
        else for (k = 0; k < n; k++) { ((int8_t *) dst)[2 * k] = (int8_t) (in[k] >> 8); ((int8_t *) dst)[2 * k + 1] = 0; }
        break;
    }
    }
}

/* a typical buffer from the library: one transfer of up to 24 blocks */
size_t SoapyMiriSDR::getStreamMTU(SoapySDR::Stream *stream) const
{
    return 24 * 504;
}

/* nothing may be thrown through the library's C frames: out of memory ends the stream */
static void rxCallbackC(unsigned char *buf, uint32_t len, void *ctx)
{
    SoapyMiriSDR *d = (SoapyMiriSDR *) ctx;

    try { d->rxCallback(buf, len); }
    catch (...) { d->rxFailed(); }
}

int SoapyMiriSDR::activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs,
                                 const size_t numElems)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    if (rxThread.joinable()) return 0;

    {
        std::lock_guard<std::mutex> rl(ringMutex);
        ringHead = ringTail = ringCount = 0;
        dropPending = false;
        pairOpen = false;
        slotPos = gapPos = 0;
    }
    if (ring.empty()) return SOAPY_SDR_STREAM_ERROR;    /* closed, or never set up */

    /* the library counts from 0 again: the time carries on from where the last stream
       ended, setHardwareTime() with it */
    timeOffsetNs = getHardwareTime("");
    timeBaseNs = 0;
    timeBaseIndex = 0;
    timeBaseRate = 0;
    lastIndex = 0;
    lastRate = 0;
    stopping = false;
    rxDone = false;
    rxEnded = false;
    rxResult = 0;

    mirisdr_reset_buffer(dev);
    rxThread = std::thread([this]() {
        /* a stop before the library runs: the callback catches one after */
        /* an error the callback recorded stays */
        if (!stopping)
        {
            int r = mirisdr_read_async(dev, rxCallbackC, this, asyncBuffers, 0);
            if (r < 0) rxResult = r;
        }
        /* under the ring's lock, so a reader between its check and its wait hears it */
        std::lock_guard<std::mutex> rl(ringMutex);
        rxEnded = !stopping;
        rxDone = true;
        ringCond.notify_all();
    });

    return 0;
}

int SoapyMiriSDR::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    if (!rxThread.joinable()) return 0;

    stopping = true;
    mirisdr_cancel_async(dev);
    rxThread.join();

    return 0;
}

void SoapyMiriSDR::rxCallback(unsigned char *buf, uint32_t len)
{
    mirisdr_buffer_info_t info;
    uint32_t i;
    int bytes, kind;

    if (stopping)
    {
        mirisdr_cancel_async(dev);
        return;
    }

    if (mirisdr_get_buffer_info(dev, &info) < 0) return;

    /* what the library hands over: baseband float, or as it comes */
    if (info.type == MIRISDR_SAMPLE_F32) { kind = K_CF32; bytes = 8; }
    else if (info.adc != MIRISDR_IQ_BOTH) { kind = K_S16; bytes = 2; }
    else if (info.type == MIRISDR_SAMPLE_S8) { kind = K_CS8; bytes = 2; }
    else { kind = K_CS16; bytes = 4; }

    {
        std::lock_guard<std::mutex> rl(ringMutex);
        const bool dual = streamTuners() == 3;

        if (ring.empty()) return;

        /* two tuners: tuner 1's buffer, for the same span, completes the slot tuner 0's
           opened (one dropped for a full ring takes its partner with it) */
        if (dual && info.tuner == 1)
        {
            if (!pairOpen) return;
            Slot &s = ring[ringTail];
            if (len / bytes < s.n) s.n = len / bytes;
            s.data2.assign(buf, buf + s.n * bytes);

            /* the shorter of the two, and the gaps of either within it: one lost in tuner
               1's half alone is lost for the pair too */
            for (i = 0; i < info.gaps_len && i < MIRISDR_GAPS_MAX; i++)
            {
                if (info.gaps[i].filled >= info.gaps[i].samples) continue;
                auto at = std::find_if(s.gaps.begin(), s.gaps.end(),
                                       [&](const Gap &g) { return g.offset == info.gaps[i].offset; });
                if (at != s.gaps.end()) { at->lost = true; continue; }
                Gap g;
                g.offset = info.gaps[i].offset;
                g.missing = info.gaps[i].samples - info.gaps[i].filled;
                g.lost = true;
                s.gaps.push_back(g);
            }
            s.gaps.erase(std::remove_if(s.gaps.begin(), s.gaps.end(), [&](const Gap &g) { return g.offset > s.n; }),
                         s.gaps.end());
            std::sort(s.gaps.begin(), s.gaps.end(), [](const Gap &a, const Gap &b) { return a.offset < b.offset; });
            pairOpen = false;
            ringTail = (ringTail + 1) % ring.size();
            ringCount++;
            ringCond.notify_one();
            return;
        }

        if (ringCount == ring.size())
        {
            dropPending = true;
            pairOpen = false;
            return;
        }

        Slot &s = ring[ringTail];
        s.data.assign(buf, buf + len);
        s.n = len / bytes;
        s.kind = kind;
        s.bytes = bytes;
        s.index = info.index;
        s.rate = info.rate;
        s.dropped = dropPending;
        dropPending = false;

        s.gaps.clear();
        for (i = 0; i < info.gaps_len && i < MIRISDR_GAPS_MAX; i++)
        {
            Gap g;
            g.offset = info.gaps[i].offset;
            g.missing = info.gaps[i].samples - info.gaps[i].filled;
            g.lost = info.gaps[i].filled < info.gaps[i].samples;
            s.gaps.push_back(g);
        }

        /* tuner 0's half: held until tuner 1's comes */
        if (dual)
        {
            pairOpen = true;
            return;
        }

        ringTail = (ringTail + 1) % ring.size();
        ringCount++;
    }
    ringCond.notify_one();
}

/* the callback could not go on (out of memory): the stream ends with an error */
void SoapyMiriSDR::rxFailed(void)
{
    rxResult = -1;
    mirisdr_cancel_async(dev);
}

int SoapyMiriSDR::readStream(SoapySDR::Stream *stream, void * const *buffs, const size_t numElems,
                             int &flags, long long &timeNs, const long timeoutUs)
{
    size_t n, end, k;
    uint64_t index;

    flags = 0;

    {
        std::unique_lock<std::mutex> rl(ringMutex);

        if (ring.empty()) return SOAPY_SDR_STREAM_ERROR;    /* closed */

        if (!ringCount)
        {
            /* stopped, the whole timeout: a reader looping on it does not spin */
            ringCond.wait_for(rl, std::chrono::microseconds(timeoutUs),
                              [this] { return ringCount != 0 || rxEnded || (rxDone && rxResult < 0); });
            if (ring.empty()) return SOAPY_SDR_STREAM_ERROR;
            /* a stream that ended without a deactivate gives no more: an error, not a wait */
            if (!ringCount) return rxEnded || (rxDone && rxResult < 0) ? SOAPY_SDR_STREAM_ERROR : SOAPY_SDR_TIMEOUT;
        }
    }

    /* the slot at the head stays put while ringCount holds it */
    Slot &s = ring[ringHead];

    if (s.dropped)
    {
        s.dropped = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
    }

    /* a lost gap where we are: report it once, then go on past it */
    while (gapPos < s.gaps.size() && s.gaps[gapPos].offset <= slotPos)
    {
        Gap &g = s.gaps[gapPos++];

        if (g.lost)
        {
            g.lost = false;
            SoapySDR::log(SOAPY_SDR_SSI, "O");
            return SOAPY_SDR_OVERFLOW;
        }
    }

    /* up to the next gap that is lost, within the slot */
    end = s.n;
    for (k = gapPos; k < s.gaps.size(); k++)
        if (s.gaps[k].lost) { end = std::min<size_t>(s.gaps[k].offset, s.n); break; }
    n = std::min(numElems, end - slotPos);

    index = s.index + slotPos;
    for (k = 0; k < s.gaps.size(); k++)
        if (s.gaps[k].offset > 0 && s.gaps[k].offset <= slotPos) index += s.gaps[k].missing;
    timeRebase(index, s.rate);
    timeNs = indexToNs(index, s.rate);
    flags |= SOAPY_SDR_HAS_TIME;
    lastIndex = (long long) (index + n);
    lastRate = s.rate;

    /* the channels in the order asked: tuner 0's samples, and with two tuners tuner 1's */
    for (k = 0; k < 2 && chanOrder[k] >= 0; k++)
    {
        const std::vector<uint8_t> &d = (streamTuners() == 3 && chanOrder[k] == 1) ? s.data2 : s.data;
        convert(s.kind, d.data() + slotPos * s.bytes, outFormat, buffs[k], n);
    }

    slotPos += n;
    if (slotPos >= s.n)
    {
        std::lock_guard<std::mutex> rl(ringMutex);
        ringHead = (ringHead + 1) % ring.size();
        ringCount--;
        slotPos = gapPos = 0;
    }

    return (int) n;
}
