/*
 * SoapySDR module for libmirisdr: finding and making devices.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "SoapyMiriSDR.hpp"

#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

static std::vector<SoapySDR::Kwargs> findMiriSDR(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;
    const char *want = args.count("serial") ? args.at("serial").c_str() : nullptr;
    const char *at = args.count("usb_port") ? args.at("usb_port").c_str() : nullptr;
    uint32_t i, n = mirisdr_get_device_count();
    /* by the library's rules, which the open matches again */
    int only = (want || at) ? mirisdr_get_index_by_serial_port(want, at) : -1;

    if ((want || at) && (only < 0)) return results;

    for (i = 0; i < n; i++)
    {
        char manufact[256] = "", product[256] = "", serial[256] = "", port[256] = "";
        SoapySDR::Kwargs dev;

        if ((only >= 0) && (i != (uint32_t) only)) continue;
        /* the serial is the system's copy, so a receiver in use elsewhere is not disturbed */
        if (mirisdr_get_device_usb_strings(i, manufact, product, nullptr) != 0) continue;
        if (mirisdr_get_device_port(i, port, sizeof port, serial, sizeof serial) != 0) continue;

        dev["driver"] = SOAPY_MIRISDR_DRIVER;
        /* asked for by serial or port, those: the device is opened by them, matched in the
           list it is opened from, where the index found here may by then be another's */
        if (want || at)
        {
            if (want) dev["serial"] = want;
            if (at) dev["usb_port"] = at;
        }
        else
        {
            /* no serial until it declares one, as from ROM it does not */
            if (*serial) dev["serial"] = serial;
            dev["usb_port"] = port;
        }
        dev["manufacturer"] = manufact;
        dev["product"] = product;
        dev["label"] = std::string(product) + " :: " + (*serial ? serial : port);
        results.push_back(dev);
    }

    return results;
}

static SoapySDR::Device *makeMiriSDR(const SoapySDR::Kwargs &args)
{
    return new SoapyMiriSDR(args);
}

static SoapySDR::Registry registerMiriSDR(SOAPY_MIRISDR_DRIVER, &findMiriSDR, &makeMiriSDR, SOAPY_SDR_ABI_VERSION);
