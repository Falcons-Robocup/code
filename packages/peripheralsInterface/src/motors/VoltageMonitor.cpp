// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VoltageMonitor.cpp
 *
 *  Created on: Apr 25, 2017
 *      Author: Jan Feitsma
 */

#include <algorithm>

#include "int/motors/VoltageMonitor.hpp"
#include "cDiagnostics.hpp"

VoltageMonitor::VoltageMonitor()
{
    _buffer.push_back(26.0);
}

VoltageMonitor::~VoltageMonitor()
{
}

void VoltageMonitor::feed(float voltage)
{
    _buffer.push_back(voltage);

    // if buffer too large, remove element at the front
    if (_buffer.size() > BUFFERSIZE)
    {
        _buffer.pop_front();
    }
}

float VoltageMonitor::get()
{
    // crude filtering: simply take the max over buffer
    auto it = std::max_element(std::begin(_buffer), std::end(_buffer));

    if (it != std::end(_buffer))
    {
        return *it;
    }
    else
    {
        return 0.0;
    }
}

void VoltageMonitor::check()
{
    float voltage = get();
    // generate warning or error events every 0.1 volt??
    if (voltage < ERROR_THRESHOLD)
    {
        TRACE_ERROR_TIMEOUT(EVENT_TIMEOUT, "voltage dropped to %.1fV", voltage);
    }
    else if (voltage < WARNING_THRESHOLD)
    {
        TRACE_WARNING_TIMEOUT(EVENT_TIMEOUT, "voltage dropped to %.1fV", voltage);
    }
}

