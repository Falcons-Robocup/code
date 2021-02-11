// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBConfigAdapter.cpp
 *
 *  Created on: Aug 17, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBconfigAdapter.hpp"
#include "int/RTDBaccess.hpp"

#include <stdexcept>
#include <thread>

#include "FalconsRtDB2.hpp"
#include "tracing.hpp"

RTDBConfigAdapter::RTDBConfigAdapter()
{
    // Fetch configuration from yaml file
    //std::string configFile = determineConfig("simulation");
    _configAdapter = new ConfigRTDBAdapter<T_CONFIG_SIMULATION>(CONFIG_SIMULATION);
    _configAdapter->load();
}

RTDBConfigAdapter::~RTDBConfigAdapter()
{
    delete _configAdapter;
}

std::string RTDBConfigAdapter::getArbiter() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);
    tprintf("get CONFIG_SIMULATION arbiter=%s", config.arbiter.c_str());
    return config.arbiter;
}

int RTDBConfigAdapter::getSize(const TeamID teamID) const
{
    TRACE_FUNCTION("");

    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);

    int teamSize = 0;
    if (teamID == TeamID::A)
    {
        teamSize = config.sizeTeamA;
        tprintf("get CONFIG_SIMULATION teamID=A teamSize=%d", teamSize);
    }
    else
    {
        teamSize = config.sizeTeamB;
        tprintf("get CONFIG_SIMULATION teamID=B teamSize=%d", teamSize);
    }

    return teamSize;
}

int RTDBConfigAdapter::getTickFrequency() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);
    //tprintf("get CONFIG_SIMULATION tickFrequency=%d", config.tick_frequency);
    return config.tick_frequency;
}

int RTDBConfigAdapter::getStepSizeMs() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);
    tprintf("get CONFIG_SIMULATION stepSize=%d", config.tick_stepsize_ms);
    return config.tick_stepsize_ms;
}
