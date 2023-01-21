// Copyright 2019-2021 Coen Tempelaars (Falcons)
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

#include "FalconsRTDB.hpp"
#include "tracing.hpp"

RTDBConfigAdapter::RTDBConfigAdapter()
{
    // Fetch configuration from yaml file
    std::string configFile = determineConfig("execution");
    _configAdapter = new ConfigRTDBAdapter<T_CONFIG_EXECUTION>(CONFIG_EXECUTION);
    _configAdapter->loadYAML(configFile);
}

RTDBConfigAdapter::~RTDBConfigAdapter()
{
    delete _configAdapter;
}

float RTDBConfigAdapter::getTickFrequency() const
{
    TRACE_FUNCTION("");
    T_CONFIG_EXECUTION config;
    _configAdapter->get(config);
    return config.frequency;
}

float RTDBConfigAdapter::getSimulationSpeedupFactor() const
{
    TRACE_FUNCTION("");
    T_CONFIG_EXECUTION config;
    _configAdapter->get(config);
    return config.simulationSpeedupFactor;
}

std::string RTDBConfigAdapter::getTickFinishRtdbKey() const
{
    TRACE_FUNCTION("");
    T_CONFIG_EXECUTION config;
    _configAdapter->get(config);
    return config.tickFinishRtdbKey;
}