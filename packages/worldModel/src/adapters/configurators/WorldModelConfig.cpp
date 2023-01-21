// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * WorldModelConfig.cpp
 *
 *  Created on: July 11 2020
 */

#include "int/adapters/configurators/WorldModelConfig.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"


WorldModelConfig::WorldModelConfig()
{
    // Fetch configuration from yaml file
    std::string configFile = determineConfig("worldModel");
    _configAdapter = new ConfigRTDBAdapter<T_CONFIG_WORLDMODEL>(CONFIG_WORLDMODEL);
    _configAdapter->setConfigUpdateCallback(boost::bind(&WorldModelConfig::updateConfiguration, this));
    _configAdapter->loadYAML(configFile);
}

WorldModelConfig::~WorldModelConfig()
{    
    delete _configAdapter;
}

void WorldModelConfig::updateConfiguration()
{
    _configAdapter->get(_configData);
}


T_CONFIG_WORLDMODEL WorldModelConfig::getConfiguration() const
{
    return _configData;
}

