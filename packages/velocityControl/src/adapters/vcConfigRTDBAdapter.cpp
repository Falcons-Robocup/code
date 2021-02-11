// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcConfigRTDBAdapter.cpp
 *
 *  Created on: Nov 09, 2020
 *      Author: Erik Kouters
 */

#include <yaml-cpp/yaml.h>

#include "int/adapters/vcConfigRTDBAdapter.hpp"
#include "tracing.hpp"

vcConfigRTDBAdapter::vcConfigRTDBAdapter(std::string const &configKey)
    : ConfigRTDBAdapter<ConfigVelocityControl>(configKey)
{
    TRACE_FUNCTION("");
}

vcConfigRTDBAdapter::~vcConfigRTDBAdapter()
{
    TRACE_FUNCTION("");
}

void vcConfigRTDBAdapter::loadYAML(std::string const &yamlFile)
{
    // Load the YAML
    YAML::Node yamlNode = YAML::LoadFile(yamlFile);
    YAML::Node motionTypes = yamlNode["motionTypes"];

    std::stringstream str;

    // Get all motionTypes defined in the YAML, and add to _config
    YAML::const_iterator it;
    for(it = motionTypes.begin(); it != motionTypes.end(); ++it)
    {
        str << it->first.as<std::string>() << ", ";

        _config.motionTypes.insert( std::make_pair(it->first.as<std::string>(), MotionTypeConfig()) );
    }

    // Follow the default loadYAML procedure, using the populated _config
    ConfigRTDBAdapter::loadYAML(yamlFile, false);

    // Read the config from RTDB to get the values for the "NORMAL" configuration
    _rtdb->get(_configKey, &_config);

    // Overwrite all motionType configurations with the "NORMAL" configuration
    MotionTypeConfig normalConfig = _config.motionTypes.at("NORMAL"); 
    for (auto i : _config.motionTypes)
    {
        _config.motionTypes.at(i.first) = normalConfig;
    } 

    // Redo the default loadYAML procedure, now with _config filled with all motionTypes with the same values as the "NORMAL" configuration
    // This loadYAML will only write the lines in the yaml (e.g., maxVelX for SLOW), and other values will stay intact from the "NORMAL" configuration
    ConfigRTDBAdapter::loadYAML(yamlFile, false);
}

