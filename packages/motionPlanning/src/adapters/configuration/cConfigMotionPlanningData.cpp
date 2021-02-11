// Copyright 2018-2020 Shepherd Takawira (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cConfigMotionPlanningData.cpp
 *
 *  Created on: April 19 2018
 */

#include "int/adapters/configuration/cConfigMotionPlanningData.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"


cConfigMotionPlanningData::cConfigMotionPlanningData()
{
	// Fetch configuration from yaml file
	std::string configFile = determineConfig("motionPlanning");
	_configAdapter = new ConfigRTDBAdapter<ConfigMotionPlanning>(CONFIG_MOTIONPLANNING);
	_configAdapter->loadYAML(configFile);
}

cConfigMotionPlanningData::~cConfigMotionPlanningData()
{
	delete _configAdapter;
}

ConfigMotionPlanning cConfigMotionPlanningData::getConfiguration()
{   
    ConfigMotionPlanning result;
    _configAdapter->get(result);
    return result;
}

