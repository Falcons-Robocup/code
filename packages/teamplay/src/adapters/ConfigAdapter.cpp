// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ConfigAdapter.cpp
 *
 *  Created on: 28 Sep 2019
 *      Author: Coen Tempelaars
 */

#include "int/adapters/ConfigAdapter.hpp"

#include "tracing.hpp"
#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"

#include "int/stores/configurationStore.hpp"

boost::mutex g_mutex_tp;


ConfigAdapter::ConfigAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, getTeamChar());
    _updateThread = boost::thread(&ConfigAdapter::loopUpdate, this);
    _config = {};
    TRACE("<");
}

ConfigAdapter::~ConfigAdapter()
{
}

void ConfigAdapter::loadYAML(std::string const &yamlFile)
{
    // first do a put to make sure the structure of the data is in place
    // (this is read by the python script and used when mapping yaml to)
    _rtdb->put(CONFIG_TEAMPLAY, &_config);

    // call a python script which can load YAML and put on top of this data
    std::string command = std::string("python3 ") + pathToCodeRepo() + "/scripts/loadYAML.py -s -a " + std::to_string(_myRobotId)
        + " -k " + CONFIG_TEAMPLAY + " -p " + _rtdb->getPath() + " " + yamlFile;
    // NOTE: use option -s to disable strict key checking, because in a nominal RTDB struct the heightmap
    // factors map is empty, so python tool will give a warning about a missing heightmap factor, e.g.
    //     WARNING: missing key heightmaps.factors.MOVE_TO_FREE_SPOT in rtdb dict
    tprintf("command: %s", command.c_str());
    int r = system(command.c_str());
    if (r != 0)
    {
        TRACE_ERROR("something went wrong while reading CONFIG_TEAMPLAY");
    }
    else
    {
        r = _rtdb->get(CONFIG_TEAMPLAY, &_config);
        if (r != 0)
        {
            TRACE_ERROR("something went wrong while caching CONFIG_TEAMPLAY");
        }
        else
        {
            teamplay::configurationStore::getConfiguration().update(_config);
        }
    }
}

bool ConfigAdapter::get()
{
    boost::mutex::scoped_lock l(g_mutex_tp);
    if (_rtdb != NULL)
    {
        int r = _rtdb->get(CONFIG_TEAMPLAY, &_config);
        return (r == RTDB2_SUCCESS);
    }
    return false;
}

void ConfigAdapter::loopUpdate()
{
    sleep(1); // give loadYAML some time
    update();
    while (true)
    {
        _rtdb->waitForPut(CONFIG_TEAMPLAY);
        tprintf("teamplay configuration was touched");
        update();
    }
}

void ConfigAdapter::update()
{
    // serialize current config, so we can do a string-compare
    std::string currentConfigSerialized;
    RtDB2Serializer::serialize(_config, currentConfigSerialized);
    // get new config
    bool success = get();
    if (!success)
    {
        return;
    }
    // serialize new config
    std::string newConfigSerialized;
    RtDB2Serializer::serialize(_config, newConfigSerialized);
    // report change and only notify PP in case of change
    if (currentConfigSerialized != newConfigSerialized)
    {
        teamplay::configurationStore::getConfiguration().update(_config);
        tprintf("teamplay configuration changed");
    }
}

