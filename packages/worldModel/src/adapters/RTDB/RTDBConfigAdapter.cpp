// Copyright 2019-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBConfigAdapter.cpp
 *
 *  Created on: August 2019
 *      Author: Jan Feitsma
 */

#include "tracing.hpp"
#include "ext/RTDBConfigAdapter.hpp"
#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"

boost::mutex g_mutex_wm;


RTDBConfigAdapter::RTDBConfigAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, getTeamChar());
    _config = {};
    std::string configFile = determineConfig("worldModelSyncConfig");
    loadYAML(configFile);
    _updateThread = boost::thread(&RTDBConfigAdapter::loopUpdate, this);
    TRACE("<");
}

RTDBConfigAdapter::~RTDBConfigAdapter()
{
}

void RTDBConfigAdapter::loadYAML(std::string const &yamlFile)
{
    boost::mutex::scoped_lock l(g_mutex_wm);
    // first do a put to make sure the structure of the data is in place
    // (this is read by the python script and used when mapping yaml to)
    _rtdb->put(_configKey, &_config);

    // call a python script which can load YAML and put on top of this data
    std::string command = std::string("python3 ") + pathToScripts() + "/loadYAML.py -a " + std::to_string(_myRobotId)
        + " -k " + _configKey + " -p " + _rtdb->getPath() + " " + yamlFile;
    tprintf("command: %s", command.c_str());
    int r = system(command.c_str());
    if (r != 0)
    {
        TRACE_ERROR("something went wrong reading while initializing %s", _configKey.c_str());
    }
}

void RTDBConfigAdapter::get(T_CONFIG_WORLDMODELSYNC &config)
{
    boost::mutex::scoped_lock l(g_mutex_wm);
    config = _config;
}

void RTDBConfigAdapter::loopUpdate()
{
    INIT_TRACE_THREAD("RTDBConfigAdapter");

    //sleep(1); // give loadYAML some time
    update();
    while (true)
    {
        _rtdb->waitForPut(_configKey);
        tprintf("config was touched");
        update();
    }
}

void RTDBConfigAdapter::update()
{
    boost::mutex::scoped_lock l(g_mutex_wm);
    // serialize current config, so we can do a string-compare
    std::string currentConfigSerialized;
    RtDB2Serializer::serialize(_config, currentConfigSerialized);
    // get new config
    bool success = false;
    if (_rtdb != NULL)
    {
        int r = _rtdb->get(_configKey, &_config);
        success = (r == RTDB2_SUCCESS);
    }
    if (!success)
    {
        return;
    }
    // serialize new config
    std::string newConfigSerialized;
    RtDB2Serializer::serialize(_config, newConfigSerialized);
    // report change
    if (currentConfigSerialized != newConfigSerialized)
    {
        tprintf("configuration changed");
        // callbacks might be triggered here
    }
}

