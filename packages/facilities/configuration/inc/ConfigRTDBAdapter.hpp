// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ConfigRTDBAdapter.hpp
 *
 *  Created on: December 2019
 *      Author: Jan Feitsma
 */

#ifndef CONFIGRTDBADAPTER_HPP_
#define CONFIGRTDBADAPTER_HPP_

#include "FalconsRTDB.hpp"
#include "ConfigInterface.hpp"

#include <boost/thread/thread.hpp>

template <typename T>
class ConfigRTDBAdapter : public ConfigInterface<T>
{
public:
    ConfigRTDBAdapter(std::string const &configKey, bool testmode = false);
    ~ConfigRTDBAdapter();

    ///// To initialize and listen for configuration updates, either call loadYAML() or call load()
    
    // load() will read the value from RtDB in memory, and listen for configuration updates
    void load();

    // loadYAML will initialize from a YAML file, write it to RtDB, keep it in memory, and listen for configuration updates
    void loadYAML(std::string const &yamlFile, bool strict = true);

    void setConfigUpdateCallback( std::function<void()> func );

    bool get(T &config);

protected:
    T _config;
    std::string _configKey;
    FalconsRTDB *_rtdb = NULL;

private:
    int _myRobotId = 0;
    bool _updated = false;
    bool _verbose = false;
    bool _testmode = false;
    bool _firstChange = true; // prevent init spam
    bool _terminateRequested = false; // used to break out of the while(true) when terminating
    boost::thread _updateThread;
    boost::mutex _configuration_mutex;
    std::function<void()> _callbackFunc; // this function ptr allows an update to the client when the configuration changes

    void startLoopUpdate();
    void loopUpdate();
    void update(bool allowOldData);
    void terminate();

};

// template implementations
// note: quite some headers are dealt here ...
// this seems to be the best solution out of several mentioned here:
// https://www.codeproject.com/Articles/48575/How-to-define-a-template-class-in-a-h-file-and-imp
#include "ConfigRTDBAdapterImpl.hpp"

#endif

