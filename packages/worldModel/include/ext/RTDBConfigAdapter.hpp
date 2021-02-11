// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBConfigAdapter.hpp
 *
 *  Created on: August 2019
 *      Author: Jan Feitsma
 */

#ifndef CONFIGADAPTER_WORLDMODEL_HPP_
#define CONFIGADAPTER_WORLDMODEL_HPP_

#include "FalconsRtDB2.hpp" // need configWorldModel from sharedTypes

#include <boost/thread/thread.hpp>

class RTDBConfigAdapter
{
public:
    RTDBConfigAdapter();
    ~RTDBConfigAdapter();

    void get(T_CONFIG_WORLDMODELSYNC &config);

private:
    RtDB2 *_rtdb = NULL;
    int _myRobotId = 0;
    T_CONFIG_WORLDMODELSYNC _config;
    std::string _configKey = CONFIG_WORLDMODELSYNC;

    boost::thread _updateThread;
    void loopUpdate();
    void update();
    void loadYAML(std::string const &yamlFile);
};

#endif

