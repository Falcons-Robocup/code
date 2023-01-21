// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ConfigAdapter.hpp
 *
 *  Created on: 28 Sep 2019
 *      Author: Coen Tempelaars
 */

#ifndef CONFIGADAPTER_TEAMPLAY_HPP_
#define CONFIGADAPTER_TEAMPLAY_HPP_

#include <boost/thread/thread.hpp>

#include "FalconsRTDB.hpp"

class ConfigAdapter
{
public:
    ConfigAdapter();
    ~ConfigAdapter();

    void loadYAML(std::string const &yamlFile);

private:
    FalconsRTDB *_rtdb = NULL;
    int _myRobotId = 0;
    ConfigTeamplay _config;

    boost::thread _updateThread;
    void loopUpdate();
    void update();
    bool get();
};

#endif
