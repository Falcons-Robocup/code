// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * WorldModelConfig.hpp
 *
 *  Created on: July 11 2020
 */

#ifndef WORLDMODELCONFIG_HPP_
#define WORLDMODELCONFIG_HPP_

#include "FalconsRtDB2.hpp"
#include "ConfigRTDBAdapter.hpp"


class WorldModelConfig
{
public:
    WorldModelConfig();
    ~WorldModelConfig();

    T_CONFIG_WORLDMODEL getConfiguration() const;

private:
    T_CONFIG_WORLDMODEL _configData; // cache, because getConfiguration is called VERY often from within WorldModel
    ConfigRTDBAdapter<T_CONFIG_WORLDMODEL>* _configAdapter;
    void updateConfiguration();
};


#endif /* WORLDMODELCONFIG_HPP_ */
