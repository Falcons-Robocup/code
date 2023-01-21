// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RtdbAdapterControlOverride.hpp
 *
 *  Created on: April, 2019
 *      Author: Jan Feitsma
 */

#ifndef RTDBADAPTERCONTROLOVERRIDE_HPP_
#define RTDBADAPTERCONTROLOVERRIDE_HPP_

#include "FalconsRTDB.hpp"


class RtdbAdapterControlOverride
{
public:
    RtdbAdapterControlOverride();
    ~RtdbAdapterControlOverride();

    bool getOverrideState(T_TP_OVERRIDE_STATE &overrideState); // return success
    void setOverrideResult(T_TP_OVERRIDE_RESULT const &overrideResult);
    
private:
    RtDB2 *_rtdb;
    int _myRobotId;

};

#endif

