// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballHandlersMotorSetpoint.hpp
 *
 *  Created on: Dec 8, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef BALLHANDLERSMOTORSETPOINT_HPP_
#define BALLHANDLERSMOTORSETPOINT_HPP_

#include "RtDB2.h" // required for serialization

#include "ballHandlersMotorData.hpp"


struct ballHandlersMotorSetpoint
{
    bool                   enabled;
    ballHandlersMotorData  bhMotorData;
    
    SERIALIZE_DATA_FIXED(enabled, bhMotorData);
};

#endif

