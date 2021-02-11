// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * kickerSetpoint.hpp
 *
 *  Created on: Dec 08, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef KICKERSETPOINT_HPP_
#define KICKERSETPOINT_HPP_


#include "RtDB2.h" // required for serialization


enum class kickerSetpointTypeEnum
{
    HOME,
    SET_HEIGHT,
    SHOOT
};

SERIALIZE_ENUM(kickerSetpointTypeEnum);

struct kickerSetpoint
{
    kickerSetpointTypeEnum  kickerSetpointType;
    float                   kickerHeight;
    float                   kickerPower;
    
    SERIALIZE_DATA_FIXED(kickerSetpointType, kickerHeight, kickerPower);
};

#endif

