// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballHandlersMotorData.hpp
 *
 *  Created on: Jan 05, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef BALLHANDLERSMOTORDATA_HPP_
#define BALLHANDLERSMOTORDATA_HPP_

#include "RtDB2.h" // required for serialization


struct ballHandlersMotorData
{
    float              angleLeft;
    float              angleRight;
    float              velocityLeft;
    float              velocityRight;
    
    SERIALIZE_DATA_FIXED(angleLeft, angleRight, velocityLeft, velocityRight);
};

#endif

