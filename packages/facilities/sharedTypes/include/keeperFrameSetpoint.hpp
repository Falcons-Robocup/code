// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * keeperFrameSetpoint.hpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Jan Feitsma
 */

#ifndef KEEPERFRAMESETPOINT_HPP_
#define KEEPERFRAMESETPOINT_HPP_

#include "RtDB2.h" // required for serialization


enum class keeperFrameSetpointEnum
{
    UNKNOWN,
    NONE,
    LEFT,
    RIGHT,
    UP
};

typedef keeperFrameSetpointEnum keeperFrameSetpoint;

SERIALIZE_ENUM(keeperFrameSetpoint);

#endif

