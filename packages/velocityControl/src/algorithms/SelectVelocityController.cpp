// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SelectVelocityController.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"


void SelectVelocityController::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // get delta
    float distanceToSubTarget = data.deltaPositionFcs.xy().size();

    // switch between long and short stroke
    if (distanceToSubTarget > data.currentMotionTypeConfig.velocityControllers.threshold)
    {
        data.shortStroke = false;
        data.vcSetpointConfig = data.currentMotionTypeConfig.velocityControllers.longStroke;
    }
    else
    {
        data.shortStroke = true;
        data.vcSetpointConfig = data.currentMotionTypeConfig.velocityControllers.shortStroke;
    }
}

