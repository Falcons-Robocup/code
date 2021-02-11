// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * UnShiftBallOffset.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "int/facilities/vcgeometry.hpp"


void UnShiftBallOffset::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    if (data.robot.hasBall && data.ppConfig.forwardDriving.applyLimitsToBall)
    {
        // resultVelocityRcs applies to the ball
        // convert to motor setpoint
        // this used to be called TokyoDrift
        data.resultVelocityRcs.x += data.resultVelocityRcs.phi * data.ppConfig.forwardDriving.radiusRobotToBall;

        // unshift coordinates because deadzone still needs them
        auto offset = Position2D(0.0, -data.ppConfig.forwardDriving.radiusRobotToBall, 0.0);
        data.targetPositionFcs = addRcsToFcs(offset, data.targetPositionFcs);
        data.currentPositionFcs = addRcsToFcs(offset, data.currentPositionFcs);
    }
}

