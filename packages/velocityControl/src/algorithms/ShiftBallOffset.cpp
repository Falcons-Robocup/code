// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ShiftBallOffset.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "int/facilities/vcgeometry.hpp"


void ShiftBallOffset::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // get uncorrected data
    data.targetPositionFcs = Position2D(data.target.pos.x, data.target.pos.y, data.target.pos.Rz);
    data.currentPositionFcs = Position2D(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);
    data.currentVelocityFcs = Velocity2D(data.robot.velocity.x, data.robot.velocity.y, data.robot.velocity.Rz);

    // add ball offset, if applicable
    if (data.robot.hasBall && data.ppConfig.forwardDriving.applyLimitsToBall)
    {
        auto offset = Position2D(0.0, data.ppConfig.forwardDriving.radiusRobotToBall, 0.0);
        data.targetPositionFcs = addRcsToFcs(offset, data.targetPositionFcs);
        data.currentPositionFcs = addRcsToFcs(offset, data.currentPositionFcs);
    }
}

