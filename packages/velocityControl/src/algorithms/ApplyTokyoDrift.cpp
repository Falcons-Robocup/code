// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ApplyTokyoDrift.cpp
 *
 *  Created on: Sep, 2021
 *      Author: Erik Kouters
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "int/facilities/vcgeometry.hpp"


void ApplyTokyoDrift::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    if (data.robot.hasBall && fabs(data.deltaPositionRcs.phi) > data.ppConfig.tokyoDrift.toleranceRz)
    {
        // resultVelocityRcs applies to the ball
        // convert to motor setpoint
        TRACE("Adding %8.4f to resultVelocityRcs.x to induce Tokyo Drift", (data.resultVelocityRcs.phi * data.ppConfig.forwardDriving.radiusRobotToBall));
        data.resultVelocityRcs.x += data.resultVelocityRcs.phi * data.ppConfig.forwardDriving.radiusRobotToBall;
    }
}

