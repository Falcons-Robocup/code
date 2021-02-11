// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CalculateAccelerating.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"


void CalculateAccelerating::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // calculate current velocity in RCS
    Velocity2D c = data.currentVelocityFcs;
    c = c.transform_fcs2rcs(data.currentPositionFcs);

    // calculate acceleration based on setpoints (real acceleration typically lags behind a bit)
    auto a = (data.resultVelocityRcs - data.previousVelocityRcs) / data.dt;

    TRACE("prevVelocity=(%8.4f,%8.4f,%8.4f)", data.previousVelocityRcs.x, data.previousVelocityRcs.y, data.previousVelocityRcs.phi);
    TRACE("currVelocity=(%8.4f,%8.4f,%8.4f)", c.x, c.y, c.phi);
    TRACE("newVelocity=(%8.4f,%8.4f,%8.4f)", data.resultVelocityRcs.x, data.resultVelocityRcs.y, data.resultVelocityRcs.phi);
    TRACE("dt=%8.4f", data.dt);

    // take inproduct of acceleration with current velocity to determine if robot is accelerating or not
    data.isAccelerating[0] = (a.x * c.x >= data.currentMotionTypeConfig.limits.accThresholdX);
    data.isAccelerating[1] = (a.y * c.y >= data.currentMotionTypeConfig.limits.accThresholdY);
    data.isAccelerating[2] = (a.phi * c.phi >= data.currentMotionTypeConfig.limits.accThresholdRz);

    TRACE("isAccX=%d isAccY=%d isAccRz=%d", data.isAccelerating[0], data.isAccelerating[1], data.isAccelerating[2]);
}

