// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CalculateVelocity.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "cDiagnostics.hpp"


CalculateVelocity::CalculateVelocity(boost::function<AbstractVelocitySetpointController *(void)> callback)
{
    _callback = callback;
}

void CalculateVelocity::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // pathPlanning library should ensure some controller is configured
    // we trigger this just-in-time operation via a callback, which makes it possible
    // for instance to overrule runtime whichever controller with Tokyo drift from associated algorithm
    auto controller = _callback();
    assert(controller != NULL);

    // call, check result, store output
    Velocity2D resultVelocity;
    bool success = controller->calculate(data, resultVelocity);
    if (!success)
    {
        resultVelocity = Velocity2D(0.0, 0.0, 0.0);
        TRACE_WARNING("velocity setpoint calculation failed");
    }

    if (data.robotPosVelMoveType == robotPosVelEnum::POSVEL || data.robotPosVelMoveType == robotPosVelEnum::POS_ONLY)
    {
        // if needed, transform from FCS to RCS
        if (data.vcSetpointConfig.coordinateSystem == CoordinateSystemEnum::RCS)
        {
            // already in the correct coordinate system
            data.resultVelocityRcs = resultVelocity;
        }
        else
        {
            // need a transformation
            data.resultVelocityRcs = resultVelocity.transform_fcs2rcs(data.currentPositionFcs);
        }
    }
    else if (data.robotPosVelMoveType == robotPosVelEnum::VEL_ONLY)
    {
        data.resultVelocityRcs = resultVelocity;
    }

    // store for next iteration
    data.previousTimestamp = data.timestamp;
    data.previousVelocityRcs = data.resultVelocityRcs;

    TRACE("vx=%8.4f vy=%8.4f vphi=%8.4f (RCS) success=%d", data.resultVelocityRcs.x, data.resultVelocityRcs.y, data.resultVelocityRcs.phi, success);
}

