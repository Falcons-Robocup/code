// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * LinearVelocitySetpointController.cpp
 *
 *  Created on: October, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocitySetpointControllers.hpp"


bool LinearVelocitySetpointController::calculate(VelocityControlData &data, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    Position2D deltaPosition; // in RCS or FCS depending on configuration

    if (data.vcSetpointConfig.coordinateSystem == CoordinateSystemEnum::RCS)
    {
        deltaPosition = data.deltaPositionRcs;
        TRACE("controlling velocity in RCS");
    }
    else
    {
        deltaPosition = data.deltaPositionFcs;
        TRACE("controlling velocity in FCS");
    }

    // XY
    Vector2D v = deltaPosition.xy() / data.dt;
    resultVelocity.x = v.x;
    resultVelocity.y = v.y;
    // Rz
    float dRz = deltaPosition.phi;
    resultVelocity.phi = dRz / data.dt;

    TRACE("resultVelocity=(%6.4f, %6.4f, %6.4f)", resultVelocity.x, resultVelocity.y, resultVelocity.phi);

    return true;
}

