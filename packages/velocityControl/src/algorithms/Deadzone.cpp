// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Deadzone.cpp
 *
 *  Created on: January, 2020
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "cDiagnostics.hpp"


void Deadzone::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // apply Deadzone only when getting a position-based setpoint
    if (data.robotPosVelMoveType == robotPosVelEnum::POSVEL || data.robotPosVelMoveType == robotPosVelEnum::POS_ONLY)
    {

        // compare with tolerances
        bool xyOk = data.deltaPositionRcs.xy().size() < data.ppConfig.deadzone.toleranceXY;
        bool RzOk = fabs(data.deltaPositionRcs.phi) < data.ppConfig.deadzone.toleranceRz;
        TRACE("deadzone XY -> deltaPositionRcs=%8.4f < toleranceXY=%8.4f", data.deltaPositionRcs.xy().size(), data.ppConfig.deadzone.toleranceXY);
        TRACE("deadzone Rz -> deltaPositionRcs=%8.4f < toleranceRz=%8.4f", fabs(data.deltaPositionRcs.phi), data.ppConfig.deadzone.toleranceRz);
        TRACE("xyOk=%d RzOk=%d", xyOk, RzOk);

        // "deadzone" overrule: in case of partial convergence, set zero output
        // this is especially useful during tuning, to prevent a tiny Rz move setpoint interfering with XY
        data.deadzone[0] = false;
        data.deadzone[1] = false;
        data.deadzone[2] = false;
        if (data.ppConfig.deadzone.enabled)
        {
            if (xyOk)
            {
                data.resultVelocityRcs.x = 0.0;
                data.resultVelocityRcs.y = 0.0;
                data.deadzone[0] = true;
                data.deadzone[1] = true;
                TRACE("deadzone XY -> setting vx=0 and vy=0");
            }
            
            if (RzOk)
            {
                data.resultVelocityRcs.phi = 0.0;
                data.deadzone[2] = true;
                TRACE("deadzone Rz -> setting vphi=0");
            }

            if (!xyOk && !RzOk)
            {
                TRACE("deadzone not applied");
            }
        }
        else
        {
            TRACE("deadzone option disabled");
        }
    }
    else
    {
        TRACE("deadzone ignored: setpoint is not position-based");
    }
}

