// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CheckTargetReached.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"


void CheckTargetReached::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");
    // get delta - TODO simplify
    Position2D targetPos = data.getSubTarget();
    Position2D robotPos(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);
    Position2D deltaPositionFcs = targetPos - robotPos;
    deltaPositionFcs.phi = project_angle_mpi_pi(deltaPositionFcs.phi);
    // compare with tolerances
    bool xyOk = deltaPositionFcs.xy().size() < data.configPP.deadzone.toleranceXY;
    bool RzOk = fabs(deltaPositionFcs.phi) < data.configPP.deadzone.toleranceRz;
    TRACE("xyOk=%d RzOk=%d", xyOk, RzOk);
    // convergence criterion, especially useful for testing where overshoot can cause premature 'PASSED'
    static int tickCountTargetReached = 0;
    // update data
    if (xyOk && RzOk)
    {
        tickCountTargetReached++;
        if (tickCountTargetReached >= data.configPP.numExtraSettlingTicks)
        {
            data.resultStatus = actionResultTypeEnum::PASSED;
            data.done = true;
        }
    }
    else
    {
        tickCountTargetReached = 0;
        data.resultStatus = actionResultTypeEnum::RUNNING;

        TRACE("xyOK=%d -> deltaPositionFcs=%6.2f < toleranceXY=%6.2f", xyOk, data.deltaPositionFcs.xy().size(), data.configPP.deadzone.toleranceXY);
        TRACE("RzOK=%d -> deltaPositionFcs=%8.4f < toleranceRz=%8.4f", RzOk, fabs(deltaPositionFcs.phi), data.configPP.deadzone.toleranceRz);
    }
}

