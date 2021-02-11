// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CheckStopCommand.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"


void CheckStopCommand::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");
    if (data.stop)
    {
        data.resultStatus = actionResultTypeEnum::PASSED;
        data.path.clear();
        
        // create wayPoint on robot itself
        wayPoint stopPosition = wayPoint();
        stopPosition.pos = data.robot.position;
        stopPosition.vel = pose(0.0, 0.0, 0.0);
        data.path.push_back(stopPosition);

        data.done = true;
    }
}

