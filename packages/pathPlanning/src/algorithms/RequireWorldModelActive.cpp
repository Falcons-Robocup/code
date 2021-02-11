// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RequireWorldModelActive.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"


void RequireWorldModelActive::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");
    bool wmActive = (data.robot.status == robotStatusEnum::INPLAY);
    if (!wmActive)
    {
        data.resultStatus = actionResultTypeEnum::FAILED;
        data.stop = true;
        data.done = true;
    }
}

