// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ForwardDriving.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"
#include "int/facilities/ppgeometry.hpp"


void ForwardDriving::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // get applicable configuration
    ForwardDrivingConfig config = (data.robot.hasBall ? data.config.forwardDriving.withBall : data.config.forwardDriving.withoutBall);

    // disabled?
    if (!config.enabled)
    {
        return;
    }

    // if there are already sub-targets (typically calculated by obstacle avoidance), then modify their angles
    if (data.path.size() > 1)
    {
        for (int it = 0; it < (int)data.path.size() - 1; ++it)
        {
            // each one must face the next
            auto wp = data.path.at(it);
            auto wpNext = data.path.at(it+1);
            data.path[it].pos.Rz = atan2(wpNext.pos.y - wp.pos.y, wpNext.pos.x - wp.pos.x);
        }
    }

    // get sub-target and current robot position
    Position2D subTarget = data.getSubTarget();
    Position2D robotPos(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);

    // check if distance is large enough
    if ((subTarget - robotPos).xy().size() < config.minimumDistance)
    {
        TRACE("too close to sub-target");
        return;
    }

    // insert sub-targets
    Position2D subTarget1 = faceTowards(robotPos, subTarget.x, subTarget.y);
    subTarget1 = addRcsToFcs(Position2D(0.0, config.minimumDistance, 0.0), subTarget1);
    data.insertSubTarget(subTarget1);
    /*
    Position2D subTarget2 = subTarget;
    if ((subTarget - robotPos).xy().size() > 2.0 * config.minimumDistance)
    {
        subTarget2.phi = subTarget1.phi;
        subTarget2 = addRcsToFcs(Position2D(0.0, -config.minimumDistance, 0.0), subTarget2);
        data.insertSubTarget(subTarget2);
    }*/
}

