// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cQueryInterface.cpp
 *
 *  Created on: Apr 24, 2018
 *      Author: Erik Kouters / Coen Tempelaars
 */

#include "int/cQueryInterface.hpp"
#include "tracing.hpp"

void cQueryInterface::connect(MP_WorldModelInterface* wm)
{
    _wm = wm;
}

double cQueryInterface::timeToBall(const uint8_t robotID) const
{
    TRACE("> robotID=%d", robotID);
    Vector3D ball = _wm->ballPosition();

    robotState robot;
    double distance = 999;
    if (_wm->getRobotState(robot, robotID))
    {
        distance = calc_distance(ball.x, ball.y, robot.position.x, robot.position.y);
    }
    // TODO: what if robot is offline? do we trust teamplay won't call us then?
    
    // TODO: make a more accurate model here, by factoring in robot speed, acceleration, possibly even pathing
    
    TRACE("< timeToBall=%4.2f", distance);
    return distance;
}
