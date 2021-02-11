// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Shielding.cpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"


void Shielding::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    if (!data.robot.hasBall)
    {
        // nothing to do -- Shielding requires ball
        return;
    }

    // get subtarget and configuration parameters
    Position2D currPos(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);
    Position2D subTarget = data.getSubTarget();
    Position2D target(data.target.pos.x, data.target.pos.y, data.target.pos.Rz);
    Vector2D subTargetDelta = subTarget.xy() - currPos.xy();
    float subTargetAngle = atan2(subTargetDelta.y, subTargetDelta.x);
    float shieldingAngleOffset = 0.4; // TODO make reconfigurable
    float shieldingObstacleProximity = 1.0;
    float shieldingMinimumTargetDistance = 1.0;

    // robot must be sufficiently far away from target
    if ((target.xy() - currPos.xy()).size() < shieldingMinimumTargetDistance)
    {
        // nothing to do, robot is too close to final destination
        TRACE("no shielding, robot is too close to final target");
        return;
    }

    // find closest opponent
    float closestOpponentDistance = 9.9;
    bool closestOpponentIsLeft = false;
    for (auto opponent: data.obstacles)
    {
        Vector2D opponentPos(opponent.position.x, opponent.position.y);
        Vector2D delta = currPos.xy() - opponentPos;
        float distance = delta.size();
        if (distance < closestOpponentDistance)
        {
            closestOpponentDistance = distance;
            // calculate on which side this opponent is, w.r.t. the path from robot to sub-target
            closestOpponentIsLeft = project_angle_mpi_pi(atan2(delta.y, delta.x) - subTargetAngle) < 0.0;
        }
    }
    TRACE("closestOpponentDistance=%.2f", closestOpponentDistance);

    // check if opponent closeby
    if (closestOpponentDistance > shieldingObstacleProximity)
    {
        TRACE("no shielding, closest opponent is too far away");
        return;
    }
    if (closestOpponentDistance < 0.1)
    {
        // cannot calculate a good angle ... this might even need to be a warning in worldModel
        TRACE("no shielding, opponent is standing right on top of robot?!");
        return;
    }

    // final check: in case there is also an opponent closeby on the OTHER side,
    // then it makes no sense to shield either way
    for (auto opponent: data.obstacles)
    {
        Vector2D opponentPos(opponent.position.x, opponent.position.y);
        Vector2D delta = currPos.xy() - opponentPos;
        bool opponentIsLeft = project_angle_mpi_pi(atan2(delta.y, delta.x) - subTargetAngle) < 0.0;
        float distance = delta.size();
        bool opponentCloseEnough = (distance < shieldingObstacleProximity);
        bool opponentOnSameSideAsClosestOpponent = (closestOpponentIsLeft == opponentIsLeft);
        if (opponentCloseEnough && !opponentOnSameSideAsClosestOpponent)
        {
            TRACE("no shielding, there are two nearby opponent on both sides of the path");
            return;
        }
    }

    // apply shielding to the correct side by modifing the angle of next subtarget
    if (closestOpponentIsLeft)
    {
        TRACE("shielding to the right side");
        data.path.at(0).pos.Rz -= shieldingAngleOffset;
    }
    else
    {
        TRACE("shielding to the left side");
        data.path.at(0).pos.Rz += shieldingAngleOffset;
    }
}

