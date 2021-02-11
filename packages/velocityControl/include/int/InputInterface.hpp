// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * InputInterface.hpp
 *
 *  Created on: Oct 2020
 *      Author: Erik Kouters
 */

#ifndef INPUTINTERFACE_VELOCITYCONTROL_HPP_
#define INPUTINTERFACE_VELOCITYCONTROL_HPP_

#include <vector>

// sharedTypes
#include "robotPosVel.hpp"
#include "forbiddenArea.hpp"
#include "robotState.hpp"
#include "obstacleResult.hpp"
#include "ballResult.hpp"


class InputInterface
{
public:
    InputInterface() {};
    virtual ~InputInterface() {};

    virtual void                        fetch() = 0;

    virtual robotPosVel                 getRobotPosVelSetpoint() = 0;
    virtual robotState                  getRobotState() = 0;
    virtual std::vector<robotState>     getTeamMembers() = 0;
    virtual std::vector<ballResult>     getBalls() = 0;
    virtual std::vector<obstacleResult> getObstacles() = 0;

};

#endif

