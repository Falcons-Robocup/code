// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * InputInterface.hpp
 *
 *  Created on: July 2019
 *      Author: Jan Feitsma
 */

#ifndef INPUTINTERFACE_PATHPLANNING_HPP_
#define INPUTINTERFACE_PATHPLANNING_HPP_

#include <vector>

// sharedTypes
#include "motionSetpoint.hpp"
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

    virtual motionSetpoint              getMotionSetpoint() = 0;
    virtual std::vector<forbiddenArea>  getForbiddenAreas() = 0;
    virtual robotState                  getRobotState() = 0;
    virtual std::vector<robotState>     getTeamMembers() = 0;
    virtual std::vector<ballResult>     getBalls() = 0;
    virtual std::vector<obstacleResult> getObstacles() = 0;

};

#endif

