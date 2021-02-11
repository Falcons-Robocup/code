// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * worldModelInfo.hpp
 *
 *  Created on: Sep 24, 2017
 *      Author: Coen Tempelaars
 */

#ifndef WORLDMODELINFO_HPP_
#define WORLDMODELINFO_HPP_

#include "boost/optional.hpp"
#include "position2d.hpp"
#include "int/types/cBallLocationTypes.hpp"
#include "int/types/cBallPossessionTypes.hpp"
#include "int/types/cRobotLocationTypes.hpp"
#include "int/types/robot.hpp"

namespace teamplay
{

typedef struct
{
    robotNumber number;
    Position2D position;
    Velocity2D velocity;
} worldModelInfoOwnRobot;

typedef struct
{
    worldModelInfoOwnRobot ownRobot;
    robotLocations activeTeammembers;
    robotLocations obstacles;
    boost::optional<ballLocation> ball;
    ballPossession_struct_t ballPossession;
} worldModelInfo;

} /* namespace teamplay */

#endif /* WORLDMODELINFO_HPP_ */
