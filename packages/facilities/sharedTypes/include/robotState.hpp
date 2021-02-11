// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotState.hpp
 *
 *  Created on: Jul 22, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef ROBOTSTATE_HPP_
#define ROBOTSTATE_HPP_

#include "robotStatusEnum.hpp"
#include "teamIdType.hpp"
#include "pose.hpp"
#include "vec2d.hpp"

#include "RtDB2.h" // required for serialization


struct robotState
{
    robotStatusEnum    status;
    rtime              timestamp;
    pose               position;
    pose               velocity;
    bool               hasBall;
    vec2d              ballAcquired; // only filled in when having ball, for dribble rule
    int                robotId;
    teamIdType         teamId;
    
    SERIALIZE_DATA_FIXED(status, timestamp, position, velocity, hasBall, ballAcquired, robotId, teamId);
};

#endif

