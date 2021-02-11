// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * actionTypeEnum.hpp
 *
 *  Created on: Dec 01, 2018
 *      Author: Erik Kouters
 */

#ifndef ACTIONTYPEENUM_HPP_
#define ACTIONTYPEENUM_HPP_

#include "RtDB2.h" // required for serialization


enum class actionTypeEnum
{
    UNKNOWN,
    MOVE,
    PASS,
    SHOOT,
    LOB,
    STOP,
    GET_BALL,
    TURN_AWAY_FROM_OPPONENT,
    KEEPER_MOVE,
    KICK,
    INTERCEPT_BALL
};

SERIALIZE_ENUM(actionTypeEnum);

#endif

