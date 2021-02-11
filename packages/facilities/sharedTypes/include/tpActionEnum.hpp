// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpActionEnum.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#ifndef TPACTIONENUM_HPP_
#define TPACTIONENUM_HPP_

#include "RtDB2.h"


enum class tpActionEnum
{
    INVALID = 0,
    SUCCESS,
    MOVE,
    STOP,
    SHOOT,
    PASS,
    POSITION_BEFORE_POI,
    POSITION_BEHIND_POI,
    POSITION_FOR_OPP_SETPIECE,
    POSITION_FOR_OWN_SETPIECE,
    GET_BALL,
    GOALKEEPER,
    MOVE_TO_FREE_SPOT,
    INTERCEPT_BALL,
    AVOID_POI,
    DEFEND_PENALTY_AREA,
    TURN_AWAY_FROM_OPPONENT,
    DEFEND_ATTACKING_OPPONENT,
    DRIBBLE,

    SIZE_OF_ENUM
};

SERIALIZE_ENUM(tpActionEnum);

#endif

