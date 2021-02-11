// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * matchState.hpp
 *
 *  Created on: Oct 18, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef MATCHSTATE_HPP_
#define MATCHSTATE_HPP_

#include "RtDB2.h" // required for serialization


enum class matchPhaseEnum
{
    UNKNOWN,
    FIRST_HALF,
    HALF_TIME,
    SECOND_HALF,
    FIRST_HALF_OVERTIME,
    SECOND_HALF_OVERTIME,
    END_GAME
};

SERIALIZE_ENUM(matchPhaseEnum);

struct matchState // associated RTDB key: MATCH_STATE
{
    rtime              currentTime;
    int                goalsOwn;
    int                goalsOpponent;
    matchPhaseEnum     phase;
    
    std::string        lastRefboxCommand;
    rtime              lastRefboxCommandTime;

    SERIALIZE_DATA(currentTime, goalsOwn, goalsOpponent, phase, lastRefboxCommand, lastRefboxCommandTime);
};

#endif

