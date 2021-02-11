// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagTeamplay.hpp
 *
 *  Created on: Nov 27, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef DIAGTEAMPLAY_HPP_
#define DIAGTEAMPLAY_HPP_

#include "RtDB2.h" // required for serialization
#include <string>

struct diagTeamplay
{
    std::string   action;
    std::string   behavior;
    std::string   role;
    std::string   state; // the gamestate
    std::vector<std::string> trees; // the full stack of executed "trees", including the action that is executed
    bool          aiming;
    float         shootTargetX;
    float         shootTargetY;
    
    SERIALIZE_DATA(action, behavior, role, state, trees, aiming, shootTargetX, shootTargetY);
};

#endif

