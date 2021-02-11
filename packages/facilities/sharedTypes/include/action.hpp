// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * action.hpp
 *
 *  Created on: Dec 01, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef ACTION_HPP_
#define ACTION_HPP_

#include "actionTypeEnum.hpp"
#include "motionTypeEnum.hpp"
#include "vec3d.hpp"

#include "RtDB2.h" // required for serialization


struct action
{
    actionTypeEnum     action;
    vec3d              position; // used by move, pass, shoot, lob, turnAwayFromOpponent
    motionTypeEnum     motionType; // different move types (e.g., normal, accurate (setpiece), intercept)
    bool               ballHandlersEnabled; // used by move -- teamplay will disable BH during a prepare setpiece
    
    SERIALIZE_DATA(action, position, motionType, ballHandlersEnabled);
};

#endif

