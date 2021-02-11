// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpOverrideState.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#ifndef TPOVERRIDESTATE_HPP_
#define TPOVERRIDESTATE_HPP_

#include "RtDB2.h"

#include "tpOverrideLevelEnum.hpp"
#include "treeEnum.hpp"
#include "tpActionEnum.hpp"

struct tpOverrideState
{
    bool                     active;
    tpOverrideLevelEnum      level;
    treeEnum                 treeValue;         // Used when level={GAMESTATE,ROLE,BEHAVIOR}
    tpActionEnum             tpAction;          // Used when level=TP_ACTION
    std::map< std::string, std::string> params; // Used when level={GAMESTATE,ROLE,BEHAVIOR,TP_ACTION}
    action                   mpAction;          // Used when level=MP_ACTION
    
    SERIALIZE_DATA(active, level, treeValue, tpAction, params, mpAction);
};


#endif

