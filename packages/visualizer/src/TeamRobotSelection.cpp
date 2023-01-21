// Copyright 2016-2022 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TeamRobotSelection.cpp
 *
 *  Created on: October 19, 2016
 *      Author: Diana Koenraadt
 */

#include <cassert>

// Internal:
#include "int/TeamRobotSelection.h"

void TeamRobotSelection::setTeamMode() 
{
    _viewMode = TEAM::OWN;
    onTeamModeChanged();
}

void TeamRobotSelection::setRobotMode(uint8_t id) 
{
    assert(id != 0);
    _viewMode = id;
    onRobotModeChanged();
}
