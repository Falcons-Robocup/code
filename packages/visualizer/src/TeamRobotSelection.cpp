// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * FieldWidgetGameSignalSubscriber.h
 *
 *  Created on: October 19, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/TeamRobotSelection.h"

void TeamRobotSelection::setTeamMode() 
{
    _viewMode = TEAM;
    onTeamModeChanged();
}

void TeamRobotSelection::setRobotMode(uint8_t id) 
{
    _viewMode = ROBOT;
    _robotModeId = id; 
    onRobotModeChanged();
}
