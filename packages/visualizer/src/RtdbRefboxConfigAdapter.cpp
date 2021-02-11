// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RtdbRefboxConfigAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Edwin Schreuder
 */

#include "../include/int/RtdbRefboxConfigAdapter.h"

#include <exception>

#include "cDbConnection.hpp"
#include "falconsCommon.hpp"

#include "rtdbKeys.hpp"
// #include "agentIDs.hpp"
#include "refboxConfig.hpp"

using std::string;
using std::runtime_error;

RtdbRefboxConfigAdapter::RtdbRefboxConfigAdapter()
{
}

RtdbRefboxConfigAdapter::~RtdbRefboxConfigAdapter()
{

}

template <typename T> void RtdbRefboxConfigAdapter::setValue(int database, string key, T value)
{
    RtDB2* coachDatabase = connection._rtdb[database];

    int result = coachDatabase->put(key, &value);
    if (result != RTDB2_SUCCESS)
    {
        throw(runtime_error("RTDB error."));
    }
}

template <typename T> T RtdbRefboxConfigAdapter::getValue(int database, const string key)
{
    T value;

    RtDB2* coachDatabase = connection._rtdb[database];
    int life = 0;

    int result = coachDatabase->get(key, &value, life, database);
    if (result != RTDB2_SUCCESS)
    {
        throw(runtime_error("RTDB error."));
    }

    return value;
}

void RtdbRefboxConfigAdapter::setTeamColor(TeamColor teamColor)
{
    refboxConfig config = getValue<refboxConfig>(COACH_AGENTID, REFBOX_CONFIG);

    switch(teamColor)
    {
    case CYAN:
        config.teamColor = refboxConfigTeamColor::CYAN;
        break;
    case MAGENTA:
        config.teamColor = refboxConfigTeamColor::MAGENTA;
        break;
    }

    setValue(COACH_AGENTID, REFBOX_CONFIG, config);
}

void RtdbRefboxConfigAdapter::setPlayingField(PlayingField playingField)
{
    refboxConfig config = getValue<refboxConfig>(COACH_AGENTID, REFBOX_CONFIG);

    switch(playingField)
    {
    case FIELD_A:
        config.field = refboxConfigField::FIELD_A;
        break;
    case FIELD_B:
        config.field = refboxConfigField::FIELD_B;
        break;
    case FIELD_C:
        config.field = refboxConfigField::FIELD_C;
        break;
    case FIELD_D:
        config.field = refboxConfigField::FIELD_D;
        break;
    case FIELD_FALCONS:
        config.field = refboxConfigField::FIELD_FALCONS;
        break;
    case FIELD_LOCALHOST:
        config.field = refboxConfigField::FIELD_LOCALHOST;
        break;
    }

    setValue(COACH_AGENTID, REFBOX_CONFIG, config);
}

void RtdbRefboxConfigAdapter::setTTAConfiguration(RefboxConfigAdapter::TTAConfiguration ttaConfig)
{
    refboxConfig config = getValue<refboxConfig>(COACH_AGENTID, REFBOX_CONFIG);

    switch(ttaConfig)
    {
    case NONE:
        config.technicalTeamArea = refboxConfigTTAside::NONE;
        break;
    case FRONT_LEFT:
        config.technicalTeamArea = refboxConfigTTAside::FRONT_LEFT;
        break;
    case FRONT_RIGHT:
        config.technicalTeamArea = refboxConfigTTAside::FRONT_RIGHT;
        break;
    case BACK_LEFT:
        config.technicalTeamArea = refboxConfigTTAside::BACK_LEFT;
        break;
    case BACK_RIGHT:
        config.technicalTeamArea = refboxConfigTTAside::BACK_RIGHT;
        break;
    }

    setValue(COACH_AGENTID, REFBOX_CONFIG, config);
}

