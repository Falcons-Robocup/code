// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBrefboxAdapter.cpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBrefboxAdapter.hpp"
#include "int/RTDBaccess.hpp"
#include "int/simulation_generated_enum2str.hpp"

#include <stdexcept>

#include "FalconsRTDB.hpp"
#include "tracing.hpp"
#include "ftime.hpp"


RTDBRefBoxAdapter::RTDBRefBoxAdapter()
: _command_for_team_A("")
, _command_for_team_B("")
, _score_for_team_A(0)
, _score_for_team_B(0)
{ }

void RTDBRefBoxAdapter::sendCommand () const
{
    T_MATCH_STATE matchState;
    matchState.currentTime = ftime::now();
    matchState.goalsOwn = _score_for_team_A;
    matchState.goalsOpponent = _score_for_team_B;

    auto rtdbConnection = getRTDBConnection(TeamID::A);
    matchState.lastRefboxCommand = _command_for_team_A;
    tprintf("put MATCH_STATE (Team A) lastRefboxCommand=%s", matchState.lastRefboxCommand.c_str());
    auto r = rtdbConnection->put(MATCH_STATE, &matchState);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing MATCH_STATE (refbox signal) to RtDB for team A");
    }

    rtdbConnection = getRTDBConnection(TeamID::B);
    matchState.lastRefboxCommand = _command_for_team_B;
    tprintf("put MATCH_STATE (Team B) lastRefboxCommand=%s", matchState.lastRefboxCommand.c_str());
    r = rtdbConnection->put(MATCH_STATE, &matchState);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing MATCH_STATE (refbox signal) to RtDB for team B");
    }
}

void RTDBRefBoxAdapter::registerGoal (const TeamID& teamID)
{
    TRACE_FUNCTION("team = %s", teamID);
    if (teamID == TeamID::A)
    {
        _score_for_team_A += 1;
    }
    else
    {
        _score_for_team_B += 1;
    }
}

void RTDBRefBoxAdapter::republish() const
{
    TRACE_FUNCTION("");
    sendCommand();
}

void RTDBRefBoxAdapter::sendStart()
{
    TRACE_FUNCTION("");
    _command_for_team_A = "START";
    _command_for_team_B = "START";
    sendCommand();
}


void RTDBRefBoxAdapter::sendStop()
{
    TRACE_FUNCTION("");
    _command_for_team_A = "STOP";
    _command_for_team_B = "STOP";
    sendCommand();
}


void RTDBRefBoxAdapter::sendSetpiece (const SetpieceEnum& setpiece, const TeamID& teamID)
{
    TRACE_FUNCTION("setpiece = %s, team = %s", setpiece, teamID);

    if (setpiece == SetpieceEnum::DROPPED_BALL)
    {
        _command_for_team_A = "DROP_BALL";
        _command_for_team_B = "DROP_BALL";
    }
    else
    {
        auto infix = enum2str(setpiece);

        auto suffix_for_team_A = std::string();
        auto suffix_for_team_B = std::string();

        if (teamID == TeamID::A)
        {
            suffix_for_team_A = "OWN";
            suffix_for_team_B = "OPP";
        }
        else
        {
            suffix_for_team_A = "OPP";
            suffix_for_team_B = "OWN";
        }

        _command_for_team_A = (boost::format("%1%_%2%") % infix % suffix_for_team_A).str();
        _command_for_team_B = (boost::format("%1%_%2%") % infix % suffix_for_team_B).str();
    }

    sendCommand();
}
