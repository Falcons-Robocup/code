// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameState.cpp
 *
 *  Created on: Jul 25, 2016
 *      Author: Coen Tempelaars
 */

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <tuple>

#include "int/types/GameState.hpp"

using namespace teamplay;

static bool isValid(const GoverningGameState& g, const GoverningMatchState& p, const SetpieceOwner& o, const SetpieceType& t)
{
    bool validGameState = false;

    /* Is the combination of inputs valid? */
    if (  (g == GoverningGameState::INVALID)
       || (g == GoverningGameState::NEUTRAL_PLAYING)
       || (g == GoverningGameState::NEUTRAL_STOPPED)  )
    {
        // Restrictions:
        // - For setpiece settings, the governing gamestate must be setpiece preparing/executing.
        //   Otherwise, setpieces settings must be NONE.
        // - In case of out of play state, we cannot be playing (NEUTRAL_PLAYING => IN_PLAY)
        validGameState = ((o == SetpieceOwner::NONE) && (t == SetpieceType::NONE)) && ((g != GoverningGameState::NEUTRAL_PLAYING) || (p == GoverningMatchState::IN_MATCH));
    }
    else if (  (g == GoverningGameState::SETPIECE_PREPARING)
            || (g == GoverningGameState::SETPIECE_EXECUTING)  )
    {
        // Restrictions:
        // - The setpiecetype must be set
        // - (SetpieceOwner::NONE <==> SetpieceType::DROPPED_BALL AND NOT SetpieceType::NONE), i.e.:
        //   If there is no setpiece owner, the Ball must have been dropped
        //   If the Ball is dropped, there can be no setpiece owner
        // - The playstate OUT_OF_PLAY is only allowed when the setpiece is PENALTY
        //   (GoverningMatchState::OUT_OF_MATCH => SetpieceType::PENALTY)
        validGameState = (  ((o != SetpieceOwner::NONE) || (t == SetpieceType::DROPPED_BALL))
                         && ((o == SetpieceOwner::NONE) || (t != SetpieceType::DROPPED_BALL))
                         && (t != SetpieceType::NONE)
                         && ((p != GoverningMatchState::OUT_OF_MATCH) || (t == SetpieceType::PENALTY))
                         );

        // Special case for parking
        if (t == SetpieceType::PARKING)
        {
            validGameState = (o == SetpieceOwner::NONE);
        }
    }
    else
    {
        validGameState = false;
    }

    return validGameState;
}


GameState::GameState()
{
    setGameState(GoverningGameState::INVALID, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
}

GameState::GameState(const GoverningGameState& g)
{
    setGameState(g, GoverningMatchState::IN_MATCH, SetpieceOwner::NONE, SetpieceType::NONE);
}

GameState::GameState(const GoverningGameState& g, const GoverningMatchState& p, const SetpieceOwner& o, const SetpieceType& t)
{
    setGameState(g, p, o, t);
}

GameState::~GameState() { }


void GameState::setGameState(const GoverningGameState& g, const GoverningMatchState& p, const SetpieceOwner& o, const SetpieceType& t)
{
    if (isValid(g, p, o, t))
    {
        _governingGameState = g;
        _governingMatchState = p;
        _setpieceOwner = o;
        _setpieceType = t;
    }
    else
    {
        std::ostringstream msg;
        msg << "Error: attempt to construct an illegal gamestate. " << toString(g, p, o, t);
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }
}

bool GameState::operator== (const GameState& other) const
{
    return (  (_governingGameState == other._governingGameState)
           && (_governingMatchState == other._governingMatchState)
           && (_setpieceOwner == other._setpieceOwner)
           && (_setpieceType == other._setpieceType)  );
}

bool GameState::operator!= (const GameState& other) const
{
    return !(*this == other);
}

// To allow GameState as key in std::map
bool GameState::operator< (const GameState& other) const
{
    //return std::tie (lhs.field1, lhs.field2) < std::tie (rhs.field1, rhs.field);

    return std::tie(_governingGameState, _governingMatchState, _setpieceOwner, _setpieceType)
            < std::tie(other._governingGameState, other._governingMatchState, other._setpieceOwner, other._setpieceType);

    //return (  (_governingGameState < other._governingGameState)
    //       && (_governingMatchState < other._governingMatchState)
    //       && (_setpieceOwner < other._setpieceOwner)
    //       && (_setpieceType < other._setpieceType)  );
}

bool GameState::isSetPiece() const
{
    return (  (_governingGameState == GoverningGameState::SETPIECE_PREPARING)
           || (_governingGameState == GoverningGameState::SETPIECE_EXECUTING)  );
}

bool GameState::isOwnSetPiece() const
{
    return (_setpieceOwner == SetpieceOwner::OWN);
}

bool GameState::isPrepareSetPiece() const
{
    return (_governingGameState == GoverningGameState::SETPIECE_PREPARING);
}

bool GameState::isExecuteSetPiece() const
{
    return (_governingGameState == GoverningGameState::SETPIECE_EXECUTING);
}

bool GameState::isDroppedBallSetPiece() const
{
    return (_setpieceType == SetpieceType::DROPPED_BALL);
}

bool GameState::isKickoffSetPiece() const
{
    return (_setpieceType == SetpieceType::KICKOFF);
}

bool GameState::isPenaltySetPiece() const
{
    return (_setpieceType == SetpieceType::PENALTY);
}

bool GameState::isGoalkickSetPiece() const
{
    return (_setpieceType == SetpieceType::GOALKICK);
}

bool GameState::isFreekickSetPiece() const
{
    return (_setpieceType == SetpieceType::FREEKICK);
}

bool GameState::isCornerSetPiece() const
{
    return (_setpieceType == SetpieceType::CORNER);
}

bool GameState::isThrowinSetPiece() const
{
    return (_setpieceType == SetpieceType::THROWIN);
}

bool GameState::isParkingSetPiece() const
{
    return (_setpieceType == SetpieceType::PARKING);
}

bool GameState::isShowingAliveSetPiece() const
{
    return (_setpieceType == SetpieceType::SHOWING_ALIVE);
}

bool GameState::isInMatch() const
{
    return (_governingMatchState == GoverningMatchState::IN_MATCH);
}

bool GameState::isStopped() const
{
    return (_governingGameState == GoverningGameState::NEUTRAL_STOPPED);
}

bool GameState::isValidGameState() const
{
    return isValid(_governingGameState, _governingMatchState, _setpieceOwner, _setpieceType);
}

GoverningMatchState GameState::fromStringGoverningMatchState(const std::string& str) const
{
    // OUT_OF_MATCH_NEUTRAL_STOPPED
    // OWN_PENALTY_SHOOTOUT_EXECUTE
    // OWN_PENALTY_SHOOTOUT_PREPARE
    // OPP_PENALTY_SHOOTOUT_EXECUTE
    // OPP_PENALTY_SHOOTOUT_PREPARE
    // PARKING
    if (str == "OUT_OF_MATCH_NEUTRAL_STOPPED" ||
        str == "OWN_PENALTY_SHOOTOUT_EXECUTE" ||
        str == "OWN_PENALTY_SHOOTOUT_PREPARE" ||
        str == "OPP_PENALTY_SHOOTOUT_EXECUTE" ||
        str == "OPP_PENALTY_SHOOTOUT_PREPARE" ||
        str == "PARKING"
        )
    {
        return GoverningMatchState::OUT_OF_MATCH;
    }
    else
    {
        // INVALID
        // DROPPED_BALL_EXECUTE
        // DROPPED_BALL_PREPARE
        // NEUTRAL_PLAYING
        // NEUTRAL_STOPPED
        // OPP_CORNER_EXECUTE
        // OPP_CORNER_PREPARE
        // OPP_FREEKICK_EXECUTE
        // OPP_FREEKICK_PREPARE
        // OPP_GOALKICK_EXECUTE
        // OPP_GOALKICK_PREPARE
        // OPP_KICKOFF_EXECUTE
        // OPP_KICKOFF_PREPARE
        // OPP_PENALTY_EXECUTE
        // OPP_PENALTY_PREPARE
        // OPP_THROWIN_EXECUTE
        // OPP_THROWIN_PREPARE
        // OWN_CORNER_EXECUTE
        // OWN_CORNER_PREPARE
        // OWN_FREEKICK_EXECUTE
        // OWN_FREEKICK_PREPARE
        // OWN_GOALKICK_EXECUTE
        // OWN_GOALKICK_PREPARE
        // OWN_KICKOFF_EXECUTE
        // OWN_KICKOFF_PREPARE
        // OWN_PENALTY_EXECUTE
        // OWN_PENALTY_PREPARE
        // OWN_THROWIN_EXECUTE
        // OWN_THROWIN_PREPARE
        return GoverningMatchState::IN_MATCH;
    }
}

GoverningGameState GameState::fromStringGoverningGameState(const std::string& str) const
{
    // INVALID
    if (str == "INVALID")
    {
        return GoverningGameState::INVALID;
    }

    // OUT_OF_MATCH_NEUTRAL_STOPPED
    // NEUTRAL_STOPPED
    // PARKING
    else if (str == "OUT_OF_MATCH_NEUTRAL_STOPPED" ||
             str == "NEUTRAL_STOPPED" ||
             str == "PARKING"
             )
    {
        return GoverningGameState::NEUTRAL_STOPPED;
    }

    // NEUTRAL_PLAYING
    else if (str == "NEUTRAL_PLAYING")
    {
        return GoverningGameState::NEUTRAL_PLAYING;
    }

    // OWN_PENALTY_SHOOTOUT_PREPARE
    // OPP_PENALTY_SHOOTOUT_PREPARE
    // DROPPED_BALL_PREPARE
    // OPP_CORNER_PREPARE
    // OPP_FREEKICK_PREPARE
    // OPP_GOALKICK_PREPARE
    // OPP_KICKOFF_PREPARE
    // OPP_PENALTY_PREPARE
    // OPP_THROWIN_PREPARE
    // OWN_CORNER_PREPARE
    // OWN_FREEKICK_PREPARE
    // OWN_GOALKICK_PREPARE
    // OWN_KICKOFF_PREPARE
    // OWN_PENALTY_PREPARE
    // OWN_THROWIN_PREPARE
    else if (str.ends_with("PREPARE"))
    {
        return GoverningGameState::SETPIECE_PREPARING;
    }

    // OWN_PENALTY_SHOOTOUT_EXECUTE
    // OPP_PENALTY_SHOOTOUT_EXECUTE
    // DROPPED_BALL_EXECUTE
    // OPP_CORNER_EXECUTE
    // OPP_FREEKICK_EXECUTE
    // OPP_GOALKICK_EXECUTE
    // OPP_KICKOFF_EXECUTE
    // OPP_PENALTY_EXECUTE
    // OPP_THROWIN_EXECUTE
    // OWN_CORNER_EXECUTE
    // OWN_FREEKICK_EXECUTE
    // OWN_GOALKICK_EXECUTE
    // OWN_KICKOFF_EXECUTE
    // OWN_PENALTY_EXECUTE
    // OWN_THROWIN_EXECUTE
    else
    {
        return GoverningGameState::SETPIECE_EXECUTING;
    }
}

SetpieceType GameState::fromStringSetpieceType(const std::string& str) const
{
    // PARKING
    if (str == "PARKING")
    {
        return SetpieceType::PARKING;
    }

    // SHOWING_ALIVE
    else if (str == "SHOWING_ALIVE")
    {
        return SetpieceType::SHOWING_ALIVE;
    }

    // DROPPED_BALL_EXECUTE
    // DROPPED_BALL_PREPARE
    else if (str.starts_with("DROPPED_BALL"))
    {
        return SetpieceType::DROPPED_BALL;
    }

    // OPP_CORNER_EXECUTE
    // OPP_CORNER_PREPARE
    // OWN_CORNER_EXECUTE
    // OWN_CORNER_PREPARE
    else if (str.find("CORNER") != std::string::npos)
    {
        return SetpieceType::CORNER;
    }

    // OPP_FREEKICK_EXECUTE
    // OPP_FREEKICK_PREPARE
    // OWN_FREEKICK_EXECUTE
    // OWN_FREEKICK_PREPARE
    else if (str.find("FREEKICK") != std::string::npos)
    {
        return SetpieceType::FREEKICK;
    }

    // OPP_GOALKICK_EXECUTE
    // OPP_GOALKICK_PREPARE
    // OWN_GOALKICK_EXECUTE
    // OWN_GOALKICK_PREPARE
    else if (str.find("GOALKICK") != std::string::npos)
    {
        return SetpieceType::GOALKICK;
    }

    // OPP_KICKOFF_EXECUTE
    // OPP_KICKOFF_PREPARE
    // OWN_KICKOFF_EXECUTE
    // OWN_KICKOFF_PREPARE
    else if (str.find("KICKOFF") != std::string::npos)
    {
        return SetpieceType::KICKOFF;
    }

    // OPP_PENALTY_EXECUTE
    // OPP_PENALTY_PREPARE
    // OPP_PENALTY_SHOOTOUT_EXECUTE
    // OPP_PENALTY_SHOOTOUT_PREPARE
    // OWN_PENALTY_EXECUTE
    // OWN_PENALTY_PREPARE
    // OWN_PENALTY_SHOOTOUT_EXECUTE
    // OWN_PENALTY_SHOOTOUT_PREPARE
    else if (str.find("PENALTY") != std::string::npos)
    {
        return SetpieceType::PENALTY;
    }

    // OPP_THROWIN_EXECUTE
    // OPP_THROWIN_PREPARE
    // OWN_THROWIN_EXECUTE
    // OWN_THROWIN_PREPARE
    else if (str.find("THROWIN") != std::string::npos)
    {
        return SetpieceType::THROWIN;
    }

    // INVALID
    // NEUTRAL_PLAYING
    // NEUTRAL_STOPPED
    // OUT_OF_MATCH_NEUTRAL_STOPPED
    else
    {
        return SetpieceType::NONE;
    }
}

SetpieceOwner GameState::fromStringSetpieceOwner(const std::string& str) const
{
    // OWN_CORNER_EXECUTE
    // OWN_CORNER_PREPARE
    // OWN_FREEKICK_EXECUTE
    // OWN_FREEKICK_PREPARE
    // OWN_GOALKICK_EXECUTE
    // OWN_GOALKICK_PREPARE
    // OWN_KICKOFF_EXECUTE
    // OWN_KICKOFF_PREPARE
    // OWN_PENALTY_EXECUTE
    // OWN_PENALTY_PREPARE
    // OWN_PENALTY_SHOOTOUT_EXECUTE
    // OWN_PENALTY_SHOOTOUT_PREPARE
    // OWN_THROWIN_EXECUTE
    // OWN_THROWIN_PREPARE
    if (str.starts_with("OWN"))
    {
        return SetpieceOwner::OWN;
    }

    // OPP_CORNER_EXECUTE
    // OPP_CORNER_PREPARE
    // OPP_FREEKICK_EXECUTE
    // OPP_FREEKICK_PREPARE
    // OPP_GOALKICK_EXECUTE
    // OPP_GOALKICK_PREPARE
    // OPP_KICKOFF_EXECUTE
    // OPP_KICKOFF_PREPARE
    // OPP_PENALTY_EXECUTE
    // OPP_PENALTY_PREPARE
    // OPP_PENALTY_SHOOTOUT_EXECUTE
    // OPP_PENALTY_SHOOTOUT_PREPARE
    // OPP_THROWIN_EXECUTE
    // OPP_THROWIN_PREPARE
    else if (str.starts_with("OPP"))
    {
        return SetpieceOwner::OPPONENT;
    }

    // INVALID
    // NEUTRAL_PLAYING
    // NEUTRAL_STOPPED
    // OUT_OF_MATCH_NEUTRAL_STOPPED
    // PARKING
    // DROPPED_BALL_EXECUTE
    // DROPPED_BALL_PREPARE
    else
    {
        return SetpieceOwner::NONE;
    }
}

void GameState::fromString(const std::string& str)
{
    _governingMatchState = fromStringGoverningMatchState(str);
    _governingGameState = fromStringGoverningGameState(str);
    _setpieceType = fromStringSetpieceType(str);
    _setpieceOwner = fromStringSetpieceOwner(str);
}

/* Function that translates a valid gamestate into a compact string */
std::string GameState::toString() const
{
    std::ostringstream oss;

    switch (_governingMatchState)
    {
    case GoverningMatchState::IN_MATCH:
        oss << "in match ";
        break;
    case GoverningMatchState::OUT_OF_MATCH:
        oss << "out of match ";
        break;
    }

    switch (_setpieceOwner)
    {
    case SetpieceOwner::OWN:
        oss << "own ";
        break;
    case SetpieceOwner::OPPONENT:
        oss << "opp ";
        break;
    case SetpieceOwner::NONE:
    default:
        break;
    }

    switch (_setpieceType)
    {
    case SetpieceType::FREEKICK:
        oss << "freekick ";
        break;
    case SetpieceType::GOALKICK:
        oss << "goalkick ";
        break;
    case SetpieceType::KICKOFF:
        oss << "kickoff ";
        break;
    case SetpieceType::CORNER:
        oss << "corner ";
        break;
    case SetpieceType::THROWIN:
        oss << "throwin ";
        break;
    case SetpieceType::PARKING:
        oss << "park ";
        break;
    case SetpieceType::SHOWING_ALIVE:
        oss << "show alive ";
        break;
    case SetpieceType::PENALTY:
        oss << "penalty ";
        break;
    case SetpieceType::DROPPED_BALL:
        oss << "dropball ";
        break;
    case SetpieceType::NONE:
    default:
        break;
    }

    switch (_governingGameState)
    {
    case GoverningGameState::INVALID:
        oss << "invalid";
        break;
    case GoverningGameState::NEUTRAL_STOPPED:
        oss << "stopped";
        break;
    case GoverningGameState::NEUTRAL_PLAYING:
        oss << "playing";
        break;
    case GoverningGameState::SETPIECE_PREPARING:
        oss << "prep";
        break;
    case GoverningGameState::SETPIECE_EXECUTING:
        oss << "exec";
        break;
    default:
        break;
    }

    return oss.str();
}

/* Function that translates any gamestate (valid or invalid) into a complete string */
std::string GameState::toString(const GoverningGameState& g, const GoverningMatchState& p, const SetpieceOwner& o, const SetpieceType& t) const
{
    std::ostringstream oss;

    oss << "[";
    switch (g)
    {
    case GoverningGameState::INVALID:
        oss << "invalid";
        break;
    case GoverningGameState::NEUTRAL_STOPPED:
        oss << "neutral stopped";
        break;
    case GoverningGameState::NEUTRAL_PLAYING:
        oss << "neutral playing";
        break;
    case GoverningGameState::SETPIECE_PREPARING:
        oss << "setpiece preparing";
        break;
    case GoverningGameState::SETPIECE_EXECUTING:
        oss << "setpiece executing";
        break;
    default:
        oss << "UNKNOWN";
        break;
    }

    oss << ", ";
    switch (p)
    {
    case GoverningMatchState::IN_MATCH:
        oss << "in match";
        break;
    case GoverningMatchState::OUT_OF_MATCH:
        oss << "out of match";
        break;
    default:
        oss << "UNKNOWN";
        break;
    }

    oss << ", ";
    switch (o)
    {
    case SetpieceOwner::NONE:
        oss << "no owner";
        break;
    case SetpieceOwner::OWN:
        oss << "own";
        break;
    case SetpieceOwner::OPPONENT:
        oss << "opponent";
        break;
    default:
        oss << "UNKNOWN";
        break;
    }

    oss << ", ";
    switch (t)
    {
    case SetpieceType::NONE:
        oss << "no setpiece type";
        break;
    case SetpieceType::FREEKICK:
        oss << "freekick";
        break;
    case SetpieceType::GOALKICK:
        oss << "goalkick";
        break;
    case SetpieceType::KICKOFF:
        oss << "kickoff";
        break;
    case SetpieceType::CORNER:
        oss << "corner";
        break;
    case SetpieceType::THROWIN:
        oss << "throw-in";
        break;
    case SetpieceType::PARKING:
        oss << "park";
        break;
    case SetpieceType::SHOWING_ALIVE:
        oss << "show alive";
        break;
    case SetpieceType::PENALTY:
        oss << "penalty";
        break;
    case SetpieceType::DROPPED_BALL:
        oss << "dropped Ball";
        break;
    default:
        oss << "UNKNOWN";
        break;
    }

    oss << "]";
    return oss.str();
}
