// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * runningController.cpp
 *
 *  Created on: Dec 29, 2018
 *      Author: Coen Tempelaars
 */

#include "int/runningController.hpp"

#include "cEnvironmentField.hpp"
#include "tracing.hpp"


const static float minimumWaitingTime =  5.0; //seconds

static Vector2D getPositionNoise()
{
    /* Returns a Vector2D between (-0.5, -0.5) and (0.5, 0.5) */
    double random_x = ( (std::rand() % 1000000) * 0.000001 ) - 0.5;
    double random_y = ( (std::rand() % 1000000) * 0.000001 ) - 0.5;
    return Vector2D(random_x, random_y);
}

boost::signals2::connection RunningController::goalSignalSubscribe (const goalSignal_t::slot_type& subscriber)
{
    return _goalSignal.connect(subscriber);
}

boost::signals2::connection RunningController::stoppingSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _stoppingSignal.connect(subscriber);
}

void RunningController::declareGameStopping(const Judgement& judgement)
{
    if (!_signalSent)
    {
        // raise the 'stopping' signal
        _stoppingSignal(judgement);

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void RunningController::control (const ArbiterGameData& gamedata)
{
    control(gamedata, 0.0);
}

void RunningController::control (const ArbiterGameData& gamedata, const float secondsSinceLastTransition)
{
    TRACE_FUNCTION("");
    _signalSent = false;

    if (gamedata.ballIsInNegativeGoal())
    {
        _goalSignal(TeamID::B);

        Judgement judgement;
        poiInfo p;
        cEnvironmentField::getInstance().getFieldPOI(P_CENTER, p);
        judgement.ballPosition = Point2D(p.x, p.y);
        judgement.setpiece = Setpiece::KICKOFF;
        judgement.teamID = TeamID::A;
        declareGameStopping(judgement);
    }

    if (gamedata.ballIsInPositiveGoal())
    {
        _goalSignal(TeamID::A);

        Judgement judgement;
        poiInfo p;
        cEnvironmentField::getInstance().getFieldPOI(P_CENTER, p);
        judgement.ballPosition = Point2D(p.x, p.y);
        judgement.setpiece = Setpiece::KICKOFF;
        judgement.teamID = TeamID::B;
        declareGameStopping(judgement);
    }

    if (gamedata.ballIsAcrossNegativeGoalline())
    {
        Judgement judgement;

        if (gamedata.teamLastHoldingBall == TeamID::A)
        {
            poiInfo p;
            cEnvironmentField::getInstance().getFieldPOI(P_OWN_CORNER_LEFT, p);
            judgement.ballPosition = Point2D(p.x + 0.5, p.y + 0.5);
            judgement.setpiece = Setpiece::CORNER;
            judgement.teamID = TeamID::B;
        }

        if (gamedata.teamLastHoldingBall == TeamID::B)
        {
            poiInfo p;
            cEnvironmentField::getInstance().getFieldPOI(P_OWN_GOALAREA_GOALLINE_LEFT, p);
            judgement.ballPosition = Point2D(p.x - 1.25, p.y + 2.0);
            judgement.setpiece = Setpiece::GOALKICK;
            judgement.teamID = TeamID::A;
        }

        declareGameStopping(judgement);
    }

    if (gamedata.ballIsAcrossPositiveGoalline())
    {
        Judgement judgement;

        if (gamedata.teamLastHoldingBall == TeamID::A)
        {
            poiInfo p;
            cEnvironmentField::getInstance().getFieldPOI(P_OPP_GOALAREA_GOALLINE_RIGHT, p);
            judgement.ballPosition = Point2D(p.x + 1.25, p.y - 2.0);
            judgement.setpiece = Setpiece::GOALKICK;
            judgement.teamID = TeamID::B;
        }

        if (gamedata.teamLastHoldingBall == TeamID::B)
        {
            poiInfo p;
            cEnvironmentField::getInstance().getFieldPOI(P_OPP_CORNER_RIGHT, p);
            judgement.ballPosition = Point2D(p.x - 0.5, p.y - 0.5);
            judgement.setpiece = Setpiece::CORNER;
            judgement.teamID = TeamID::A;
        }

        declareGameStopping(judgement);
    }

    if (gamedata.ballIsAcrossSideLine())
    {
        Judgement judgement;
        judgement.ballPosition = gamedata.ball.getLocation() * 0.8;
        judgement.setpiece = Setpiece::THROWIN;
        judgement.teamID = otherTeam(gamedata.teamLastHoldingBall);
        declareGameStopping(judgement);
    }

    if (secondsSinceLastTransition > minimumWaitingTime)
    {
        if (gamedata.isScrum())
        {
            Judgement judgement;
            judgement.ballPosition = gamedata.ball.getLocation() * 0.8 + getPositionNoise();
            judgement.setpiece = Setpiece::DROPPED_BALL;
            declareGameStopping(judgement);
        }
    }
}
