// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleSetpieceExecuteFinished.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#include "int/rules/RuleSetpieceExecuteFinished.hpp"

#include "int/stores/BallStore.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/GameStateStore.hpp"

using namespace teamplay;

RuleSetpieceExecuteFinished::RuleSetpieceExecuteFinished(boost::shared_ptr<teamplay::Timer> t)
{
    _timerAtStartRule = t;
    resetRule();
}

RuleSetpieceExecuteFinished::~RuleSetpieceExecuteFinished()
{
    _timerAtStartRule.reset();  //This releases the shared pointer
}

void RuleSetpieceExecuteFinished::resetRule()
{
    _timerAtStartRule->reset();  //This sets the timer to 'now'

    _ballAtStartRule = BallStore::getBall();
    _gameStateAtStartRule = GameStateStore::getInstance().getGameState();

    if (_gameStateAtStartRule.isOwnSetPiece() && _gameStateAtStartRule.isKickoffSetPiece())
    {
        _ballMinDistance = ConfigurationStore::getConfiguration().getMinOwnKickoffDistanceKickedMeters();
        _timerMaxValue = ConfigurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds();
    }
    else if (_gameStateAtStartRule.isPenaltySetPiece())
    {
        _ballMinDistance = ConfigurationStore::getConfiguration().getMinPenaltyDistanceKickedMeters();
        _timerMaxValue = ConfigurationStore::getConfiguration().getPenaltyExecuteTimeoutSeconds();
    }
    else
    {
        _ballMinDistance = ConfigurationStore::getConfiguration().getMinKickDistanceKickedMeters();
        _timerMaxValue = ConfigurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds();
    }
}

bool RuleSetpieceExecuteFinished::isRuleValid()
{
    GameState currentGameState = GameStateStore::getInstance().getGameState();

    if (currentGameState.isDroppedBallSetPiece())
    {
        return true;
    }

    if (currentGameState == _gameStateAtStartRule)
    {
        if(_timerAtStartRule->hasElapsed(_timerMaxValue))
        {
            return true;
        }
        else
        {
            Ball currentBall = BallStore::getBall();
            if(currentBall.isLocationKnown())
            {
                if (_ballAtStartRule.isLocationKnown())
                {
                    Vector3D ballMovedDistance = currentBall.getPosition() - _ballAtStartRule.getPosition();
                    if(vectorsize(ballMovedDistance) > _ballMinDistance)
                    {
                        return true;
                    }
                }
                else
                {
                    /* the location of the ball was not known at start rule, but now it is:
                     * refine the ball location at start rule by setting it to the current ball location */
                    _ballAtStartRule = currentBall;
                }
            }
        }
    }
    else /* currentGameState != _gameStateAtStartRule */
    {
        resetRule();
    }

    return false;
}
