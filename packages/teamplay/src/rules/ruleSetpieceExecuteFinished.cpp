// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleSetpieceExecuteFinished.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#include "int/rules/ruleSetpieceExecuteFinished.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/gameStateStore.hpp"

using namespace teamplay;

ruleSetpieceExecuteFinished::ruleSetpieceExecuteFinished(boost::shared_ptr<teamplay::timer> t)
{
    _timerAtStartRule = t;
    resetRule();
}

ruleSetpieceExecuteFinished::~ruleSetpieceExecuteFinished()
{
    _timerAtStartRule.reset();  //This releases the shared pointer
}

void ruleSetpieceExecuteFinished::resetRule()
{
    _timerAtStartRule->reset();  //This sets the timer to 'now'

    _ballAtStartRule = ballStore::getBall();
    _gameStateAtStartRule = gameStateStore::getInstance().getGameState();

    if (_gameStateAtStartRule.isOwnSetPiece() && _gameStateAtStartRule.isKickoffSetPiece())
    {
        _ballMinDistance = configurationStore::getConfiguration().getMinOwnKickoffDistanceKickedMeters();
        _timerMaxValue = configurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds();
    }
    else if (_gameStateAtStartRule.isPenaltySetPiece())
    {
        _ballMinDistance = configurationStore::getConfiguration().getMinPenaltyDistanceKickedMeters();
        _timerMaxValue = configurationStore::getConfiguration().getPenaltyExecuteTimeoutSeconds();
    }
    else
    {
        _ballMinDistance = configurationStore::getConfiguration().getMinKickDistanceKickedMeters();
        _timerMaxValue = configurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds();
    }
}

bool ruleSetpieceExecuteFinished::isRuleValid()
{
    gameState currentGameState = gameStateStore::getInstance().getGameState();

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
            ball currentBall = ballStore::getBall();
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
