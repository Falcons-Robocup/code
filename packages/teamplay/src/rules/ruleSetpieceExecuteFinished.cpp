 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
