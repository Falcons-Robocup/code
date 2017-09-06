 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameStateManager.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

/* Own include */
#include "int/gameStateManager.hpp"

/* Other teamplay includes */
#include "int/stores/gameStateStore.hpp"
#include "int/gameStateTransitionTable.hpp"
#include "int/rules/ruleSetpieceExecuteFinished.hpp"
#include "int/rules/ruleStimulatePassing.hpp"
#include "int/utilities/timer.hpp"
#include "int/utilities/trace.hpp"

/* System includes */
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

using namespace teamplay;

/* Static definitions */
static boost::scoped_ptr<ruleSetpieceExecuteFinished> rule_p;
static boost::shared_ptr<timer> timer_p;


gameStateManager::gameStateManager()
{
    timer_p.reset(new timer());
    rule_p.reset(new ruleSetpieceExecuteFinished(timer_p));

    ruleStimulatePassing::getInstance().resetRule();
}

gameStateManager::~gameStateManager()
{
    rule_p.reset();
    timer_p.reset();
}

void gameStateManager::refreshGameState()
{
    gameState currentGameState = gameStateStore::getInstance().getGameState();

    // If the current gamestate is of type "setpiece execute"...
    // .. and we're within a match
    // (Outside of match we can never to back to neutral playing, which is an in-match-only state. The only thing that can get you back in a match is a kickoff.)
    if (currentGameState.isInMatch() && currentGameState.isSetPiece() && !currentGameState.isPrepareSetPiece())
    {
        // If the 'setpiece finished' rule is valid, the gamestate must be set to neutral playing
        if (rule_p->isRuleValid())
        {
            gameStateStore::getInstance().updateGameState(governingGameState::NEUTRAL_PLAYING);
        }
    }
}

void gameStateManager::refBoxSignalReceived(const refboxSignalEnum& refboxSignal)
{
    gameState currentGameState = gameStateStore::getInstance().getGameState();
    gameState newGameState = gameStateTransitionTable::getInstance().calculateNewGameState(currentGameState, refboxSignal);

    // If the new gamestate is of type "setpiece execute"...
    if (newGameState.isSetPiece() && !newGameState.isPrepareSetPiece())
    {
        // If the gamestate has changed, the 'setpiece finished' rule must be reset
        if (currentGameState != newGameState)
        {
            rule_p->resetRule();
        }

        // If the 'setpiece finished' rule is valid, the gamestate must be set to neutral playing
        // But only inside a match
        // (Outside of match we can never to back to neutral playing, which is an in-match-only state. The only thing that can get you back in a match is a kickoff.)
        if (currentGameState.isInMatch() && rule_p->isRuleValid())
        {
            newGameState = gameState(governingGameState::NEUTRAL_PLAYING);
        }
    }

    // If the new gamestate is stopped, reset the 'stimulate passing' rule
    if (newGameState.isStopped())
    {
        ruleStimulatePassing::getInstance().resetRule();
    }

    // If the gamestate has changed, store it
    if (currentGameState != newGameState)
    {
        gameStateStore::getInstance().updateGameState(newGameState);
        TRACE("gameStateManager::refBoxSignalReceived - updated the gamestate to ") << newGameState.toString();
    }
    else
    {
        TRACE("gameStateManager::refBoxSignalReceived - gamestate already was ") << newGameState.toString() << ". No update performed.";
    }
}
