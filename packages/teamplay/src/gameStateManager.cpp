// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include "tracing.hpp"

/* Falcons includes */
#include "falconsCommon.hpp" // getRobotNumber

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

void gameStateManager::refBoxSignalReceived(const refboxSignalEnum& refboxSignal, std::string refboxSignalArgument)
{
    gameState currentGameState = gameStateStore::getInstance().getGameState();
    gameState newGameState = gameStateTransitionTable::getInstance().calculateNewGameState(currentGameState, refboxSignal);

    // Special case: substitution should only affect gamestate for targeted robots (in refboxSignalArgument)
    // The command SUBSTITUTION_OWN is basically treated the same as PARK
    // except that we need to apply it conditionally, all non-targeted robots should remain in their state
    if (refboxSignal == refboxSignalEnum::SUBSTITUTION_OWN)
    {
        // check if this robot is targeted
        refboxSignalArgument = " " + refboxSignalArgument + " ";
        bool targeted = (refboxSignalArgument.find(" " + std::to_string(getRobotNumber())+ " ") != std::string::npos);
        if (!targeted)
        {
            // do not go into parking mode, instead keep current state
            newGameState = currentGameState;
        }
    }

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
