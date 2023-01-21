// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateManager.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

/* Own include */
#include "int/gamestate/GameStateManager.hpp"

/* Other teamplay includes */
#include "int/stores/GameStateStore.hpp"
#include "int/gamestate/GameStateTransitionTable.hpp"
#include "int/rules/RuleSetpieceExecuteFinished.hpp"
#include "int/rules/RuleStimulatePassing.hpp"
#include "int/utilities/Timer.hpp"

/* Falcons includes */
#include "falconsCommon.hpp" // getRobotNumber
#include "tracing.hpp"

/* System includes */
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

using namespace teamplay;

/* Static definitions */
static boost::scoped_ptr<RuleSetpieceExecuteFinished> rule_p;
static boost::shared_ptr<Timer> timer_p;


GameStateManager::GameStateManager()
{
    timer_p.reset(new Timer());
    rule_p.reset(new RuleSetpieceExecuteFinished(timer_p));

    RuleStimulatePassing::getInstance().resetRule();
}

GameStateManager::~GameStateManager()
{
    rule_p.reset();
    timer_p.reset();
}

void GameStateManager::refreshGameState()
{
    GameState currentGameState = GameStateStore::getInstance().getGameState();

    tprintf("Current gamestate: %s", currentGameState.toString().c_str());

    // If the current gamestate is of type "setpiece execute"...
    // .. and we're within a match
    // (Outside of match we can never to back to neutral playing, which is an in-match-only state. The only thing that can get you back in a match is a kickoff.)
    if (currentGameState.isInMatch() && currentGameState.isSetPiece() && !currentGameState.isPrepareSetPiece())
    {
        // If the 'setpiece finished' rule is valid, the gamestate must be set to neutral playing
        if (rule_p->isRuleValid())
        {
            // Unless we are executing the showing alive setpiece, which is the only one initiated from the stopped state
            if (currentGameState.isShowingAliveSetPiece())
            {
                GameStateStore::getInstance().updateGameState(GoverningGameState::NEUTRAL_STOPPED);
            }
            else
            {
                GameStateStore::getInstance().updateGameState(GoverningGameState::NEUTRAL_PLAYING);
            }
        }
    }
}

void GameStateManager::refBoxSignalReceived(const RefboxSignalEnum& refboxSignal, const RefboxSignalArguments& refboxSignalArguments)
{
    GameState currentGameState = GameStateStore::getInstance().getGameState();
    GameState newGameState = GameStateTransitionTable::getInstance().calculateNewGameState(currentGameState, refboxSignal);

    // Special case: substitution should only affect gamestate for targeted robots (in refboxSignalArgument)
    // The command SUBSTITUTION_OWN is basically treated the same as PARK
    // except that we need to apply it conditionally, all non-targeted robots should remain in their state
    if (refboxSignal == RefboxSignalEnum::SUBSTITUTION_OWN || refboxSignal == RefboxSignalEnum::IS_ALIVE_OWN)
    {
        // check if this robot is targeted

        std::string target = refboxSignalArguments.at("robotID");
        if (target != std::to_string(getRobotNumber()))
        {
            // do not go into parking mode, instead keep current state
            newGameState = currentGameState;
        }
    }


    tprintf("New gamestate from refbox signal: %s", newGameState.toString().c_str());

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
            newGameState = GameState(GoverningGameState::NEUTRAL_PLAYING);
        }
    }

    // If the new gamestate is stopped, reset the 'stimulate passing' rule
    if (newGameState.isStopped())
    {
        RuleStimulatePassing::getInstance().resetRule();
    }

    // If the gamestate has changed, store it
    if (currentGameState != newGameState)
    {
        GameStateStore::getInstance().updateGameState(newGameState);
        TRACE("GameStateManager::refBoxSignalReceived - updated the gamestate to ") << newGameState.toString();
    }
    else
    {
        TRACE("GameStateManager::refBoxSignalReceived - gamestate already was ") << newGameState.toString() << ". No update performed.";
    }
}