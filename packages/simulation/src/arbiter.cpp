// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * arbiter.cpp
 *
 *  Created on: Jan 10, 2019
 *      Author: Coen Tempelaars
 */

#include <stdexcept>
#include <iostream>

#include "int/arbiter.hpp"
#include "int/RTDBrefboxAdapter.hpp"
#include "tracing.hpp"

static RTDBRefBoxAdapter rtdbRefBoxAdapter;


Arbiter::Arbiter()
: _refBoxAdapter(&rtdbRefBoxAdapter)
, _gameState(GameState::STOPPED)
, _secondsSinceLastTransition(0.0)
{
    _lastJudgement.ballPosition = Point2D(0.0, 0.0);
    _lastJudgement.setpiece = Setpiece::KICKOFF;
    _lastJudgement.teamID = TeamID::A;

    _preparingController.preparedSignalSubscribe(boost::bind(&Arbiter::preparedSignalHandler, this));
    _preparingController.stoppingSignalSubscribe(boost::bind(&Arbiter::stoppingSignalHandler, this));
    _runningController.goalSignalSubscribe(boost::bind(&Arbiter::goalSignalHandler, this, _1));
    _runningController.stoppingSignalSubscribe(boost::bind(&Arbiter::stoppingSignalHandler, this, _1));
    _stoppingController.stoppedSignalSubscribe(boost::bind(&Arbiter::stoppedSignalHandler, this));
    _stoppedController.preparingSignalSubscribe(boost::bind(&Arbiter::preparingSignalHandler, this));
}

void Arbiter::initialize()
{
    TRACE_FUNCTION("");

    /* Sanity checks */
    if (_refBoxAdapter == nullptr)
    {
        throw std::runtime_error("Error: refBoxAdapter is null");
    }

    try {
        _refBoxAdapter->sendStop();
    }
    catch (std::exception& e) {
        std::cout << "Error sending refbox stop signal: " << e.what() << std::endl;
    }
}

GameData Arbiter::control(const GameData& gameData, const float simulationPeriod)
{
    TRACE_FUNCTION("");

    /* Sanity checks */
    if (_refBoxAdapter == nullptr)
    {
        throw std::runtime_error("Error: refBoxAdapter is null");
    }

    /* Refresh the gamedata */
    try
    {
        _arbiterGameData.refresh(gameData);
    }
    catch (std::exception& e) {
        std::cout << "Error refreshing gamedata: " << e.what() << std::endl;
    }

    /* Request the appropriate controller to control the game */
    switch (_gameState)
    {
    case GameState::PREPARING:
        _preparingController.control(_arbiterGameData, _secondsSinceLastTransition);
        break;

    case GameState::RUNNING:
        _runningController.control(_arbiterGameData, _secondsSinceLastTransition);
        break;

    case GameState::STOPPING:
        _stoppingController.control(_arbiterGameData, _secondsSinceLastTransition);
        break;

    case GameState::STOPPED:
        _stoppedController.control(_arbiterGameData);
        break;

    default:
        break;
    }

    /* Republish the last refbox command */
    try {
        _refBoxAdapter->republish();
    }
    catch (std::exception& e) {
        std::cout << "Error republishing last refbox command: " << e.what() << std::endl;
    }

    /* Progress time */
    _secondsSinceLastTransition += simulationPeriod;

    /* Return the (potentially updated) gamestate */
    return _arbiterGameData;
}

GameState Arbiter::getGameState() const
{
    return _gameState;
}

void Arbiter::goalSignalHandler(const TeamID& teamID)
{
    TRACE_FUNCTION("");

    try {
        _refBoxAdapter->registerGoal(teamID);
    }
    catch (std::exception& e) {
        std::cout << "Error registering goal: " << e.what() << std::endl;
    }
}

void Arbiter::preparingSignalHandler()
{
    TRACE_FUNCTION("");

    // Reset the time since last transition
    _secondsSinceLastTransition = 0.0;

    // Teleport the ball
    _arbiterGameData.ball.setLocation(_lastJudgement.ballPosition);
    _arbiterGameData.ball.stopMoving();

    // Send the appropriate refbox command
    try {
        _refBoxAdapter->sendSetpiece(_lastJudgement.setpiece, _lastJudgement.teamID);
    }
    catch (std::exception& e) {
        std::cout << "Error publishing refbox command: " << e.what() << std::endl;
    }

    // Finally store the gamestate
    _gameState = GameState::PREPARING;
}

void Arbiter::preparedSignalHandler()
{
    TRACE_FUNCTION("");

    // Reset the time since last transition
    _secondsSinceLastTransition = 0.0;

    // Send the appropriate refbox command
    try {
        _refBoxAdapter->sendStart();
    }
    catch (std::exception& e) {
        std::cout << "Error publishing refbox command: " << e.what() << std::endl;
    }

    // Finally store the gamestate
    _gameState = GameState::RUNNING;
}

void Arbiter::stoppingSignalHandler()
{
    TRACE_FUNCTION("");

    // Reset the time since last transition
    _secondsSinceLastTransition = 0.0;

    // Send the appropriate refbox command
    try {
        _refBoxAdapter->sendStop();
    }
    catch (std::exception& e) {
        std::cout << "Error publishing refbox command: " << e.what() << std::endl;
    }

    // Finally store the gamestate
    _gameState = GameState::STOPPING;
}

void Arbiter::stoppingSignalHandler(const Judgement& judgement)
{
    _lastJudgement = judgement;
    stoppingSignalHandler();
}

void Arbiter::stoppedSignalHandler()
{
    TRACE_FUNCTION("");

    // Reset the time since last transition
    _secondsSinceLastTransition = 0.0;

    _gameState = GameState::STOPPED;
}
