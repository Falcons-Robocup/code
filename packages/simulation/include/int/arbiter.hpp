// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * arbiter.hpp
 *
 *  Created on: Jan 10, 2019
 *      Author: Coen Tempelaars
 */

#ifndef ARBITER_HPP_
#define ARBITER_HPP_

#include "arbiterGameData.hpp"
#include "gameState.hpp"
#include "judgement.hpp"
#include "preparingController.hpp"
#include "runningController.hpp"
#include "stoppingController.hpp"
#include "stoppedController.hpp"

class Arbiter {
public:
    Arbiter();

    void initialize();
    GameData control(const GameData&, const float simulationPeriod);

    GameState getGameState() const;

protected:
    AbstractRefBoxAdapter* _refBoxAdapter;

private:
    PreparingController _preparingController;
    RunningController _runningController;
    StoppingController _stoppingController;
    StoppedController _stoppedController;

    void goalSignalHandler(const TeamID&);
    void preparingSignalHandler();
    void preparedSignalHandler();
    void stoppingSignalHandler();
    void stoppingSignalHandler(const Judgement&);
    void stoppedSignalHandler();

    ArbiterGameData _arbiterGameData;
    GameState _gameState;
    Judgement _lastJudgement;
    float _secondsSinceLastTransition;
};

#endif /* ARBITER_HPP_ */
