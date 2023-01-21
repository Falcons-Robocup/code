// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameStateStore.cpp
 *
 *  Created on: Jul 26, 2016
 *      Author: Coen Tempelaars
 */

#include "int/stores/GameStateStore.hpp"

using namespace teamplay;

GameStateStore::GameStateStore()
{
    _theGameState = GameState(GoverningGameState::NEUTRAL_STOPPED);
}

GameStateStore::~GameStateStore()
{
}

const GameState& GameStateStore::getGameState() const
{
    return _theGameState;
}

void GameStateStore::updateGameState(const GameState& g)
{
    _theGameState = g;
}

void GameStateStore::updateGameState(const GoverningGameState& g)
{
    _theGameState = GameState(g);
}

void GameStateStore::updateGameState(const GoverningGameState& g, const GoverningMatchState& m, const SetpieceOwner& o, const SetpieceType& t)
{
    _theGameState = GameState(g, m, o, t);
}
