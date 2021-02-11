// Copyright 2016-2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameStateStore.cpp
 *
 *  Created on: Jul 26, 2016
 *      Author: Coen Tempelaars
 */

#include "int/stores/gameStateStore.hpp"

using namespace teamplay;

gameStateStore::gameStateStore()
{
    _theGameState = gameState(governingGameState::NEUTRAL_STOPPED);
}

gameStateStore::~gameStateStore()
{
}

treeEnum gameStateStore::getGameState_treeEnum() const
{
    return _theGameState.toTreeEnum();
}

const gameState& gameStateStore::getGameState() const
{
    return _theGameState;
}

void gameStateStore::updateGameState(const gameState& g)
{
    _theGameState = g;
}

void gameStateStore::updateGameState(const governingGameState& g)
{
    _theGameState = gameState(g);
}

void gameStateStore::updateGameState(const governingGameState& g, const setpieceOwner& o, const setpieceType& t)
{
    _theGameState = gameState(g, governingMatchState::IN_MATCH, o, t);
}

void gameStateStore::updateGameState(const treeEnum& t)
{
    _theGameState = gameState(t);
}
