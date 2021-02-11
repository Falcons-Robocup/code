// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworldGameDataFactory.cpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#include <int/simworldGameDataFactory.hpp>

SimworldGameData SimworldGameDataFactory::createCompleteWorld (const GameData& gameData)
{
    SimworldGameData simworldGameData;

    simworldGameData.ball = gameData.ball;
    simworldGameData.team = gameData.team;

    return simworldGameData;
}
