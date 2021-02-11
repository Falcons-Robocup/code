// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameDataFactory.hpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#ifndef GAMEDATAFACTORY_HPP_
#define GAMEDATAFACTORY_HPP_

#include "gameData.hpp"

class GameDataFactory {
public:
    static GameData createGameData(const int, const int);
};

#endif /* GAMEDATAFACTORY_HPP_ */
