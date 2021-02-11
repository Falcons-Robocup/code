// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworldGameDataFactory.hpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#ifndef SIMWORLDGAMEDATAFACTORY_HPP_
#define SIMWORLDGAMEDATAFACTORY_HPP_

#include "simworldGameData.hpp"

class SimworldGameDataFactory {
public:
    static SimworldGameData createCompleteWorld (const GameData&);
};

#endif /* SIMWORLDGAMEDATAFACTORY_HPP_ */
