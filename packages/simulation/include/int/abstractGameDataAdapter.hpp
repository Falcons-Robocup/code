// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractGameDataAdapter.hpp
 *
 *  Created on: Jan 16, 2019
 *      Author: Coen Tempelaars
 */

#ifndef ABSTRACTGAMEDATAADAPTER_HPP_
#define ABSTRACTGAMEDATAADAPTER_HPP_

#include "gameData.hpp"
#include "vector2d.hpp"
#include "SimulationScene.hpp"

class AbstractGameDataAdapter {
public:
    virtual ~AbstractGameDataAdapter() {}

    virtual void publishGameData (const GameData&) const = 0;
    virtual void publishGameData (const GameData&, const TeamID, const RobotID) const = 0;

    // scene manipulation
    virtual void publishScene (const GameData&) const = 0;
    virtual bool checkUpdatedScene(SimulationScene &scene) = 0;

};

#endif /* ABSTRACTGAMEDATAADAPTER_HPP_ */
