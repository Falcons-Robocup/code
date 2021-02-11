// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBGameDataAdapter.hpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Martijn van Veen
 */

#ifndef RTDBGAMEDATAADAPTER_HPP_
#define RTDBGAMEDATAADAPTER_HPP_

#include <thread>
#include "abstractGameDataAdapter.hpp"

class RTDBgameDataAdapter : public AbstractGameDataAdapter {
public:
    RTDBgameDataAdapter();
    ~RTDBgameDataAdapter();
    virtual void publishGameData (const GameData&) const; // publish complete world
    virtual void publishGameData (const GameData&, const TeamID, const RobotID) const; // publish adapted world

    // scene manipulation
    virtual void publishScene (const GameData&) const;
    virtual bool checkUpdatedScene(SimulationScene &scene);
    void monitorScene();
public:
    bool _sceneUpdated = false;
    std::thread _sceneMonitorThread;
};

#endif /* RTDBGAMEDATAADAPTER_HPP_ */
