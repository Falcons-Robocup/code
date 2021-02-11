// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworld.hpp
 *
 *  Created on: Feb 2, 2019
 *      Author: Coen Tempelaars
 */

#ifndef SIMWORLD_HPP_
#define SIMWORLD_HPP_

#include "abstractConfigAdapter.hpp"
#include "abstractGameDataAdapter.hpp"
#include "abstractMotionAdapter.hpp"
#include "abstractTimeAdapter.hpp"
#include "arbiter.hpp"
#include "simworldGameData.hpp"

class Simworld {
public:
    Simworld();

    void initialize();
    void control();
    void loop();

protected:
    AbstractConfigAdapter* _configAdapter;
    AbstractGameDataAdapter* _gameDataAdapter;
    AbstractMotionAdapter* _motionAdapter;
    AbstractTimeAdapter* _timeAdapter;

private:
    Arbiter _arbiter;
    SimworldGameData _simworldGameData;

    rtime _simulationTime;
    int _sizeTeamA;
    int _sizeTeamB;

    // The number of updates/recalculations of the simulated world per second
    int _tickFrequency;

    // The number of seconds to advance time in a single update (tick)
    double _tick_stepsize_s;

    // The number of milliseconds to sleep inbetween triggering a simulated robot
    // = (1000 / _tickFrequency) / (numRobots + arbiter)
    int _sleeptime_ms;
};

#endif /* SIMWORLD_HPP_ */
