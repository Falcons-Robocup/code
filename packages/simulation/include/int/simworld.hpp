// Copyright 2019-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworld.hpp
 *
 *  Created on: Feb 2, 2019
 *      Author: Coen Tempelaars
 */

#ifndef SIMWORLD_HPP_
#define SIMWORLD_HPP_

#include <thread>
#include <map>
#include <boost/thread.hpp>

#include "abstractConfigAdapter.hpp"
#include "abstractGameDataAdapter.hpp"
#include "abstractMotionAdapter.hpp"
#include "abstractTimeAdapter.hpp"
#include "arbiter.hpp"
#include "simworldGameData.hpp"

class Simworld {
public:
    Simworld();
    ~Simworld();

    void initialize(const std::string& arbiter, int sizeTeamA, int sizeTeamB);
    void control();

protected:
    std::unique_ptr<AbstractConfigAdapter> _configAdapter;
    std::unique_ptr<AbstractGameDataAdapter> _gameDataAdapter;
    std::unique_ptr<AbstractMotionAdapter> _motionAdapter;
    std::unique_ptr<AbstractTimeAdapter> _timeAdapter;

private:
    Arbiter _arbiter;
    SimworldGameData _simworldGameData;

    rtime _simulationTime;
    int _sizeTeamA;
    int _sizeTeamB;
    bool _autorefEnabled;

    // This vector contains for every robot a thread that is waiting for PUT to synchronize the end of the heartbeat
    class HeartbeatWaiter;
    std::map< std::pair<TeamID, RobotID>, std::unique_ptr<HeartbeatWaiter> > _robotHeartbeatWaiters;

    // The number of updates/recalculations of the simulated world per second
    int _tickFrequency;

    // The number of seconds to advance time in a single update (tick)
    double _tick_stepsize_s;

    // The number of milliseconds to sleep inbetween triggering a simulated robot
    // = (1000 / _tickFrequency) / (numRobots + arbiter)
    int _sleeptime_ms;
};

#endif /* SIMWORLD_HPP_ */
