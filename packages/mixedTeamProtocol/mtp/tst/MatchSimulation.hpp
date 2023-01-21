// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_TST_MATCHSIMULATION_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_TST_MATCHSIMULATION_HPP_

// headers from this package
#include "RobotClient.hpp"

// standard/system headers
#include <vector>
#include <string>


// a simple simulation of a partial match, using multiple simulated robots
class MatchSimulation
{
    public:
        MatchSimulation(float frequency = 10.0);
        ~MatchSimulation();

    public:
        // test setup
        RobotClient &addRobot(mtp::PlayerId const &playerId, float frequency = 10.0, float jitter = 0.0);
        RobotClient &getRobot(mtp::PlayerId const &playerId);

        // runtime match simulation
        void advanceTick();
        void advanceTicks(int ticks);
        void advanceDuration(float duration);
        void synchronize();

        // state / result inspection, see also MatchSimulationChecks
        void reportHeading() const;
        void reportTick() const;
        std::vector<mtp::PlayerId> getPlayers() const;

    private:
        bool _verbose = true;
        std::map<mtp::PlayerId, RobotClient> _robots;
        rtime _t0, _tc;
        float _tstep = 0.1;
};

#endif
