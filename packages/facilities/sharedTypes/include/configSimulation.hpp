// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGSIMULATION_HPP_
#define CONFIGSIMULATION_HPP_

#include "RtDB2.h" // required for serialization
#include <string>


struct configSimulation
{
    std::string arbiter;

    int sizeTeamA;
    int sizeTeamB;

    // The number of milliseconds to advance time in a single update (tick), e.g., 50ms == 20Hz
    int tick_stepsize_ms;

    // The frequency of updating (ticking) simulation (heartbeat), e.g., 20Hz -> 20 updates per second.
    // Increasing/decreasing this number will influence the simulation speed (e.g., 40 == 2x)
    int tick_frequency;

    SERIALIZE_DATA(arbiter, sizeTeamA, sizeTeamB, tick_stepsize_ms, tick_frequency);
};

#endif
