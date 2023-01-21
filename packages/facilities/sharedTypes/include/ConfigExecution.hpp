// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGEXECUTION_HPP_
#define CONFIGEXECUTION_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigExecution
{
    float frequency = 40.0;                 // [Hz]     The frequency of the heartbeat (tick), e.g., 40Hz -> 40 ticks per second.
    float simulationSpeedupFactor = 1.0;    // [double] The speedup factor for simulation, e.g., 2.0 -> simulation time advances at 200%: 1 real-world second == 2 seconds in simulation time
    std::string tickFinishRtdbKey = "ROBOT_VELOCITY_SETPOINT";   // [string] The RtDB key that is written when a tick / heartbeat finishes. Execution will subscribe to this RtDB key.

    SERIALIZE_DATA(frequency, simulationSpeedupFactor, tickFinishRtdbKey);
};

#endif

