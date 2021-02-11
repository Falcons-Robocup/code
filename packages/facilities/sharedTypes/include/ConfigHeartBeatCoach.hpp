// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGHEARTBEATCOACH_HPP_
#define CONFIGHEARTBEATCOACH_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigHeartBeatCoach
{
    float updateFrequency = 30.0;

    SERIALIZE_DATA(updateFrequency);
};

#endif

