// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef JOYSTICKCONTROL_HPP_
#define JOYSTICKCONTROL_HPP_

#include "robotVelocity.hpp"

#include "RtDB2.h" // required for serialization


struct joystickControl
{
    int           robotId; // which robot to control
    robotVelocity velocity; // RCS
    bool          bhEnabled; // ballHandlers
    float         kickerHeight;
    float         kickerPower;
    std::string   action; // some string that robotCLI understands (getBall, shootAtGoal, ...)

    SERIALIZE_DATA(robotId, velocity, bhEnabled, kickerHeight, kickerPower);
};

#endif

