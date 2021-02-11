// Copyright 2019 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef RTDBINPUTADAPTER_HPP_
#define RTDBINPUTADAPTER_HPP_

#include "FalconsRtDB2.hpp"

#include "Motors.pb.h"

struct KickerSetpoint
{
    enum Type
    {
        Home, SetHeight, Shoot
    };
    Type type;
    float value;
};

class RTDBInputAdapter
{
public:
    RTDBInputAdapter();
    ~RTDBInputAdapter();

    ::motors::RobotVector getRobotVelocitySetpoint();
    ::motors::BallhandlerAngles getBallhandlerAngleSetpoints();
    bool getBallhandlersOn();

    void waitForKickerSetpoint();
    KickerSetpoint getKickerSetpoint();

private:
    RtDB2 *rtdb;
};

#endif
