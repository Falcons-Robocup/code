// Copyright 2019-2021 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef RTDBINPUTADAPTER_HPP_
#define RTDBINPUTADAPTER_HPP_

#include "FalconsRTDB.hpp"

#include "motion.pb.h"

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

    ::Motion::RobotVector getRobotVelocitySetpoint();
    ::Motion::BallhandlerVector getBallhandlerAngleSetpoints();
    bool getBallhandlersOn();

    void waitForKickerSetpoint();
    KickerSetpoint getKickerSetpoint();

private:
    RtDB2 *rtdb;
};

#endif
