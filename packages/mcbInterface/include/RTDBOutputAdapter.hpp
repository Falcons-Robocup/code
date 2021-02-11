// Copyright 2019 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef RTDBOUTPUTADAPTER_HPP_
#define RTDBOUTPUTADAPTER_HPP_

#include "FalconsRtDB2.hpp"

#include "Motors.pb.h"

class RTDBOutputAdapter
{
public:
    RTDBOutputAdapter();
    ~RTDBOutputAdapter();

    void setRobotVelocity(const ::motors::RobotVector& velocity);
    void setRobotPosition(const ::motors::RobotVector& position);
    void setBallHandlerAngles(const ::motors::BallhandlerAngles& angles);
    void setInPlayStatus(bool in_play);

private:
    RtDB2 *rtdb;
};

#endif
