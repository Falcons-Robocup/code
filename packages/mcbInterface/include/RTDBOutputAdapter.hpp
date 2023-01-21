// Copyright 2019-2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef RTDBOUTPUTADAPTER_HPP_
#define RTDBOUTPUTADAPTER_HPP_

#include "FalconsRTDB.hpp"

#include "motion.pb.h"
#include "motors.pb.h"

class RTDBOutputAdapter
{
public:
    RTDBOutputAdapter();
    ~RTDBOutputAdapter();

    void setRobotVelocity(const ::Motion::RobotVector& velocity);
    void setRobotPosition(const ::Motion::RobotVector& position);
    void setBallHandlerAngles(const ::Motion::BallhandlerVector& angles);
    void setInPlayStatus(bool in_play);

    // diag only
    void setMotorDiagnosticsOutput( const ::Motors::MotorVector& torque,
                                    const ::Motors::MotorVector& velocity,
                                    const ::Motors::MotorVector& velocity_demand,
                                    const ::Motors::MotorVector& position,
                                    const ::Motors::MotorVector& current,
                                    const ::Motors::MotorVector& current_demand);
    void setMotorFeedback(const ::Motors::MotorVector &velocity);

private:
    RtDB2 *rtdb;
};

#endif
