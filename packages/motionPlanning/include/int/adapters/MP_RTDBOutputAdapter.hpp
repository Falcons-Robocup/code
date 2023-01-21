// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef MP_RTDBOUTPUTADAPTER_HPP_
#define MP_RTDBOUTPUTADAPTER_HPP_

#include <vector>

#include "FalconsRTDB.hpp"
#include "falconsCommon.hpp" //Position2D


class MP_RTDBOutputAdapter
{
public:
    MP_RTDBOutputAdapter();
    ~MP_RTDBOutputAdapter();

    void setMotionSetpoint(actionTypeEnum action, Position2D pos, motionTypeEnum motionType);
    void setShootSetpoint(shootPhaseEnum shootPhase, shootTypeEnum shootType, Position2D pos);
    void setBallHandlersSetpoint(bool enabled);
    void setActionResult(T_ACTION_RESULT mpActionResult);
    void setKeeperFrameSetpoint(T_KEEPERFRAME_SETPOINT s);
    void setKickerPower(float kickerPower);
    void setKickerHeight(float kickerHeight);
    void setKickerSetpoint(T_KICKER_SETPOINT const &kickerSetpoint);
    void disableObstacleAvoidance();

private:
    RtDB2 *_rtdb;
    int _myRobotId;

};

#endif

