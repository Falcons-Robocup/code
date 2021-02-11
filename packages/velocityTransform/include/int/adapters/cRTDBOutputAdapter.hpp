// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBOUTPUTADAPTER_HPP_
#define CRTDBOUTPUTADAPTER_HPP_

#include "int/cVelocityTransformData.hpp"

#include "FalconsRtDB2.hpp"

class cRTDBOutputAdapter
{
  public:
    cRTDBOutputAdapter();
    ~cRTDBOutputAdapter();

    void setRobotDisplacementFeedback(const vt_robot_data& robotData);
    void setRobotVelocityFeedback(const vt_robot_data& robotData);
    void setMotorVelocitySetpoint(const vt_motors_data& motorsData);

  private:
    int _myRobotId;
    RtDB2 *_rtdb;

};

#endif

