// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.hpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#ifndef CRTDBINPUTADAPTER_HPP_
#define CRTDBINPUTADAPTER_HPP_

#include "int/ballHandlingControl.hpp"

#include "FalconsRtDB2.hpp"

class cRTDBInputAdapter
{
public:
    cRTDBInputAdapter() { };
    cRTDBInputAdapter(ballHandlingControl* bhControl);
    ~cRTDBInputAdapter();

    void waitForBallHandlersSetpoint();
    void waitForBallHandlersFeedback();

    void getBallHandlersSetpoint();
    void getBallHandlersFeedback();
    void getRobotVelocityFeedback();

private:
    int _myRobotId;
    RtDB2 *_rtdb;
    ballHandlingControl *_bhControl;

};

#endif

