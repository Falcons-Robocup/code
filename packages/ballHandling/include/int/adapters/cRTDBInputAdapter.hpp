// Copyright 2019-2021 Erik Kouters (Falcons)
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

#include "FalconsRTDB.hpp"

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
    void getRobotVelocitySetpoint();

private:
    int _myRobotId;
    RtDB2 *_rtdb;
    ballHandlingControl *_bhControl;

};

#endif

