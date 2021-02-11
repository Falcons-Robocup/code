// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.hpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#ifndef CRTDBINPUTADAPTER_HPP_
#define CRTDBINPUTADAPTER_HPP_

#include "int/cVelocityTransformData.hpp"

#include "FalconsRtDB2.hpp"

class cRTDBInputAdapter
{
    public:
        cRTDBInputAdapter() { };
        cRTDBInputAdapter(cVelocityTransformData *data, iterateFunctionType feedbackfunc, iterateFunctionType setpointfunc);
        ~cRTDBInputAdapter();

        void waitForMotorFeedback();
        void waitForRobotVelocitySetpoint();
        void getRobotVelocitySetpoint();
        void getMotorFeedback();

    private:
        int _myRobotId;
        RtDB2 *_rtdb;
        cVelocityTransformData* _vtData;
        iterateFunctionType _iterateFeedbackFunc;
        iterateFunctionType _iterateSetpointFunc;

};

#endif

