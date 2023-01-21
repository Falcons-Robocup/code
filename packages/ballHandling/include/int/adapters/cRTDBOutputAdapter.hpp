// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBOUTPUTADAPTER_HPP_
#define CRTDBOUTPUTADAPTER_HPP_

#include "int/types/ballHandlersSetpointsType.hpp"

#include "FalconsRTDB.hpp"

class cRTDBOutputAdapter
{
public:
    cRTDBOutputAdapter();
    ~cRTDBOutputAdapter();

    void setBallHandlersMotorSetpoint(const bool& enabled, const ballHandlersSetpointsType& setpoints);
    void setBallHandlersBallPossession(const bool& ballPossession);
    void setDiagnostics(DiagBallHandling const &diag);

private:
    int _myRobotId;
    RtDB2 *_rtdb;

};

#endif

