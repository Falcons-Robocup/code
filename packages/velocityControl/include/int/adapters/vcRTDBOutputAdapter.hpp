// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcRTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBOUTPUTADAPTER_VELOCITYCONTROL_HPP_
#define RTDBOUTPUTADAPTER_VELOCITYCONTROL_HPP_

#include "int/OutputInterface.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "FalconsRTDB.hpp"

class vcRTDBOutputAdapter : public OutputInterface
{

public:
    vcRTDBOutputAdapter(bool verbose = false, int robotId = getRobotNumber());
    ~vcRTDBOutputAdapter();

    void setVelocity(robotVelocity const &robotVelocitySetpoint);
    void setDiagnostics(diagVelocityControl const &diagnostics);

private:
    RtDB2 *_rtdb;
    int _myRobotId;
    bool _verbose = false;

};

#endif

