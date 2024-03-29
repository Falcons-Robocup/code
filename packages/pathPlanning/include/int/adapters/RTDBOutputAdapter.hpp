// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBOUTPUTADAPTER_PATHPLANNING_HPP_
#define RTDBOUTPUTADAPTER_PATHPLANNING_HPP_

#include "int/OutputInterface.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "FalconsRTDB.hpp"

class RTDBOutputAdapter : public OutputInterface
{

public:
    RTDBOutputAdapter(bool verbose = false, int robotId = getRobotNumber());
    ~RTDBOutputAdapter();

    void setSubtarget(actionResultTypeEnum const &status, robotPosVel const &robotPosVelSetpoint);
    void setDiagnostics(diagPathPlanning const &diagnostics);

private:
    FalconsRTDB *_rtdb;
    int _myRobotId;
    bool _verbose = false;

};

#endif

