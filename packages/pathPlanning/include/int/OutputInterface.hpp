// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * OutputInterface.hpp
 *
 *  Created on: July 2019
 *      Author: Jan Feitsma
 */

#ifndef OUTPUTINTERFACE_PATHPLANNING_HPP_
#define OUTPUTINTERFACE_PATHPLANNING_HPP_

// sharedTypes
#include "actionResult.hpp"
#include "robotPosVel.hpp"
#include "diagPathPlanning.hpp"

class OutputInterface
{
public:
    OutputInterface() {};
    virtual ~OutputInterface() {};

    // required
    virtual void setSubtarget(actionResultTypeEnum const &status, robotPosVel const &robotPosVelSetpoint) = 0;

    // optional
    virtual void setDiagnostics(diagPathPlanning const &diagnostics) {};
};

#endif

