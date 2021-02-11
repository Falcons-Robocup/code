// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * OutputInterface.hpp
 *
 *  Created on: Oct 2020
 *      Author: Erik Kouters
 */

#ifndef OUTPUTINTERFACE_VELOCITYCONTROL_HPP_
#define OUTPUTINTERFACE_VELOCITYCONTROL_HPP_

// sharedTypes
#include "actionResult.hpp"
#include "robotVelocity.hpp"
#include "diagVelocityControl.hpp"


class OutputInterface
{
public:
    OutputInterface() {};
    virtual ~OutputInterface() {};

    // required
    virtual void setVelocity(robotVelocity const &robotVelocitySetpoint) = 0;

    // optional
    virtual void setDiagnostics(diagVelocityControl const &diagnostics) {};
};

#endif

