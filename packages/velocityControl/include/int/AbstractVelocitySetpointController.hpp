// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractVelocitySetpointController.hpp
 *
 *  Created on: October 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROL_ABSTRACTVELOCITYSETPOINTCONTROLLER_HPP_
#define VELOCITYCONTROL_ABSTRACTVELOCITYSETPOINTCONTROLLER_HPP_

#include "VelocityControlData.hpp"


// abstract base class for specific controllers, like PID, TokyoDrift, Linear, ...

class AbstractVelocitySetpointController
{
public:
    AbstractVelocitySetpointController() {};
    virtual ~AbstractVelocitySetpointController() {};

    // return velocity setpoint by argument, and success state by return-value
    // velocity setpoint does not have to be clipped, this will be done by limiter algorithm
    virtual bool calculate(VelocityControlData &data, Velocity2D &resultVelocity) = 0;

};

#endif

