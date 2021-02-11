// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotVelocityInterface.hpp
 *
 * Get robot velocity setpoint in RCS from velocityControl, so it can be used in feedforward.
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#ifndef ROBOTVELOCITYINTERFACE_HPP_
#define ROBOTVELOCITYINTERFACE_HPP_

#include "falconsCommon.hpp"

class robotVelocityInterface
{
  public:
    robotVelocityInterface() {};
    virtual ~robotVelocityInterface() {};

    virtual Velocity2D getRobotVelocity() = 0;
};

#endif /* ROBOTVELOCITYINTERFACE_HPP_ */

