// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocitySetpointControllers.hpp
 *
 *  Created on: October, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROL_VELOCITYSETPOINTCONTROLLERS_HPP_
#define VELOCITYCONTROL_VELOCITYSETPOINTCONTROLLERS_HPP_

// other packages
#include "falconsCommon.hpp" // Velocity2D, TODO #14
#include "tracing.hpp"

// own package
#include "AbstractVelocitySetpointController.hpp"
#include "facilities/PIDController.hpp" // TODO move to a common package?


// controller definitions, each implementation has its own file

class StopVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    bool calculate(VelocityControlData &data, Velocity2D &resultVelocity);
};

// not useful for robot, but works OK for simulation
class LinearVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    bool calculate(VelocityControlData &data, Velocity2D &resultVelocity);
};

// used on robot up to and including 2019
class PIDVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    PIDVelocitySetpointController(MotionPIDConfig const &config) { _config = config; }
    bool calculate(VelocityControlData &data, Velocity2D &resultVelocity);
private:
    MotionPIDConfig _config;
    // controllers need to remain state in between ticks, for integral term
    PIDController _controllerX;
    PIDController _controllerY;
    PIDController _controllerRz;
};

// SetPointGenerator, using Reflexxes motion control library
#include "SPGVelocitySetpointController.hpp"

#endif

