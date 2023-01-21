// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocityControlAlgorithms.hpp
 *
 *  Created on: Oct, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROL_ALGORITHMS_HPP_
#define VELOCITYCONTROL_ALGORITHMS_HPP_


#include "int/VelocityControlData.hpp"
#include "int/AbstractVelocitySetpointController.hpp"
#include "tracing.hpp"

/*!
 * \brief is the abstract class that each VelocityControl algorithm should inherit.
 *
 */
class VelocityControlAlgorithm
{
public:
    VelocityControlAlgorithm() {}

    virtual void execute(VelocityControlData &data) = 0;
};


// algorithm definitions, each implementation has its own file

class CalculateDeltas : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class CalculateAccelerating : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class SelectVelocityController : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class CalculateVelocity : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);

    // callback to get VelocitySetpointController
    // link from algorithm to VelocityControl class
    boost::function<AbstractVelocitySetpointController *(void)> _callback;

public:
    CalculateVelocity(boost::function<AbstractVelocitySetpointController *(void)> callback);
};

class Deadzone : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class ApplyLimits : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class ShiftBallOffset : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class UnShiftBallOffset : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

class ApplyTokyoDrift : public VelocityControlAlgorithm
{
    void execute(VelocityControlData &data);
};

#endif
