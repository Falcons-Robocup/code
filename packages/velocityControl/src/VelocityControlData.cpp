// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocityControlData.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlData.hpp"
#include "tracing.hpp"
#include "generated_enum2str.hpp"


void VelocityControlData::reset()
{
    TRACE_FUNCTION("");
    // clear calculation results and internal data; inputs are overridden at start of iteration
    // make sure all diagnostics related items are cleared here, otherwise you might end up diagnosing data from an older iteration
    deltaPositionFcs = Position2D(0.0, 0.0, 0.0);
    deltaPositionRcs = Position2D(0.0, 0.0, 0.0);
    resultVelocityRcs = Velocity2D(0.0, 0.0, 0.0);
    accelerationRcs = pose();
    for (int dof = 0; dof < 3; ++dof)
    {
        isAccelerating[dof] = false;
        accelerationClipping[dof] = false;
        deadzone[dof] = false;
    }
    done = false;
    motionType = motionTypeEnum::INVALID;
}

void VelocityControlData::traceInputs()
{
    TRACE_FUNCTION("");
    TRACE("robotPos=[%6.2f, %6.2f, %6.2f] robotVel=[%6.2f, %6.2f, %6.2f]", robot.position.x, robot.position.y, robot.position.Rz, robot.velocity.x, robot.velocity.y, robot.velocity.Rz);
    TRACE("targetPos=[%6.2f, %6.2f, %6.2f] motionType=%s", target.pos.x, target.pos.y, target.pos.Rz, enum2str(motionType));
    TRACE("previousVelocityRcs=%s", previousVelocityRcs.tostr());
}

void VelocityControlData::traceOutputs()
{
    TRACE_FUNCTION("");
    TRACE("ROBOT_VELOCITY_SETPOINT=[%6.2f, %6.2f, %6.2f]", resultVelocityRcs.x, resultVelocityRcs.y, resultVelocityRcs.phi);
}

void VelocityControlData::configureLimits()
{
    auto it = vcConfig.motionTypes.find( enum2str(motionType) );
    if (it == vcConfig.motionTypes.end())
    {
        std::stringstream str;
        str << "motionType '" << enum2str(motionType) << "' (" << (int)motionType << ") not found in VelocityControl configuration";
        throw std::runtime_error(str.str());
    }
    currentMotionTypeConfig = vcConfig.motionTypes.at( enum2str(motionType) );

    TRACE("limits have been configured to: %s", tostr(currentMotionTypeConfig).c_str());
}

