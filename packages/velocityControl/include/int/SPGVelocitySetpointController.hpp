// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SPGVelocitySetpointController.hpp
 *
 *  Created on: Oct, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROL_SPGVELOCITYSETPOINTCONTROLLER_HPP_
#define VELOCITYCONTROL_SPGVELOCITYSETPOINTCONTROLLER_HPP_

// other packages
#include "falconsCommon.hpp" // Velocity2D, TODO #14

// own package
#include "VelocitySetpointControllers.hpp"

// external
#include <ReflexxesAPI.h>


struct SpgLimits
{
    float vx;
    float vy;
    float vRz;
    float ax;
    float ay;
    float aRz;
};

class SPGVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    SPGVelocitySetpointController(SpgConfig const &config);
    ~SPGVelocitySetpointController();
    bool calculate(VelocityControlData &data, Velocity2D &resultVelocity);

private:
    bool calculateUsingSyncFlag(VelocityControlData &data, Velocity2D &resultVelocity, SpgLimits const &spgLimits);
    bool calculateUsingLimits(VelocityControlData &data, Velocity2D &resultVelocity, SpgLimits const &spgLimits, bool synchronize);
    bool isDofAccelerating(VelocityControlData &data, int dof, float threshold);
    SpgConfig _config;
    int NUMBER_OF_DOFS = 3;
    RMLOutputParameters* _reflexxesOutput = NULL;

    // SPG with Position setpoint
    RMLPositionOutputParameters* _reflexxesPosOutput = NULL;
    bool calculatePosUsingLimits(VelocityControlData& data, const Velocity2D& currentVelocity, const SpgLimits& spgLimits, const Position2D& deltaPosition, const pose& targetVelocity, const bool synchronize, Velocity2D &resultVelocity);

    // SPG with Velocity setpoint
    RMLVelocityOutputParameters* _reflexxesVelOutput = NULL;
    bool calculateVelUsingLimits(VelocityControlData& data, const Velocity2D& currentVelocity, const SpgLimits& spgLimits, const pose& targetVelocity, const bool synchronize, Velocity2D &resultVelocity);

};

#endif

