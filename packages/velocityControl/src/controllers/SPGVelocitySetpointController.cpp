// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SPGVelocitySetpointController.cpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */

#include "int/SPGVelocitySetpointController.hpp"
#include "tracing.hpp"



SPGVelocitySetpointController::SPGVelocitySetpointController(SpgConfig const &config)
{
    _config = config;
}

SPGVelocitySetpointController::~SPGVelocitySetpointController()
{
    if (_reflexxesOutput != NULL)
    {
        delete _reflexxesOutput;
        _reflexxesOutput = NULL;
    }
}

bool SPGVelocitySetpointController::calculate(VelocityControlData &data, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");
    TRACE("limits = %s", tostr(data.currentMotionTypeConfig.limits).c_str());

    // Type II Reflexxes Motion Library
    // note that this variant does not support jerk control
    // (TODO: buy 1 license and experiment with it?)

    // in order to achieve higher deceleration, a layer is added around the SPG algorithm
    // we start with (aggressive) deceleration limits, so the robot is normally going to be in time for braking
    // and we get a good estimate of velocity and acceleration setpoint
    // then, if robot _seems_ to be accelerating, corresponding limits are used in a recalculation
    SpgLimits spgLimits;
    spgLimits.vx = data.currentMotionTypeConfig.limits.maxVelX;
    spgLimits.vy = data.currentMotionTypeConfig.limits.maxVelYforward;
    spgLimits.vRz = data.currentMotionTypeConfig.limits.maxVelRz;
    spgLimits.ax = data.currentMotionTypeConfig.limits.maxDecX;
    spgLimits.ay = data.currentMotionTypeConfig.limits.maxDecY;
    spgLimits.aRz = data.currentMotionTypeConfig.limits.maxDecRz;
    bool result = calculateUsingSyncFlag(data, resultVelocity, spgLimits);
    bool recalculate = false;
    if (isDofAccelerating(data, 0, data.currentMotionTypeConfig.limits.accThresholdX))
    {
        TRACE("accelerating in X direction, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.ax = data.currentMotionTypeConfig.limits.maxAccX;
    }
    if (isDofAccelerating(data, 1, data.currentMotionTypeConfig.limits.accThresholdY))
    {
        TRACE("accelerating in Y direction, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.ay = (resultVelocity.y < 0.0) ? data.currentMotionTypeConfig.limits.maxAccYbackward : data.currentMotionTypeConfig.limits.maxAccYforward;
    }
    if (isDofAccelerating(data, 2, data.currentMotionTypeConfig.limits.accThresholdRz))
    {
        TRACE("accelerating in Rz, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.aRz = data.currentMotionTypeConfig.limits.maxAccRz;
    }
    if (resultVelocity.y < 0.0)
    {
        TRACE("driving backwards, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.vy = data.currentMotionTypeConfig.limits.maxVelYbackward;
    }

    // recalculate?
    if (recalculate)
    {
        TRACE("recalculating");
        result = calculateUsingSyncFlag(data, resultVelocity, spgLimits);
    }

    //cleanup
    if (_reflexxesOutput != NULL)
    {
        delete _reflexxesOutput;
        _reflexxesOutput = NULL;
    }

    return result;
}

bool SPGVelocitySetpointController::isDofAccelerating(VelocityControlData &data, int dof, float threshold)
{
    // To check if a DOF is accelerating, we should look if the currentVel -> newVel is "moving away from zero".

    Velocity2D currVelocity;
    Velocity2D previousVelocitySetpoint;
    if (data.vcSetpointConfig.coordinateSystem == CoordinateSystemEnum::RCS)
    {
        currVelocity = data.currentVelocityFcs.transform_fcs2rcs(data.currentPositionFcs);
        previousVelocitySetpoint = data.previousVelocityRcs;
    }
    else
    {
        currVelocity = data.currentVelocityFcs;
        previousVelocitySetpoint = data.previousVelocityRcs.transform_rcs2fcs(data.currentPositionFcs);
    }

    Velocity2D currentVelocity = currVelocity * _config.weightFactorClosedLoop + previousVelocitySetpoint * (1.0-_config.weightFactorClosedLoop);

    std::vector<double> currVel;
    currVel.push_back(currentVelocity.x);
    currVel.push_back(currentVelocity.y);
    currVel.push_back(currentVelocity.phi);


    if (_reflexxesOutput != NULL)
    {
        if (currVel.at(dof) < 0.0)
        {
            // newVel - currVel < threshold
            // e.g., -2.0 - -1.0 = -1.0 < (-threshold)
            TRACE("%8.4f - %8.4f < (-%8.4f)", _reflexxesOutput->NewVelocityVector->VecData[dof], currVel.at(dof), threshold);
            return (_reflexxesOutput->NewVelocityVector->VecData[dof] - currVel.at(dof)) < (-threshold);
        }
        else if (currVel.at(dof) > 0.0)
        {
            // newVel - currVel > threshold
            // e.g., 2.0 - 1.0 = 1.0 > threshold
            TRACE("%8.4f - %8.4f > %8.4f", _reflexxesOutput->NewVelocityVector->VecData[dof], currVel.at(dof), threshold);
            return (_reflexxesOutput->NewVelocityVector->VecData[dof] - currVel.at(dof)) > threshold;
        }
        else
        {
            // currVel == 0.0
            TRACE("abs(%8.4f) > %8.4f", _reflexxesOutput->NewVelocityVector->VecData[dof], threshold);
            return abs(_reflexxesOutput->NewVelocityVector->VecData[dof]) > threshold;
        }
    }

    return true;
}

bool SPGVelocitySetpointController::calculateUsingSyncFlag(VelocityControlData &data, Velocity2D &resultVelocity, SpgLimits const &spgLimits)
{
    bool result = false;
    if (_config.synchronizeRotation)
    {
        // SPG will sync x, y and Rz nicely
        result = calculateUsingLimits(data, resultVelocity, spgLimits, true);
    }
    else
    {
        // SPG will act 'greedy' for each of x, y and Rz, but we want to synchronize x and y
        // so first calculate Rz greedy
        Velocity2D resultVelocityForRz;
        result = calculateUsingLimits(data, resultVelocityForRz, spgLimits, false);
        if (result)
        {
            // now calculate x,y in sync
            result = calculateUsingLimits(data, resultVelocity, spgLimits, true);
            // and overrule Rz
            resultVelocity.phi = resultVelocityForRz.phi;
        }
    }
    return result;
}

bool SPGVelocitySetpointController::calculateUsingLimits(VelocityControlData &data, Velocity2D &resultVelocity, SpgLimits const &spgLimits, bool synchronize)
{
    TRACE_FUNCTION("");
    Position2D deltaPosition;
    Velocity2D currVelocity;
    Velocity2D previousVelocitySetpoint;
    if (data.vcSetpointConfig.coordinateSystem == CoordinateSystemEnum::RCS)
    {
        deltaPosition = data.deltaPositionRcs;
        currVelocity = data.currentVelocityFcs.transform_fcs2rcs(data.currentPositionFcs);
        previousVelocitySetpoint = data.previousVelocityRcs;
        TRACE("controlling velocity in RCS");
    }
    else
    {
        deltaPosition = data.deltaPositionFcs;
        currVelocity = data.currentVelocityFcs;
        previousVelocitySetpoint = data.previousVelocityRcs.transform_rcs2fcs(data.currentPositionFcs);
        TRACE("controlling velocity in FCS");
    }

    float w = _config.weightFactorClosedLoop;
    Velocity2D currentVelocity = currVelocity * w + previousVelocitySetpoint * (1.0-w);

    if (data.robotPosVelMoveType == robotPosVelEnum::POS_ONLY || data.robotPosVelMoveType == robotPosVelEnum::POSVEL)
    {
        return calculatePosUsingLimits(data, currentVelocity, spgLimits, deltaPosition, data.target.vel, synchronize, resultVelocity);
    }
    else if (data.robotPosVelMoveType == robotPosVelEnum::VEL_ONLY)
    {
        return calculateVelUsingLimits(data, currentVelocity, spgLimits, data.target.vel, synchronize, resultVelocity);
    }
    return false;
}

bool SPGVelocitySetpointController::calculatePosUsingLimits(VelocityControlData& data, const Velocity2D& currentVelocity, const SpgLimits& spgLimits, const Position2D& deltaPosition, const pose& targetVelocity, const bool synchronize, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // initialize
    ReflexxesAPI                *RML = NULL;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionFlags            Flags;
    Flags.SynchronizationBehavior                  = RMLPositionFlags::NO_SYNCHRONIZATION;
    if (synchronize)
    {
        Flags.SynchronizationBehavior              = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    }
    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
    // note: convergence behavior in perfect simulation is not as expected with these flags, see workaround at the bottom

    // construct Reflexxes objects
    RML = new ReflexxesAPI(NUMBER_OF_DOFS, data.dt); // degrees of freedom (x,y,Rz), nominal cycle time
    IP  = new RMLPositionInputParameters(NUMBER_OF_DOFS);

    _reflexxesPosOutput = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // instead of steering from current to target,
    IP->CurrentPositionVector->VecData      [1] = 0.0; // we steer from zero to delta, so we can better configure
    IP->CurrentPositionVector->VecData      [2] = 0.0; // controlling FCS or RCS
    TRACE("CurrentPositionVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentPositionVector->VecData[0], IP->CurrentPositionVector->VecData[1], IP->CurrentPositionVector->VecData[2]);

    IP->CurrentVelocityVector->VecData      [0] = currentVelocity.x;
    IP->CurrentVelocityVector->VecData      [1] = currentVelocity.y;
    IP->CurrentVelocityVector->VecData      [2] = currentVelocity.phi;
    TRACE("CurrentVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentVelocityVector->VecData[0], IP->CurrentVelocityVector->VecData[1], IP->CurrentVelocityVector->VecData[2]);

    IP->CurrentAccelerationVector->VecData  [0] = 0.0; // not relevant, due to limitation of TypeII library
    IP->CurrentAccelerationVector->VecData  [1] = 0.0;
    IP->CurrentAccelerationVector->VecData  [2] = 0.0;
    TRACE("CurrentAccelerationVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentAccelerationVector->VecData[0], IP->CurrentAccelerationVector->VecData[1], IP->CurrentAccelerationVector->VecData[2]);

    IP->MaxVelocityVector->VecData          [0] = spgLimits.vx;
    IP->MaxVelocityVector->VecData          [1] = spgLimits.vy;
    IP->MaxVelocityVector->VecData          [2] = spgLimits.vRz;
    TRACE("MaxVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->MaxVelocityVector->VecData[0], IP->MaxVelocityVector->VecData[1], IP->MaxVelocityVector->VecData[2]);

    IP->MaxAccelerationVector->VecData      [0] = spgLimits.ax;
    IP->MaxAccelerationVector->VecData      [1] = spgLimits.ay;
    IP->MaxAccelerationVector->VecData      [2] = spgLimits.aRz;
    TRACE("MaxAccelerationVector=(%8.4f, %8.4f, %8.4f)", IP->MaxAccelerationVector->VecData[0], IP->MaxAccelerationVector->VecData[1], IP->MaxAccelerationVector->VecData[2]);

    IP->MaxJerkVector->VecData              [0] = 0.0; // not used in TypeII library
    IP->MaxJerkVector->VecData              [1] = 0.0;
    IP->MaxJerkVector->VecData              [2] = 0.0;

    IP->TargetPositionVector->VecData       [0] = deltaPosition.x;
    IP->TargetPositionVector->VecData       [1] = deltaPosition.y;
    IP->TargetPositionVector->VecData       [2] = deltaPosition.phi;
    TRACE("TargetPositionVector=(%8.4f, %8.4f, %8.4f)", IP->TargetPositionVector->VecData[0], IP->TargetPositionVector->VecData[1], IP->TargetPositionVector->VecData[2]);

    IP->TargetVelocityVector->VecData       [0] = targetVelocity.x;
    IP->TargetVelocityVector->VecData       [1] = targetVelocity.y;
    IP->TargetVelocityVector->VecData       [2] = targetVelocity.Rz;
    TRACE("TargetVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->TargetVelocityVector->VecData[0], IP->TargetVelocityVector->VecData[1], IP->TargetVelocityVector->VecData[2]);

    IP->SelectionVector->VecData            [0] = true;
    IP->SelectionVector->VecData            [1] = true;
    IP->SelectionVector->VecData            [2] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLPosition(*IP, _reflexxesPosOutput, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", _reflexxesPosOutput->NewVelocityVector->VecData[0], _reflexxesPosOutput->NewVelocityVector->VecData[1], _reflexxesPosOutput->NewVelocityVector->VecData[2]);
    TRACE("NewAccelerationVector=(%8.4f, %8.4f, %8.4f)", _reflexxesPosOutput->NewAccelerationVector->VecData[0], _reflexxesPosOutput->NewAccelerationVector->VecData[1], _reflexxesPosOutput->NewAccelerationVector->VecData[2]);
    if (r1 < 0)
    {
        // error state
        return false;
    }

    // output parameters have been evaluated at first tick (data.dt)
    // latency correction: evaluate the trajectory at some offset
    double timeOffset = data.dt + _config.latencyOffset;
    int r2 = RML->RMLPositionAtAGivenSampleTime(timeOffset, _reflexxesPosOutput);
    TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", _reflexxesPosOutput->NewVelocityVector->VecData[0], _reflexxesPosOutput->NewVelocityVector->VecData[1], _reflexxesPosOutput->NewVelocityVector->VecData[2]);

    // convert outputs
    resultVelocity.x = _reflexxesPosOutput->NewVelocityVector->VecData[0];
    resultVelocity.y = _reflexxesPosOutput->NewVelocityVector->VecData[1];
    resultVelocity.phi = _reflexxesPosOutput->NewVelocityVector->VecData[2];

    // workaround for non-convergence (mainly applies to simulation):
    // it can happen that Reflexxes outputs v==0 when Rz exceeds tolerance
    // (just disable this workaround and watch some test cases fail)
    // no flags or options seem to be available to fix this behavior ...
    // as workaround, final step is calculated by "perfect" linear controller
    if (_config.convergenceWorkaround)
    {
        // check values, no tolerances
        bool newVelocityIsZero = (resultVelocity.x == 0.0) && (resultVelocity.y == 0.0) && (resultVelocity.phi == 0.0);
        bool distanceIsNonzero = (IP->TargetPositionVector->VecData[0] == 0.0) && (IP->TargetPositionVector->VecData[1] == 0.0) && (IP->TargetPositionVector->VecData[2] == 0.0);
        TRACE("checking convergence workaround: newVelocityIsZero=%d distanceIsNonzero=%d", newVelocityIsZero, distanceIsNonzero);
        if ((r1 == ReflexxesAPI::RML_FINAL_STATE_REACHED) || (newVelocityIsZero && distanceIsNonzero))
        {
            TRACE("applying short-stroke convergence workaround using LinearVelocitySetpointController");
            return LinearVelocitySetpointController().calculate(data, resultVelocity);
        }
    }

    // cleanup
    delete RML;
    delete IP;

    _reflexxesOutput = dynamic_cast<RMLOutputParameters*>(_reflexxesPosOutput);

    return true;
}

bool SPGVelocitySetpointController::calculateVelUsingLimits(VelocityControlData& data, const Velocity2D& currentVelocity, const SpgLimits& spgLimits, const pose& targetVelocity, const bool synchronize, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // initialize
    ReflexxesAPI                *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityFlags            Flags;
    Flags.SynchronizationBehavior                  = RMLVelocityFlags::NO_SYNCHRONIZATION;
    if (synchronize)
    {
        Flags.SynchronizationBehavior              = RMLVelocityFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    }
    // note: convergence behavior in perfect simulation is not as expected with these flags, see workaround at the bottom

    // construct Reflexxes objects
    RML = new ReflexxesAPI(NUMBER_OF_DOFS, data.dt); // degrees of freedom (x,y,Rz), nominal cycle time
    IP  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    _reflexxesVelOutput = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // instead of steering from current to target,
    IP->CurrentPositionVector->VecData      [1] = 0.0; // we steer from zero to delta, so we can better configure
    IP->CurrentPositionVector->VecData      [2] = 0.0; // controlling FCS or RCS
    TRACE("CurrentPositionVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentPositionVector->VecData[0], IP->CurrentPositionVector->VecData[1], IP->CurrentPositionVector->VecData[2]);

    IP->CurrentVelocityVector->VecData      [0] = currentVelocity.x;
    IP->CurrentVelocityVector->VecData      [1] = currentVelocity.y;
    IP->CurrentVelocityVector->VecData      [2] = currentVelocity.phi;
    TRACE("CurrentVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentVelocityVector->VecData[0], IP->CurrentVelocityVector->VecData[1], IP->CurrentVelocityVector->VecData[2]);

    IP->CurrentAccelerationVector->VecData  [0] = 0.0; // not relevant, due to limitation of TypeII library
    IP->CurrentAccelerationVector->VecData  [1] = 0.0;
    IP->CurrentAccelerationVector->VecData  [2] = 0.0;
    TRACE("CurrentAccelerationVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentAccelerationVector->VecData[0], IP->CurrentAccelerationVector->VecData[1], IP->CurrentAccelerationVector->VecData[2]);

    IP->MaxAccelerationVector->VecData      [0] = spgLimits.ax;
    IP->MaxAccelerationVector->VecData      [1] = spgLimits.ay;
    IP->MaxAccelerationVector->VecData      [2] = spgLimits.aRz;
    TRACE("MaxAccelerationVector=(%8.4f, %8.4f, %8.4f)", IP->MaxAccelerationVector->VecData[0], IP->MaxAccelerationVector->VecData[1], IP->MaxAccelerationVector->VecData[2]);

    IP->MaxJerkVector->VecData              [0] = 0.0; // not used in TypeII library
    IP->MaxJerkVector->VecData              [1] = 0.0;
    IP->MaxJerkVector->VecData              [2] = 0.0;

    IP->TargetVelocityVector->VecData       [0] = targetVelocity.x;
    IP->TargetVelocityVector->VecData       [1] = targetVelocity.y;
    IP->TargetVelocityVector->VecData       [2] = targetVelocity.Rz;
    TRACE("TargetVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->TargetVelocityVector->VecData[0], IP->TargetVelocityVector->VecData[1], IP->TargetVelocityVector->VecData[2]);

    IP->SelectionVector->VecData            [0] = true;
    IP->SelectionVector->VecData            [1] = true;
    IP->SelectionVector->VecData            [2] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLVelocity(*IP, _reflexxesVelOutput, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", _reflexxesVelOutput->NewVelocityVector->VecData[0], _reflexxesVelOutput->NewVelocityVector->VecData[1], _reflexxesVelOutput->NewVelocityVector->VecData[2]);
    TRACE("NewAccelerationVector=(%8.4f, %8.4f, %8.4f)", _reflexxesVelOutput->NewAccelerationVector->VecData[0], _reflexxesVelOutput->NewAccelerationVector->VecData[1], _reflexxesVelOutput->NewAccelerationVector->VecData[2]);
    if (r1 < 0)
    {
        // error state
        return false;
    }

    // output parameters have been evaluated at first tick (data.dt)
    // latency correction: evaluate the trajectory at some offset
    double timeOffset = data.dt + _config.latencyOffset;
    int r2 = RML->RMLVelocityAtAGivenSampleTime(timeOffset, _reflexxesVelOutput);
    TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", _reflexxesVelOutput->NewVelocityVector->VecData[0], _reflexxesVelOutput->NewVelocityVector->VecData[1], _reflexxesVelOutput->NewVelocityVector->VecData[2]);

    // convert outputs
    if (std::isnan(_reflexxesVelOutput->NewVelocityVector->VecData[0])
         || std::isnan(_reflexxesVelOutput->NewVelocityVector->VecData[1])
         || std::isnan(_reflexxesVelOutput->NewVelocityVector->VecData[2]))
    {
        TRACE("SPG output is nan. Setting to 0.0.");
        _reflexxesVelOutput->NewVelocityVector->VecData[0] = 0.0;
        _reflexxesVelOutput->NewVelocityVector->VecData[1] = 0.0;
        _reflexxesVelOutput->NewVelocityVector->VecData[2] = 0.0;
    }

    resultVelocity.x = _reflexxesVelOutput->NewVelocityVector->VecData[0];
    resultVelocity.y = _reflexxesVelOutput->NewVelocityVector->VecData[1];
    resultVelocity.phi = _reflexxesVelOutput->NewVelocityVector->VecData[2];
    

    // cleanup
    delete RML;
    delete IP;

    _reflexxesOutput = dynamic_cast<RMLOutputParameters*>(_reflexxesVelOutput);

    return true;
}
