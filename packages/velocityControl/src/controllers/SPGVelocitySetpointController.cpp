// Copyright 2020-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SPGVelocitySetpointController.cpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */

#include "int/SPGVelocitySetpointController.hpp"
#include "tracing.hpp"

#include <algorithm>

SPGVelocitySetpointController::SPGVelocitySetpointController(SpgConfig const &config)
{
    _config = config;
}

SPGVelocitySetpointController::~SPGVelocitySetpointController()
{
}

bool SPGVelocitySetpointController::calculate(VelocityControlData &data, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");
    TRACE("limits = %s", tostr(data.currentMotionTypeConfig.limits).c_str());

    // Type II Reflexxes Motion Library
    // note that this variant does not support jerk control

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

    // For position, finding a weighted average on the Rz is not trivial when the angles are around the boundary of [0, 2pi].
    // To solve this, rotate both currentPosFCS.Rz and previousPosSetpointFCS.Rz towards currentPosFCS.Rz.
    // This means currentPosFCS.Rz = 0, and previousPosSetpointFCS.Rz is the difference to currentPosFCS.Rz.
    // At this point, take the weighted factor from previousPosSetpointFCS.Rz -> deltaAngle projected between [-pi, pi]
    // This weighted factor can be rotated back from currentPosFCS.Rz.
    // The final average angle between the two is then currentPosFCS.Rz + deltaAngle 

    float w_pos = data.currentMotionTypeConfig.setPointGenerator.weightFactorClosedLoopPos;
    double deltaRz = project_angle_mpi_pi(data.previousPositionSetpointFcs.phi - data.currentPositionFcs.phi) * w_pos;
    double weightedRz = data.currentPositionFcs.phi + deltaRz;

    Position2D weightedCurrentPositionFCS = data.currentPositionFcs * w_pos + data.previousPositionSetpointFcs * (1.0-w_pos);
    weightedCurrentPositionFCS.phi = weightedRz;
    float w_vel = data.currentMotionTypeConfig.setPointGenerator.weightFactorClosedLoopVel;
    Velocity2D weightedCurrentVelocityFCS = data.currentVelocityFcs * w_vel + data.previousVelocitySetpointFcs * (1.0-w_vel);

    TRACE("currentPosFCS=%s -- previousPosSetpointFCS=%s", data.currentPositionFcs.tostr(), data.previousPositionSetpointFcs.tostr());
    TRACE("weightedCurrentPosFCS=%s", weightedCurrentPositionFCS.tostr());

    _deltaPositionRCS = Position2D(data.targetPositionFcs).transform_fcs2rcs(weightedCurrentPositionFCS);
    _deltaPositionRCS.phi = project_angle_mpi_pi(data.targetPositionFcs.phi - weightedCurrentPositionFCS.phi); // EKPC: this works, but I have no idea why
    TRACE("deltaPositionRCS=%s", _deltaPositionRCS.tostr());
    _currentVelocityRCS = Velocity2D(weightedCurrentVelocityFCS).transform_fcs2rcs(weightedCurrentPositionFCS);
    _targetVelocityRCS = Velocity2D(data.targetVelocityFcs).transform_fcs2rcs(weightedCurrentPositionFCS);

    Position2D resultPosition;
    bool result = calculateSPG(data, spgLimits, resultPosition, resultVelocity);


    bool recalculate = false;
    if (isDofAccelerating(data, resultVelocity, 0, data.currentMotionTypeConfig.limits.accThresholdX))
    {
        TRACE("accelerating in X direction, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.ax = data.currentMotionTypeConfig.limits.maxAccX;
    }
    if (isDofAccelerating(data, resultVelocity, 1, data.currentMotionTypeConfig.limits.accThresholdY))
    {
        TRACE("accelerating in Y direction, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.ay = (resultVelocity.y < 0.0) ? data.currentMotionTypeConfig.limits.maxAccYbackward : data.currentMotionTypeConfig.limits.maxAccYforward;
    }
    if (isDofAccelerating(data, resultVelocity, 2, data.currentMotionTypeConfig.limits.accThresholdRz))
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

    TRACE("resultVelocity = %s", resultVelocity.tostr());

    // recalculate?
    if (recalculate)
    {
        TRACE("recalculating");
        result = calculateSPG(data, spgLimits, resultPosition, resultVelocity);
    }

    TRACE("resultVelocity = %s", resultVelocity.tostr());

    // check if motor limits are not exceeded.
    for(int i = 0; i < 10; i++)
    {
        if (data.vtClient.exceedsMotorLimits( pose(resultVelocity.x, resultVelocity.y, resultVelocity.phi) ))
        {
            // reduce limits by 20% to see if we can find setpoints which do not exceed the motor limits
            spgLimits.vx *= 0.8;
            spgLimits.vy *= 0.8;
            spgLimits.vRz *= 0.8;
            TRACE("recalculating -- motor limits exceeded");
            result = calculateSPG(data, spgLimits, resultPosition, resultVelocity);
        }
        else
        {
            break;
        }
        TRACE("resultVelocity = %s", resultVelocity.tostr());
    }

    TRACE("SPG done: resultVelocity = %s", resultVelocity.tostr());

    // Done -- store values for next iteration

    // Store previousPositionSetpointFcs for open loop control
    Position2D tmpPos = resultPosition;
    data.previousPositionSetpointFcs = tmpPos.transform_rcs2fcs(weightedCurrentPositionFCS);
    data.previousPositionSetpointFcs.phi += M_PI_2;
    data.previousPositionSetpointFcs.phi = project_angle_0_2pi(data.previousPositionSetpointFcs.phi);

    // Store previousVelocitySetpointFcs for open loop control
    Velocity2D tmpVel = resultVelocity;
    data.previousVelocitySetpointFcs = tmpVel.transform_rcs2fcs(weightedCurrentPositionFCS);

    // Used by DeadZone calculation (see other algorithms)
    data.deltaPositionRcs = _deltaPositionRCS;

    return result;
}

bool SPGVelocitySetpointController::isDofAccelerating(const VelocityControlData &data, const Velocity2D& resultVelocity, int dof, float threshold)
{
    // To check if a DOF is accelerating, we should look if the _currentVelocityRCS -> resultVelocity is "moving away from zero".

    std::vector<double> currentVelocity;
    currentVelocity.push_back(_currentVelocityRCS.x);
    currentVelocity.push_back(_currentVelocityRCS.y);
    currentVelocity.push_back(_currentVelocityRCS.phi);

    std::vector<double> newVelocity;
    newVelocity.push_back(resultVelocity.x);
    newVelocity.push_back(resultVelocity.y);
    newVelocity.push_back(resultVelocity.phi);


    if (currentVelocity.at(dof) < 0.0)
    {
        // newVelocity - currentVelocity < threshold
        // e.g., -2.0 - -1.0 = -1.0 < (-threshold)
        TRACE("%8.4f - %8.4f < (-%8.4f)", newVelocity.at(dof), currentVelocity.at(dof), threshold);
        return (newVelocity.at(dof) - currentVelocity.at(dof)) < (-threshold);
    }
    else if (currentVelocity.at(dof) > 0.0)
    {
        // newVelocity - currentVelocity > threshold
        // e.g., 2.0 - 1.0 = 1.0 > threshold
        TRACE("%8.4f - %8.4f > %8.4f", newVelocity.at(dof), currentVelocity.at(dof), threshold);
        return (newVelocity.at(dof) - currentVelocity.at(dof)) > threshold;
    }
    else
    {
        // currentVelocity == 0.0
        TRACE("abs(%8.4f) > %8.4f", newVelocity.at(dof), threshold);
        return abs(newVelocity.at(dof)) > threshold;
    }

    return true;
}




bool SPGVelocitySetpointController::calculateSPG(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity)
{
    bool result = false;
    if (data.robotPosVelMoveType != robotPosVelEnum::VEL_ONLY)
    {
        // POS_ONLY or POSVEL
        if (_config.synchronizeRotation)
        {
            result = calculatePosXYRzPhaseSynchronized(data, spgLimits, resultPosition, resultVelocity);
        }
        else
        {
            result = calculatePosXYPhaseSynchronized(data, spgLimits, resultPosition, resultVelocity);
            result = calculatePosRzNonSynchronized(data, spgLimits, resultPosition, resultVelocity);
        }
    }
    else
    {
        // Reflexxes has a problem when velocity already reached
        double EPSILON = 1E-4;
        if (   abs(_currentVelocityRCS.x   - std::clamp(_targetVelocityRCS.x,   (double) -spgLimits.vx,  (double) spgLimits.vx))  < EPSILON
            && abs(_currentVelocityRCS.y   - std::clamp(_targetVelocityRCS.y,   (double) -spgLimits.vy,  (double) spgLimits.vy))  < EPSILON
            && abs(_currentVelocityRCS.phi - std::clamp(_targetVelocityRCS.phi, (double) -spgLimits.vRz, (double) spgLimits.vRz)) < EPSILON)
        {
            resultVelocity = Velocity2D(_currentVelocityRCS);
            return true;
        }
        else
        {
            result = calculateVelXYRzPhaseSynchronized(data, spgLimits, resultPosition, resultVelocity);
        }
    }
    return result;
}

bool SPGVelocitySetpointController::calculatePosXYRzPhaseSynchronized(VelocityControlData& data, const SpgLimits& spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // initialize
    ReflexxesAPI                *RML = NULL;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags            Flags;
    Flags.SynchronizationBehavior                  = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;

    // construct Reflexxes objects
    int numberOfDOFs = 3; //X, Y, Rz
    RML = new ReflexxesAPI(numberOfDOFs, data.dt); // degrees of freedom (x,y,Rz), nominal cycle time
    IP  = new RMLPositionInputParameters(numberOfDOFs);
    OP  = new RMLPositionOutputParameters(numberOfDOFs);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // instead of steering from current to target,
    IP->CurrentPositionVector->VecData      [1] = 0.0; // we steer from zero to delta, so we can better configure
    IP->CurrentPositionVector->VecData      [2] = 0.0; // controlling FCS or RCS
    TRACE("CurrentPositionVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentPositionVector->VecData[0], IP->CurrentPositionVector->VecData[1], IP->CurrentPositionVector->VecData[2]);

    IP->CurrentVelocityVector->VecData      [0] = _currentVelocityRCS.x;
    IP->CurrentVelocityVector->VecData      [1] = _currentVelocityRCS.y;
    IP->CurrentVelocityVector->VecData      [2] = _currentVelocityRCS.phi;
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

    IP->TargetPositionVector->VecData       [0] = _deltaPositionRCS.x;  // steering from currentPos = 0,
    IP->TargetPositionVector->VecData       [1] = _deltaPositionRCS.y;  // to targetPos = deltaPos
    IP->TargetPositionVector->VecData       [2] = _deltaPositionRCS.phi;
    TRACE("TargetPositionVector=(%8.4f, %8.4f, %8.4f)", IP->TargetPositionVector->VecData[0], IP->TargetPositionVector->VecData[1], IP->TargetPositionVector->VecData[2]);

    IP->TargetVelocityVector->VecData       [0] = _targetVelocityRCS.x;
    IP->TargetVelocityVector->VecData       [1] = _targetVelocityRCS.y;
    IP->TargetVelocityVector->VecData       [2] = _targetVelocityRCS.phi;
    TRACE("TargetVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->TargetVelocityVector->VecData[0], IP->TargetVelocityVector->VecData[1], IP->TargetVelocityVector->VecData[2]);

    IP->SelectionVector->VecData            [0] = true;
    IP->SelectionVector->VecData            [1] = true;
    IP->SelectionVector->VecData            [2] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLPosition(*IP, OP, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", OP->NewVelocityVector->VecData[0], OP->NewVelocityVector->VecData[1], OP->NewVelocityVector->VecData[2]);
    TRACE("NewAccelerationVector=(%8.4f, %8.4f, %8.4f)", OP->NewAccelerationVector->VecData[0], OP->NewAccelerationVector->VecData[1], OP->NewAccelerationVector->VecData[2]);
    if (r1 < 0)
    {
        // error state
        TRACE_ERROR("ERROR during SPG computation");
        return false;
    }

    // output parameters have been evaluated at first tick (data.dt)
    // latency correction: evaluate the trajectory at some offset
    double timeOffset = data.dt + _config.latencyOffset;
    int r2 = RML->RMLPositionAtAGivenSampleTime(timeOffset, OP);
    TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", OP->NewVelocityVector->VecData[0], OP->NewVelocityVector->VecData[1], OP->NewVelocityVector->VecData[2]);

    // convert outputs
    resultPosition.x   = OP->NewPositionVector->VecData[0];
    resultPosition.y   = OP->NewPositionVector->VecData[1];
    resultPosition.phi = OP->NewPositionVector->VecData[2];

    resultVelocity.x   = OP->NewVelocityVector->VecData[0];
    resultVelocity.y   = OP->NewVelocityVector->VecData[1];
    resultVelocity.phi = OP->NewVelocityVector->VecData[2];

    // Diag data
    data.spgCurrentPosition.x  = IP->CurrentPositionVector->VecData[0];
    data.spgCurrentPosition.y  = IP->CurrentPositionVector->VecData[1];
    data.spgCurrentPosition.Rz = IP->CurrentPositionVector->VecData[2];

    data.spgCurrentVelocity.x  = IP->CurrentVelocityVector->VecData[0];
    data.spgCurrentVelocity.y  = IP->CurrentVelocityVector->VecData[1];
    data.spgCurrentVelocity.Rz = IP->CurrentVelocityVector->VecData[2];

    data.spgMaxVelocity.x  = IP->MaxVelocityVector->VecData[0];
    data.spgMaxVelocity.y  = IP->MaxVelocityVector->VecData[1];
    data.spgMaxVelocity.Rz = IP->MaxVelocityVector->VecData[2];

    data.spgMaxAcceleration.x  = IP->MaxAccelerationVector->VecData[0];
    data.spgMaxAcceleration.y  = IP->MaxAccelerationVector->VecData[1];
    data.spgMaxAcceleration.Rz = IP->MaxAccelerationVector->VecData[2];

    data.spgTargetPosition.x  = IP->TargetPositionVector->VecData[0];
    data.spgTargetPosition.y  = IP->TargetPositionVector->VecData[1];
    data.spgTargetPosition.Rz = IP->TargetPositionVector->VecData[2];

    data.spgTargetVelocity.x  = IP->TargetVelocityVector->VecData[0];
    data.spgTargetVelocity.y  = IP->TargetVelocityVector->VecData[1];
    data.spgTargetVelocity.Rz = IP->TargetVelocityVector->VecData[2];

    data.spgNewPosition.x  = OP->NewPositionVector->VecData[0];
    data.spgNewPosition.y  = OP->NewPositionVector->VecData[1];
    data.spgNewPosition.Rz = OP->NewPositionVector->VecData[2];

    data.spgNewVelocity.x  = OP->NewVelocityVector->VecData[0];
    data.spgNewVelocity.y  = OP->NewVelocityVector->VecData[1];
    data.spgNewVelocity.Rz = OP->NewVelocityVector->VecData[2];

    // cleanup
    delete RML;
    delete IP;
    delete OP;

    // workaround for non-convergence (mainly applies to simulation):
    // it can happen that Reflexxes outputs v==0 when Rz exceeds tolerance
    // (just disable this workaround and watch some test cases fail)
    // no flags or options seem to be available to fix this behavior ...
    // as workaround, final step is calculated by "perfect" linear controller
    if (_config.convergenceWorkaround)
    {
        if (r1 == ReflexxesAPI::RML_FINAL_STATE_REACHED)
        {
            TRACE("applying short-stroke convergence workaround using LinearVelocitySetpointController");

            // Used by LinearVelocity calculation
            data.deltaPositionRcs = _deltaPositionRCS;

            Velocity2D tmpResultVelocity;
            bool result = LinearVelocitySetpointController().calculate(data, tmpResultVelocity);
            resultVelocity.x   = tmpResultVelocity.x;
            resultVelocity.y   = tmpResultVelocity.y;
            resultVelocity.phi = tmpResultVelocity.phi;
            return result;
        }
    }

    return true;

}
bool SPGVelocitySetpointController::calculatePosXYPhaseSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // initialize
    ReflexxesAPI                *RML = NULL;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags            Flags;
    Flags.SynchronizationBehavior                  = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;

    // construct Reflexxes objects
    int numberOfDOFs = 2; //X, Y
    RML = new ReflexxesAPI(numberOfDOFs, data.dt); // degrees of freedom (x,y), nominal cycle time
    IP  = new RMLPositionInputParameters(numberOfDOFs);
    OP  = new RMLPositionOutputParameters(numberOfDOFs);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // instead of steering from current to target,
    IP->CurrentPositionVector->VecData      [1] = 0.0; // we steer from zero to delta, so we can better configure
    TRACE("CurrentPositionVector=(%8.4f, %8.4f)", IP->CurrentPositionVector->VecData[0], IP->CurrentPositionVector->VecData[1]);

    IP->CurrentVelocityVector->VecData      [0] = _currentVelocityRCS.x;
    IP->CurrentVelocityVector->VecData      [1] = _currentVelocityRCS.y;
    TRACE("CurrentVelocityVector=(%8.4f, %8.4f)", IP->CurrentVelocityVector->VecData[0], IP->CurrentVelocityVector->VecData[1]);

    IP->CurrentAccelerationVector->VecData  [0] = 0.0; // not relevant, due to limitation of TypeII library
    IP->CurrentAccelerationVector->VecData  [1] = 0.0;
    TRACE("CurrentAccelerationVector=(%8.4f, %8.4f)", IP->CurrentAccelerationVector->VecData[0], IP->CurrentAccelerationVector->VecData[1]);

    IP->MaxVelocityVector->VecData          [0] = spgLimits.vx;
    IP->MaxVelocityVector->VecData          [1] = spgLimits.vy;
    TRACE("MaxVelocityVector=(%8.4f, %8.4f)", IP->MaxVelocityVector->VecData[0], IP->MaxVelocityVector->VecData[1]);

    IP->MaxAccelerationVector->VecData      [0] = spgLimits.ax;
    IP->MaxAccelerationVector->VecData      [1] = spgLimits.ay;
    TRACE("MaxAccelerationVector=(%8.4f, %8.4f)", IP->MaxAccelerationVector->VecData[0], IP->MaxAccelerationVector->VecData[1]);

    IP->MaxJerkVector->VecData              [0] = 0.0; // not used in TypeII library
    IP->MaxJerkVector->VecData              [1] = 0.0;

    IP->TargetPositionVector->VecData       [0] = _deltaPositionRCS.x;  // steering from currentPos = 0,
    IP->TargetPositionVector->VecData       [1] = _deltaPositionRCS.y;  // to targetPos = deltaPos
    TRACE("TargetPositionVector=(%8.4f, %8.4f)", IP->TargetPositionVector->VecData[0], IP->TargetPositionVector->VecData[1]);

    IP->TargetVelocityVector->VecData       [0] = _targetVelocityRCS.x;
    IP->TargetVelocityVector->VecData       [1] = _targetVelocityRCS.y;
    TRACE("TargetVelocityVector=(%8.4f, %8.4f)", IP->TargetVelocityVector->VecData[0], IP->TargetVelocityVector->VecData[1]);

    IP->SelectionVector->VecData            [0] = true;
    IP->SelectionVector->VecData            [1] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLPosition(*IP, OP, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f, %8.4f)", OP->NewVelocityVector->VecData[0], OP->NewVelocityVector->VecData[1]);
    TRACE("NewAccelerationVector=(%8.4f, %8.4f)", OP->NewAccelerationVector->VecData[0], OP->NewAccelerationVector->VecData[1]);
    if (r1 < 0)
    {
        // error state
        TRACE_ERROR("ERROR during SPG computation");
        return false;
    }

    // output parameters have been evaluated at first tick (data.dt)
    // latency correction: evaluate the trajectory at some offset
    double timeOffset = data.dt + _config.latencyOffset;
    int r2 = RML->RMLPositionAtAGivenSampleTime(timeOffset, OP);
    TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", OP->NewVelocityVector->VecData[0], OP->NewVelocityVector->VecData[1]);

    // convert outputs
    resultPosition.x = OP->NewPositionVector->VecData[0];
    resultPosition.y = OP->NewPositionVector->VecData[1];

    resultVelocity.x = OP->NewVelocityVector->VecData[0];
    resultVelocity.y = OP->NewVelocityVector->VecData[1];

    // Diag data
    data.spgCurrentPosition.x  = IP->CurrentPositionVector->VecData[0];
    data.spgCurrentPosition.y  = IP->CurrentPositionVector->VecData[1];

    data.spgCurrentVelocity.x  = IP->CurrentVelocityVector->VecData[0];
    data.spgCurrentVelocity.y  = IP->CurrentVelocityVector->VecData[1];

    data.spgMaxVelocity.x  = IP->MaxVelocityVector->VecData[0];
    data.spgMaxVelocity.y  = IP->MaxVelocityVector->VecData[1];

    data.spgMaxAcceleration.x  = IP->MaxAccelerationVector->VecData[0];
    data.spgMaxAcceleration.y  = IP->MaxAccelerationVector->VecData[1];

    data.spgTargetPosition.x  = IP->TargetPositionVector->VecData[0];
    data.spgTargetPosition.y  = IP->TargetPositionVector->VecData[1];

    data.spgTargetVelocity.x  = IP->TargetVelocityVector->VecData[0];
    data.spgTargetVelocity.y  = IP->TargetVelocityVector->VecData[1];

    data.spgNewPosition.x  = OP->NewPositionVector->VecData[0];
    data.spgNewPosition.y  = OP->NewPositionVector->VecData[1];

    data.spgNewVelocity.x  = OP->NewVelocityVector->VecData[0];
    data.spgNewVelocity.y  = OP->NewVelocityVector->VecData[1];

    // cleanup
    delete RML;
    delete IP;
    delete OP;

    // workaround for non-convergence (mainly applies to simulation):
    // it can happen that Reflexxes outputs v==0 when Rz exceeds tolerance
    // (just disable this workaround and watch some test cases fail)
    // no flags or options seem to be available to fix this behavior ...
    // as workaround, final step is calculated by "perfect" linear controller
    if (_config.convergenceWorkaround)
    {
        if (r1 == ReflexxesAPI::RML_FINAL_STATE_REACHED)
        {
            TRACE("applying short-stroke convergence workaround using LinearVelocitySetpointController");

            // Used by LinearVelocity calculation
            data.deltaPositionRcs = _deltaPositionRCS;

            Velocity2D tmpResultVelocity;
            bool result = LinearVelocitySetpointController().calculate(data, tmpResultVelocity);
            resultVelocity.x   = tmpResultVelocity.x;
            resultVelocity.y   = tmpResultVelocity.y;
            return result;
        }
    }

    return true;
}
bool SPGVelocitySetpointController::calculatePosRzNonSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // initialize
    ReflexxesAPI                *RML = NULL;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags            Flags;
    Flags.SynchronizationBehavior                  = RMLPositionFlags::NO_SYNCHRONIZATION;
    Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;

    // construct Reflexxes objects
    int numberOfDOFs = 1; // Rz
    RML = new ReflexxesAPI(numberOfDOFs, data.dt); // degrees of freedom (Rz), nominal cycle time
    IP  = new RMLPositionInputParameters(numberOfDOFs);
    OP  = new RMLPositionOutputParameters(numberOfDOFs);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // controlling FCS or RCS
    TRACE("CurrentPositionVector=(%8.4f)", IP->CurrentPositionVector->VecData[0]);

    IP->CurrentVelocityVector->VecData      [0] = _currentVelocityRCS.phi;
    TRACE("CurrentVelocityVector=(%8.4f)", IP->CurrentVelocityVector->VecData[0]);

    IP->CurrentAccelerationVector->VecData  [0] = 0.0;
    TRACE("CurrentAccelerationVector=(%8.4f)", IP->CurrentAccelerationVector->VecData[0]);

    IP->MaxVelocityVector->VecData          [0] = spgLimits.vRz;
    TRACE("MaxVelocityVector=(%8.4f)", IP->MaxVelocityVector->VecData[0]);

    IP->MaxAccelerationVector->VecData      [0] = spgLimits.aRz;
    TRACE("MaxAccelerationVector=(%8.4f)", IP->MaxAccelerationVector->VecData[0]);

    IP->MaxJerkVector->VecData              [0] = 0.0;

    IP->TargetPositionVector->VecData       [0] = _deltaPositionRCS.phi;
    TRACE("TargetPositionVector=(%8.4f)", IP->TargetPositionVector->VecData[0]);

    IP->TargetVelocityVector->VecData       [0] = _targetVelocityRCS.phi;
    TRACE("TargetVelocityVector=(%8.4f)", IP->TargetVelocityVector->VecData[0]);

    IP->SelectionVector->VecData            [0] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLPosition(*IP, OP, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f)", OP->NewVelocityVector->VecData[0]);
    TRACE("NewAccelerationVector=(%8.4f)", OP->NewAccelerationVector->VecData[0]);
    if (r1 < 0)
    {
        // error state
        TRACE_ERROR("ERROR during SPG computation");
        return false;
    }

    // output parameters have been evaluated at first tick (data.dt)
    // latency correction: evaluate the trajectory at some offset
    double timeOffset = data.dt + _config.latencyOffset;
    int r2 = RML->RMLPositionAtAGivenSampleTime(timeOffset, OP);
    TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
    TRACE("NewVelocityVector=(%8.4f)", OP->NewVelocityVector->VecData[0]);

    // convert outputs
    resultPosition.phi = OP->NewPositionVector->VecData[0];

    resultVelocity.phi = OP->NewVelocityVector->VecData[0];

    // Diag data
    data.spgCurrentPosition.Rz = IP->CurrentPositionVector->VecData[0];

    data.spgCurrentVelocity.Rz = IP->CurrentVelocityVector->VecData[0];

    data.spgMaxVelocity.Rz = IP->MaxVelocityVector->VecData[0];

    data.spgMaxAcceleration.Rz = IP->MaxAccelerationVector->VecData[0];

    data.spgTargetPosition.Rz = IP->TargetPositionVector->VecData[0];

    data.spgTargetVelocity.Rz = IP->TargetVelocityVector->VecData[0];

    data.spgNewPosition.Rz = OP->NewPositionVector->VecData[0];

    data.spgNewVelocity.Rz = OP->NewVelocityVector->VecData[0];

    // cleanup
    delete RML;
    delete IP;
    delete OP;

    // workaround for non-convergence (mainly applies to simulation):
    // it can happen that Reflexxes outputs v==0 when Rz exceeds tolerance
    // (just disable this workaround and watch some test cases fail)
    // no flags or options seem to be available to fix this behavior ...
    // as workaround, final step is calculated by "perfect" linear controller
    if (_config.convergenceWorkaround)
    {
        if (r1 == ReflexxesAPI::RML_FINAL_STATE_REACHED)
        {
            TRACE("applying short-stroke convergence workaround using LinearVelocitySetpointController");

            // Used by LinearVelocity calculation
            data.deltaPositionRcs = _deltaPositionRCS;

            Velocity2D tmpResultVelocity;
            bool result = LinearVelocitySetpointController().calculate(data, tmpResultVelocity);
            resultVelocity.phi   = tmpResultVelocity.phi;
            return result;
        }
    }

    return true;
}


bool SPGVelocitySetpointController::calculateVelXYRzPhaseSynchronized(VelocityControlData& data, SpgLimits const &spgLimits, Position2D& resultPosition, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // initialize
    ReflexxesAPI                *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags            Flags;
    Flags.SynchronizationBehavior = RMLVelocityFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;

    // construct Reflexxes objects
    int numberOfDOFs = 3; //X, Y, Rz
    RML = new ReflexxesAPI(numberOfDOFs, data.dt); // degrees of freedom (x,y,Rz), nominal cycle time
    IP  = new RMLVelocityInputParameters(numberOfDOFs);
    OP = new RMLVelocityOutputParameters(numberOfDOFs);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // instead of steering from current to target,
    IP->CurrentPositionVector->VecData      [1] = 0.0; // we steer from zero to delta, so we can better configure
    IP->CurrentPositionVector->VecData      [2] = 0.0; // controlling FCS or RCS
    TRACE("CurrentPositionVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentPositionVector->VecData[0], IP->CurrentPositionVector->VecData[1], IP->CurrentPositionVector->VecData[2]);

    IP->CurrentVelocityVector->VecData      [0] = _currentVelocityRCS.x;
    IP->CurrentVelocityVector->VecData      [1] = _currentVelocityRCS.y;
    IP->CurrentVelocityVector->VecData      [2] = _currentVelocityRCS.phi;
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

    IP->TargetVelocityVector->VecData       [0] = std::clamp(_targetVelocityRCS.x,   (double) -spgLimits.vx,  (double) spgLimits.vx);
    IP->TargetVelocityVector->VecData       [1] = std::clamp(_targetVelocityRCS.y,   (double) -spgLimits.vy,  (double) spgLimits.vy);
    IP->TargetVelocityVector->VecData       [2] = std::clamp(_targetVelocityRCS.phi, (double) -spgLimits.vRz, (double) spgLimits.vRz);
    TRACE("TargetVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->TargetVelocityVector->VecData[0], IP->TargetVelocityVector->VecData[1], IP->TargetVelocityVector->VecData[2]);

    IP->SelectionVector->VecData            [0] = true;
    IP->SelectionVector->VecData            [1] = true;
    IP->SelectionVector->VecData            [2] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLVelocity(*IP, OP, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", OP->NewVelocityVector->VecData[0], OP->NewVelocityVector->VecData[1], OP->NewVelocityVector->VecData[2]);
    TRACE("NewAccelerationVector=(%8.4f, %8.4f, %8.4f)", OP->NewAccelerationVector->VecData[0], OP->NewAccelerationVector->VecData[1], OP->NewAccelerationVector->VecData[2]);
    if (r1 < 0)
    {
        // error state
        TRACE_ERROR("ERROR during SPG computation");
        return false;
    }
    else if (r1 == ReflexxesAPI::RMLResultValue::RML_FINAL_STATE_REACHED)
    {
        // final state reached: current velocity == target velocity
        resultVelocity.x   = _targetVelocityRCS.x;
        resultVelocity.y   = _targetVelocityRCS.y;
        resultVelocity.phi = _targetVelocityRCS.phi;
    }
    else
    {
        // output parameters have been evaluated at first tick (data.dt)
        // latency correction: evaluate the trajectory at some offset
        double timeOffset = data.dt + _config.latencyOffset;
        int r2 = RML->RMLVelocityAtAGivenSampleTime(timeOffset, OP);
        TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
        TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", OP->NewVelocityVector->VecData[0], OP->NewVelocityVector->VecData[1], OP->NewVelocityVector->VecData[2]);
        if (r2 < 0)
        {
            // error state
            TRACE_ERROR("ERROR during SPG computation");
            return false;
        }
        else if (r2 == ReflexxesAPI::RMLResultValue::RML_FINAL_STATE_REACHED)
        {
            // final state reached: current velocity == target velocity
            resultVelocity.x   = _targetVelocityRCS.x;
            resultVelocity.y   = _targetVelocityRCS.y;
            resultVelocity.phi = _targetVelocityRCS.phi;
        }
        else
        {
            if (std::isnan(OP->NewVelocityVector->VecData[0])
                || std::isnan(OP->NewVelocityVector->VecData[1])
                || std::isnan(OP->NewVelocityVector->VecData[2]))
            {
                // unknown why results are nan
                TRACE("Reflexxes returned nan");
                return false;
            }

            resultVelocity.x = OP->NewVelocityVector->VecData[0];
            resultVelocity.y = OP->NewVelocityVector->VecData[1];
            resultVelocity.phi = OP->NewVelocityVector->VecData[2];
        }
    }
    


    // Diag data
    data.spgCurrentPosition.x  = IP->CurrentPositionVector->VecData[0];
    data.spgCurrentPosition.y  = IP->CurrentPositionVector->VecData[1];
    data.spgCurrentPosition.Rz = IP->CurrentPositionVector->VecData[2];

    data.spgCurrentVelocity.x  = IP->CurrentVelocityVector->VecData[0];
    data.spgCurrentVelocity.y  = IP->CurrentVelocityVector->VecData[1];
    data.spgCurrentVelocity.Rz = IP->CurrentVelocityVector->VecData[2];

    data.spgMaxAcceleration.x  = IP->MaxAccelerationVector->VecData[0];
    data.spgMaxAcceleration.y  = IP->MaxAccelerationVector->VecData[1];
    data.spgMaxAcceleration.Rz = IP->MaxAccelerationVector->VecData[2];

    data.spgTargetVelocity.x  = IP->TargetVelocityVector->VecData[0];
    data.spgTargetVelocity.y  = IP->TargetVelocityVector->VecData[1];
    data.spgTargetVelocity.Rz = IP->TargetVelocityVector->VecData[2];

    data.spgNewPosition.x  = OP->NewPositionVector->VecData[0];
    data.spgNewPosition.y  = OP->NewPositionVector->VecData[1];
    data.spgNewPosition.Rz = OP->NewPositionVector->VecData[2];

    data.spgNewVelocity.x  = OP->NewVelocityVector->VecData[0];
    data.spgNewVelocity.y  = OP->NewVelocityVector->VecData[1];
    data.spgNewVelocity.Rz = OP->NewVelocityVector->VecData[2];
    

    // cleanup
    delete RML;
    delete IP;
    delete OP;

    return true;
}
