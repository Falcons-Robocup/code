 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
    }
}

bool SPGVelocitySetpointController::calculate(PathPlanningVelocityControlData &data, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");
    TRACE("limits = %s", tostr(data.currentLimits).c_str());

    // Type II Reflexxes Motion Library
    // note that this variant does not support jerk control
    // (TODO: buy 1 license and experiment with it?)

    // in order to achieve higher deceleration, a layer is added around the SPG algorithm
    // we start with (aggressive) deceleration limits, so the robot is normally going to be in time for braking
    // and we get a good estimate of velocity and acceleration setpoint
    // then, if robot _seems_ to be accelerating, corresponding limits are used in a recalculation
    SpgLimits spgLimits;
    spgLimits.vx = data.currentLimits.maxVelX;
    spgLimits.vy = data.currentLimits.maxVelYforward;
    spgLimits.vRz = data.currentLimits.maxVelRz;
    spgLimits.ax = data.currentLimits.maxDecX;
    spgLimits.ay = data.currentLimits.maxDecY;
    spgLimits.aRz = data.currentLimits.maxDecRz;
    bool result = calculateUsingSyncFlag(data, resultVelocity, spgLimits);
    bool recalculate = false;
    if (isDofAccelerating(0, data.currentLimits.accThresholdX))
    {
        TRACE("accelerating in X direction, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.ax = data.currentLimits.maxAccX;
    }
    if (isDofAccelerating(1, data.currentLimits.accThresholdY))
    {
        TRACE("accelerating in Y direction, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.ay = (resultVelocity.y < 0.0) ? data.currentLimits.maxAccYbackward : data.currentLimits.maxAccYforward;
    }
    if (isDofAccelerating(2, data.currentLimits.accThresholdRz))
    {
        TRACE("accelerating in Rz, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.aRz = data.currentLimits.maxAccRz;
    }
    if (resultVelocity.y < 0.0)
    {
        TRACE("driving backwards, will recalculate with tighter limit");
        recalculate = true;
        spgLimits.vy = data.currentLimits.maxVelYbackward;
    }

    // recalculate?
    if (recalculate)
    {
        TRACE("recalculating");
        result = calculateUsingSyncFlag(data, resultVelocity, spgLimits);
    }

    // cleanup
    if (_reflexxesOutput != NULL)
    {
        delete _reflexxesOutput;
        _reflexxesOutput = NULL;
    }
    return result;
}

bool SPGVelocitySetpointController::isDofAccelerating(int dof, float threshold)
{
    if (_reflexxesOutput != NULL)
    {
        return _reflexxesOutput->NewVelocityVector->VecData[dof] * _reflexxesOutput->NewAccelerationVector->VecData[dof] >= threshold;
    }
    return true;
}

bool SPGVelocitySetpointController::calculateUsingSyncFlag(PathPlanningVelocityControlData &data, Velocity2D &resultVelocity, SpgLimits const &spgLimits)
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

bool SPGVelocitySetpointController::calculateUsingLimits(PathPlanningVelocityControlData &data, Velocity2D &resultVelocity, SpgLimits const &spgLimits, bool synchronize)
{
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
    int NUMBER_OF_DOFS = 3;
    RML = new ReflexxesAPI(NUMBER_OF_DOFS, data.dt); // degrees of freedom (x,y,Rz), nominal cycle time
    IP  = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    if (_reflexxesOutput != NULL)
    {
        delete _reflexxesOutput;
        _reflexxesOutput = NULL;
    }
    _reflexxesOutput = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    // set-up the input parameters
    IP->CurrentPositionVector->VecData      [0] = 0.0; // instead of steering from current to target,
    IP->CurrentPositionVector->VecData      [1] = 0.0; // we steer from zero to delta, so we can better configure
    IP->CurrentPositionVector->VecData      [2] = 0.0; // controlling FCS or RCS
    TRACE("CurrentPositionVector=(%8.4f, %8.4f, %8.4f)", IP->CurrentPositionVector->VecData[0], IP->CurrentPositionVector->VecData[1], IP->CurrentPositionVector->VecData[2]);

    float w = _config.weightFactorClosedLoop;
    Velocity2D currentVelocity = data.currentVelocity * w + data.previousVelocitySetpoint * (1.0-w);
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

    IP->MaxAccelerationVector->VecData      [0] = spgLimits.ax;
    IP->MaxAccelerationVector->VecData      [1] = spgLimits.ay;
    IP->MaxAccelerationVector->VecData      [2] = spgLimits.aRz;

    IP->MaxJerkVector->VecData              [0] = 0.0; // not used in TypeII library
    IP->MaxJerkVector->VecData              [1] = 0.0;
    IP->MaxJerkVector->VecData              [2] = 0.0;

    IP->TargetPositionVector->VecData       [0] = data.deltaPosition.x;
    IP->TargetPositionVector->VecData       [1] = data.deltaPosition.y;
    IP->TargetPositionVector->VecData       [2] = data.deltaPosition.phi;
    TRACE("TargetPositionVector=(%8.4f, %8.4f, %8.4f)", IP->TargetPositionVector->VecData[0], IP->TargetPositionVector->VecData[1], IP->TargetPositionVector->VecData[2]);

    IP->TargetVelocityVector->VecData       [0] = 0.0; // TODO
    IP->TargetVelocityVector->VecData       [1] = 0.0;
    IP->TargetVelocityVector->VecData       [2] = 0.0;
    TRACE("TargetVelocityVector=(%8.4f, %8.4f, %8.4f)", IP->TargetVelocityVector->VecData[0], IP->TargetVelocityVector->VecData[1], IP->TargetVelocityVector->VecData[2]);

    IP->SelectionVector->VecData            [0] = true;
    IP->SelectionVector->VecData            [1] = true;
    IP->SelectionVector->VecData            [2] = true;

    // call the Reflexxes Online Trajectory Generation algorithm
    int r1 = RML->RMLPosition(*IP, _reflexxesOutput, Flags);
    TRACE("trajectory calculated, r=%d", r1);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", _reflexxesOutput->NewVelocityVector->VecData[0], _reflexxesOutput->NewVelocityVector->VecData[1], _reflexxesOutput->NewVelocityVector->VecData[2]);
    if (r1 < 0)
    {
        // error state
        return false;
    }

    // output parameters have been evaluated at first tick (data.dt)
    // latency correction: evaluate the trajectory at some offset
    double timeOffset = data.dt + _config.latencyOffset;
    int r2 = RML->RMLPositionAtAGivenSampleTime(timeOffset, _reflexxesOutput);
    TRACE("trajectory evaluated with time offset %.3fs, r=%d", timeOffset, r2);
    TRACE("NewVelocityVector=(%8.4f, %8.4f, %8.4f)", _reflexxesOutput->NewVelocityVector->VecData[0], _reflexxesOutput->NewVelocityVector->VecData[1], _reflexxesOutput->NewVelocityVector->VecData[2]);

    // convert outputs
    resultVelocity.x = _reflexxesOutput->NewVelocityVector->VecData[0];
    resultVelocity.y = _reflexxesOutput->NewVelocityVector->VecData[1];
    resultVelocity.phi = _reflexxesOutput->NewVelocityVector->VecData[2];

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

    return true;
}

