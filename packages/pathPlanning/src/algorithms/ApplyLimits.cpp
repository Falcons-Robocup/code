 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ApplyLimits.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"
#include "int/facilities/clipping.hpp"


void ApplyLimits::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // skip if so configured
    // NOTE: it can be dangerous to disable this limiter, because some controllers do not inherently
    // apply limits -- linear and PID velocity controllers for instance, so be careful when using / tuning those
    if (data.currentLimits.enabled == false)
    {
        TRACE("limiter after velocity controller is disabled");
        return;
    }

    // runtime limit override
    if (data.overrideRzLimits)
    {
        data.currentLimits.maxVelRz = data.overrideRzMaxVel;
        data.currentLimits.maxAccRz = data.overrideRzMaxAcc;
    }

    // trace limits
    TRACE("using limits: %s", tostr(data.currentLimits).c_str());

    // get inputs
    Velocity2D unclippedVelocitySetpoint = data.resultVelocityRcs;
    Velocity2D previousVelocitySetpoint = data.previousVelocityRcs;
    float dt = data.dt;
    float maxSpeedX = data.currentLimits.maxVelX;
    bool drivingBackward = unclippedVelocitySetpoint.y < 0.0;
    float maxSpeedY = (drivingBackward ? data.currentLimits.maxVelYbackward : data.currentLimits.maxVelYforward);
    float maxSpeedRz = data.currentLimits.maxVelRz;
    Velocity2D currentVelocity = Velocity2D(data.robot.velocity.x, data.robot.velocity.y, data.robot.velocity.Rz);
    currentVelocity.transform_fcs2rcs(Position2D(data.robot.position.x, data.robot.position.y, data.robot.position.Rz));

    // initialize result
    Velocity2D newVelocitySetpoint = unclippedVelocitySetpoint;

    // XY velocity and acceleration
    if (data.isAccelerating[0])
    {
        if (gclip(newVelocitySetpoint.x, previousVelocitySetpoint.x, data.currentLimits.maxAccX * dt, "accX"))
        {
            data.accelerationClipping[0] = true;
        }
    }
    else
    {
        if (gclip(newVelocitySetpoint.x, previousVelocitySetpoint.x, data.currentLimits.maxDecX * dt, "decX"))
        {
            data.accelerationClipping[0] = true;
        }
    }
    (void)gclip(newVelocitySetpoint.x, 0.0, maxSpeedX, "velX");
    if (data.isAccelerating[1])
    {
        float maxAccY = data.currentLimits.maxAccYforward;
        if (drivingBackward)
        {
            maxAccY = data.currentLimits.maxAccYbackward;
        }
        if (gclip(newVelocitySetpoint.y, previousVelocitySetpoint.y, maxAccY * dt, "accY"))
        {
            data.accelerationClipping[1] = true;
        }
    }
    else
    {
        if (gclip(newVelocitySetpoint.y, previousVelocitySetpoint.y, data.currentLimits.maxDecY * dt, "decY"))
        {
            data.accelerationClipping[1] = true;
        }
    }
    (void)gclip(newVelocitySetpoint.y, 0.0, maxSpeedY, "velY");

    // Rz velocity and acceleration
    if (data.isAccelerating[2])
    {
        if (gclip(newVelocitySetpoint.phi, previousVelocitySetpoint.phi, data.currentLimits.maxAccRz * dt, "accRz"))
        {
            data.accelerationClipping[2] = true;
        }
    }
    else
    {
        if (gclip(newVelocitySetpoint.phi, previousVelocitySetpoint.phi, data.currentLimits.maxDecRz * dt, "decRz"))
        {
            data.accelerationClipping[2] = true;
        }
    }
    (void)gclip(newVelocitySetpoint.phi, 0.0, maxSpeedRz, "velRz");

    // store
    // NOTE: in case of ball possession, this velocity- and acceleration result applies to ball, not robot!
    // TODO: this might be getting a bit confusing, better to split? for example, kstplot will now show inconsistent diagnostics ...
    data.resultVelocityRcs = newVelocitySetpoint;
    auto a = (data.resultVelocityRcs - data.previousVelocityRcs) / data.dt; // TODO #14
    data.accelerationRcs = pose(a.x, a.y, a.phi);
    data.previousVelocityRcs = data.resultVelocityRcs;
    data.previousTimestamp = data.timestamp;
}

