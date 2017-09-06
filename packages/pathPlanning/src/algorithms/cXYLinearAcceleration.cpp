 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPIDPathPlanning.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/algorithms/cXYLinearAcceleration.hpp"

double cXYLinearAcceleration::getBrakeDistance(Velocity2D currVel)
{
    TRACE(">");
    // Get limits (reconfig) from cPathPlanningData
    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);

    //   v = target velocity
    //   u = current velocity
    //   a = acceleration (negative: deceleration)
    //   d = distance
    // solving for d:
    //   d = (v*v - u*u) / 2a
    TRACE(">");
    return -(currVel.size()*currVel.size()) / (-2*limits.maxAccXY);
}

/* Overwrite functionality of cAbstractPathPlanning update function */
void cXYLinearAcceleration::execute()
{
    TRACE(">");
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    // Get current velocity from cPathPlanningData
    Velocity2D currVel;
    _main->_ppData->getVelocity(currVel);

    // Get limits (reconfig) from cPathPlanningData
    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);

    // If new target received
    if ((_ppData.pos - _currTarget).xy().size() > 0.1)
    {
        _isBraking = false;
    }

    // Use s_set_own_encoder_displacement to obtain robotSpeed.

    /* Calculate error */
    Vector2D distToTarget = (currPos).xy();

    // By default send out maximum acceleration.
    // This will be limited to 0 if the robot is already at maximum velocity.
    Vector2D targetAcc = distToTarget.normalized() * limits.maxAccXY;

    // Compute the distance from the target where to start decelerating
    double A = getBrakeDistance(currVel);

    // If we should already start braking, perform maximum deceleration
    if (fabs(distToTarget.size() < A))
    {
        targetAcc *= -1;
        _isBraking = true;
    }
    else if (_isBraking)
    {
        // If we are braking, set acceleration to 0.
        targetAcc *= 0;
    }

    // Compute newVel from targetAcc
    Vector2D newVelVect = currVel.xy() + (targetAcc * _dt);

    /* If the current velocity is too great, limit the new velocity and publish */
    Velocity2D newVel = Velocity2D(newVelVect.x, newVelVect.y, 0.0);

    TRACE("distToTarget x=%12.9f, y=%12.9f", distToTarget.x, distToTarget.y);
    TRACE("brakeDist=%12.9f", A);
    TRACE("targetAcc x=%12.9f, y=%12.9f", targetAcc.x, targetAcc.y);
    TRACE("currVel x=%12.9f, y=%12.9f", currVel.x, currVel.y);
    TRACE("newVel x=%12.9f, y=%12.9f", newVel.x, newVel.y);

    _currTarget = _ppData.pos;

    _ppData.vel.x = newVel.x;
    _ppData.vel.y = newVel.y;
    TRACE("<");
}

