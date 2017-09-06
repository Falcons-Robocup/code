 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionGetBallOnVector.cpp
 *
 *  Created on: July 2nd, 2016
 *      Author: Erik Kouters
 */

#include "int/actions/cActionGetBallOnVector.hpp"
#include "int/stores/ballStore.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;

cActionGetBallOnVector::cActionGetBallOnVector()
{
	intention.actionType = actionEnum::GET_BALL_ON_VECTOR;
}

cActionGetBallOnVector::~cActionGetBallOnVector()
{

}

// Only run this function if the robot's velocity is larger than threshold
double velocityThreshold = 0.05;

// Add extra offset to ball projection to maintain speed when almost grabbing ball.
double extraOffset = 0.15;

// Move to ballPos (x,y).
// Turn to look towards 'facing' (x,y).
// If no 'facing' was given, turn to look towards the ball.
// If no ball was found, do not turn (keep current facing).
// Returns: RUNNING   if the robot is not at the target position
//          PASSED    if the robot reached the target position
//          FAILED    if something went wrong
behTreeReturnEnum cActionGetBallOnVector::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos;
        cWorldModelInterface::getInstance().getOwnLocation(myPos);

        Velocity2D myVel;
        cWorldModelInterface::getInstance().getOwnVelocity(myVel);

        ball ball = ballStore::getBall();

        ballPossession_struct_t ballPossession;
        cWorldModelInterface::getInstance().getBallPossession(ballPossession);
        if (ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER)
        {
            // Got ball. Do nothing and return PASSED
            return behTreeReturnEnum::PASSED;
        }

        if (ball.isLocationKnown() && ball.isVelocityKnown())
        {
            geometry::Pose2D currentPose(myPos.x, myPos.y, myPos.phi);

            // Compute ball velocity in RCS
            geometry::Velocity2D ballVelocity(ball.getVelocity().x, ball.getVelocity().y, 0);
            geometry::Velocity2D ballVelocityRCS = ballVelocity.transformFCS2RCS(currentPose);

            // Compute robot velocity in RCS
            geometry::Velocity2D robotVelocity(myVel.x, myVel.y, 0);
            geometry::Velocity2D robotVelocityRCS = robotVelocity.transformFCS2RCS(currentPose);

            // Convert to Vector2D
            Vector2D ballPosition(ball.getPosition().x, ball.getPosition().y);
            Vector2D ballSpeed(ball.getVelocity().x, ball.getVelocity().y);

            // ballProjection is where we will tell the robot to move to
            Vector2D ballProjection;

            // With trial&error testing on the robot, a non-moving ball approached by a robot has about a 0.35m/s velocity vector.
            // If the vector is below 0.35m/s, ignore the vector (noise) and simply put the robot on top of the ball.
            // TODO - Remove magic number -> make configurable?
            if (ballSpeed.size() > 0.35)
            {
                // Vector is larger than 0.35m/s, place target on ball vector.
                // TODO - Remove magic number of the normalized ball velocity vector
                Vector2D normBallSpeed = ballSpeed.normalized(0.6);
                ballProjection = ballPosition + normBallSpeed;
            }
            else
            {
                // Vector is smaller than 0.35m/s, place target on ball to ignore noise on the ball vector.
                ballProjection = ballPosition;
            }

            // Always look towards the ball
            double angle = angle_between_two_points_0_2pi(myPos.x, myPos.y, ball.getPosition().x, ball.getPosition().y);
            Position2D targetPos = Position2D(ballProjection.x, ballProjection.y, angle);

            intention.x = targetPos.x;
            intention.y = targetPos.y;
            sendIntention();

            // ensure that we move according to the rules
            if (!isCurrentPosValid())
            {
                // Move towards the center, while maintaining angle
                moveTo(0.0, 0.0, targetPos.phi);
                return behTreeReturnEnum::RUNNING;
            }

            if (!isTargetPosInsideSafetyBoundaries(targetPos))
            {
                return behTreeReturnEnum::FAILED;
            }

            moveTo(targetPos.x, targetPos.y, targetPos.phi);
            return behTreeReturnEnum::RUNNING;
        }
        else
        {
            // We see no balls. Failed.
            return behTreeReturnEnum::FAILED;
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}

