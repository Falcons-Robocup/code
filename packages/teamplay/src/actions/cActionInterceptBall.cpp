 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionInterceptBall.cpp
 *
 *  Created on: Jun 11, 2016
 *      Author: Jan Feitsma
 */

#include "int/actions/cActionInterceptBall.hpp"
#include "yaml-cpp/yaml.h"

#include "pose2d.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/utilities/trace.hpp"


using namespace teamplay;

cActionInterceptBall::cActionInterceptBall()
{
    boost::assign::insert( _actionParameters )
        ("captureRadius", std::make_pair(std::vector<std::string>{"float"}, true))
        ;

    intention.actionType = actionEnum::INTERCEPT_BALL;
}

cActionInterceptBall::~cActionInterceptBall()
{

}

// Execute ball intercept action. If the ball is coming towards the robot, then move to intercept.
// Returns: PASSED    if the robot managed to get the ball
//          RUNNING   if the robot does not yet have the ball
//          FAILED    if conditions are not met
// TODO: if ball was within radius, but then moved away, should we report failure?
// NOTE: balls which come from behind are ignored!

// parameters:
// float captureRadius = radius in meters or 'tbd': get value from teamplayInterceptBall.yaml
behTreeReturnEnum cActionInterceptBall::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        auto captureRadius = configurationStore::getConfiguration().getInterceptBallCaptureRadius();
        auto minimumSpeed = configurationStore::getConfiguration().getInterceptBallMinimumSpeed();

        // Override the capture radius if its value is not 'tbd'
        std::string paramStr("captureRadius");
        auto paramValPair = parameters.find(paramStr);
        if (paramValPair != parameters.end())
        {
            std::string paramVal = paramValPair->second;
            if (paramVal != emptyValue)
            {
                captureRadius = boost::lexical_cast<float>(paramVal);
            }
        }
        TRACE("captureRadius = ") << std::to_string(captureRadius);

        // If we have ball, PASSED.
		ballPossession_struct_t ball_possession;
		cWorldModelInterface::getInstance().getBallPossession(ball_possession);
		if ((ball_possession.possessionType == ballPossessionEnum::TEAMMEMBER)
				&& (ball_possession.robotID == cWorldStateFunctions::getInstance().getRobotID()))
		{
            TRACE("PASSED");
            return behTreeReturnEnum::PASSED; // success!
        }

		// The position where the robot intersects with the ball trajectory.
		// So basically the position where to intercept.
		Position2D intersectPos;

		// Get current position
        Position2D currentPos;
        cWorldModelInterface::getInstance().getOwnLocation(currentPos);
        geometry::Pose2D currentPose(currentPos.x, currentPos.y, currentPos.phi); // TODO do something about duplicity in basic geometry types (Position2D versus Pose2D)

		// Get ball data
        ball ball = ballStore::getBall();
        geometry::Pose2D ballPosition(ball.getPosition().x, ball.getPosition().y, 0);
        Vector2D ballPositionVec2D(ball.getPosition().x, ball.getPosition().y);
        geometry::Pose2D ballPositionTransformed = ballPosition;
        geometry::Pose2D ballPositionRCS = ballPositionTransformed.transformFCS2RCS(currentPose);
        geometry::Velocity2D ballVelocity(ball.getVelocity().x, ball.getVelocity().y, 0);
        geometry::Velocity2D ballVelocityTransformed = ballVelocity;
        geometry::Velocity2D ballVelocityRCS = ballVelocityTransformed.transformFCS2RCS(currentPose);
        float ballSpeed = ballVelocity.size();
        Vector2D ballSpeedVec2D(ball.getVelocity().x, ball.getVelocity().y);

        // Compute flags: ballIsMovingTowardsUs, ballMovingFastEnough
        bool ballIsMovingTowardsUs = (ballVelocityRCS.y < 0);
        bool ballMovingFastEnough = (ballSpeed > minimumSpeed);
        
        //// Compute flag: ballIntersectsCaptureRadius
        bool ballIntersectsCaptureRadius = false;
        // span a strafing line using RCS coordinates
        Position2D leftPosRcs(-captureRadius, 0, 0);
        Position2D rightPosRcs(captureRadius, 0, 0);

        // modify currentpos, as if already facing the ball, so we can use transformation
        Position2D transformedCurrentPos = currentPos;
        transformedCurrentPos.phi = angle_between_two_points_0_2pi(transformedCurrentPos.x, transformedCurrentPos.y, ball.getPosition().x, ball.getPosition().y);
        Position2D leftPosFcs = leftPosRcs.transform_rcs2fcs(transformedCurrentPos);
        Position2D rightPosFcs = rightPosRcs.transform_rcs2fcs(transformedCurrentPos);

        // intersect
        Vector2D ballProjection = ballPositionVec2D + (10.0 * ballSpeedVec2D); // make vector long enough
        Vector2D intersectResult;
        if (intersect(ballPosition, ballProjection, leftPosFcs.xy(), rightPosFcs.xy(), intersectResult))
        {
            if ((intersectResult - currentPos.xy()).size() < captureRadius)
            {
                TRACE("intercept: ") << std::to_string(intersectResult.x) << ", " << std::to_string(intersectResult.y);
                intersectPos.phi = transformedCurrentPos.phi;
                intersectPos.x = intersectResult.x;
                intersectPos.y = intersectResult.y;
                ballIntersectsCaptureRadius = true;
            }
        }

        // select?
        TRACE("The ball is moving towards us: ") << std::to_string(ballIsMovingTowardsUs)
         << "  The ball is moving fast enough: " << std::to_string(ballMovingFastEnough)
         << "  The ball intersects capture radius: " << std::to_string(ballIntersectsCaptureRadius);

        // If all conditions are true, intersect ball and return RUNNING
        if (ballIsMovingTowardsUs && ballMovingFastEnough && ballIntersectsCaptureRadius)
        {
        	intention.x = intersectPos.x;
        	intention.y = intersectPos.y;
        	sendIntention();

            // ensure that we move according to the rules
            if (!isCurrentPosValid())
            {
                // Move towards the center, while maintaining angle
                moveTo(0.0, 0.0, intersectPos.phi);
                return behTreeReturnEnum::RUNNING;
            }

            if (!isTargetPosInsideSafetyBoundaries(intersectPos))
            {
                return behTreeReturnEnum::FAILED;
            }

            // Move is allowed.
            moveTo(intersectPos.x, intersectPos.y, intersectPos.phi);
            return behTreeReturnEnum::RUNNING;
        }
        else
        {
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


