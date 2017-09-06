 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionGoalKeeper.cpp
 *
 *  Created on: May 4, 2016
 *      Author: Tim Kouters
 */
#include "int/actions/cActionGoalKeeper.hpp"

#include "int/stores/ballStore.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/utilities/trace.hpp"
#include "int/cTeamplayCommon.hpp"

using namespace teamplay;

cActionGoalKeeper::cActionGoalKeeper()
{
	boost::assign::insert( _actionParameters )
		("interceptCaptureRadius", std::make_pair(std::vector<std::string>{"float"}, true) )
		("interceptMinimumSpeed", std::make_pair(std::vector<std::string>{"float"}, true))
		;

	intention.actionType = actionEnum::GOALKEEPER;
}

cActionGoalKeeper::~cActionGoalKeeper()
{

}

// Always position between goal and ball
// Always face forwards (away from goal)
// If no ball was found, goto middle of the goal
// Returns: RUNNING   always, action does not end

behTreeReturnEnum cActionGoalKeeper::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
    	// parameter "interceptCaptureRadius" is the radius to capture the ball
		std::string interceptCaptureRadiusStr("interceptCaptureRadius");
		double interceptCaptureRadius = 0.0;
		auto paramValPair = parameters.find(interceptCaptureRadiusStr);
		if (paramValPair != parameters.end())
		{
			std::string interceptCaptureRadiusVal = paramValPair->second;
			if (interceptCaptureRadiusVal.compare(emptyValue) != 0)
			{
				interceptCaptureRadius = std::stod(interceptCaptureRadiusVal);
			}
		}

		// parameter "interceptMinimumSpeed" is the minimum speed to capture the ball
		std::string interceptMinimumSpeedStr("interceptMinimumSpeed");
		double interceptMinimumSpeed = 0.0;
		paramValPair = parameters.find(interceptMinimumSpeedStr);
		if (paramValPair != parameters.end())
		{
			std::string interceptMinimumSpeedVal = paramValPair->second;
			if (interceptMinimumSpeedVal.compare(emptyValue) != 0)
			{
				interceptMinimumSpeed = std::stod(interceptMinimumSpeedVal);
			}
		}


    	// ball position
        ball ball = ballStore::getBall();

        // own position
        Position2D myPos;
        cWorldModelInterface::getInstance().getOwnLocation(myPos);

        // By default (no ball visible) goto center of goal
        Point2D goalCenter = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALLINE_CENTER);
        Position2D targetPos = Position2D(goalCenter.x, goalCenter.y + _Y_MAX_OFFSET_KEEPER, M_PI_2); // Face forward

        if (ball.isLocationKnown())
        {
        	// Goalpost parameters
	        Point2D goalPostLeft = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALPOST_LEFT);
	        Point2D goalPostRight = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALPOST_RIGHT);

        	// determine relative speed
            geometry::Pose2D currentPose(myPos.x, myPos.y, myPos.phi);
            geometry::Velocity2D ballVelocity(ball.getVelocity().x, ball.getVelocity().y, 0.0);
        	geometry::Velocity2D ballVelocityRCS = ballVelocity.transformFCS2RCS(currentPose);
        	float ballSpeed = ballVelocity.size();
        	bool ballIsMovingTowardsUs = (ballVelocityRCS.y < 0.0);
        	bool ballMovingFastEnough = (ballSpeed > interceptMinimumSpeed);

        	// Intercept if ball is moving fast enough towards us on own half
        	if (ballIsMovingTowardsUs && ballMovingFastEnough && ball.isAtOwnSide())
        	{
                Vector2D ballPos(ball.getPosition().x, ball.getPosition().y);
                Vector2D ballSpeed(ball.getVelocity().x, ball.getVelocity().y);

                // projection of line on which goalkeeper moves
                Vector2D leftPosGoalkeeper((fieldDimensionsStore::getFieldDimensions().getWidth() / -2.0), goalCenter.y + _Y_MAX_OFFSET_KEEPER);
                Vector2D rightPosGoalkeeper(fieldDimensionsStore::getFieldDimensions().getWidth() / 2.0, goalCenter.y + _Y_MAX_OFFSET_KEEPER);

				// intersect
				Vector2D ballProjection = ballPos + 10.0 * ballSpeed; // make vector long enough
				Vector2D intersectResult;
				if (intersect(ballPos, ballProjection, leftPosGoalkeeper, rightPosGoalkeeper, intersectResult))
				{
					if ((intersectResult - myPos.xy()).size() < interceptCaptureRadius)
					{
						TRACE("intercept: ") << std::to_string(intersectResult.x) << ", " << std::to_string(intersectResult.y);
						targetPos.x = fmax(intersectResult.x, goalPostLeft.x + _GOALPOST_OFFSET_KEEPER);
						targetPos.x = fmin(targetPos.x, goalPostRight.x - _GOALPOST_OFFSET_KEEPER);
						targetPos.y = intersectResult.y;
					}
				}
        	}
        	else
        	{
        		Position2D newPos = targetPos;
        		// X distance between ball and goalline
				float ballDistanceX = ball.getPosition().x - goalCenter.x;
				// Y distance between ball from goalline
				float ballDistanceY = ball.getPosition().y - goalCenter.y;
				float offsetY= 2.0;
				newPos.x = (offsetY * ballDistanceX)/(offsetY + ballDistanceY);
				newPos.x = fmax(newPos.x, goalPostLeft.x + _GOALPOST_OFFSET_KEEPER);
				newPos.x = fmin(newPos.x, goalPostRight.x - _GOALPOST_OFFSET_KEEPER);
				newPos.y = goalPostLeft.y + _Y_MAX_OFFSET_KEEPER;
				targetPos = newPos;
        	}
        }

        intention.x = targetPos.x;
        intention.y = targetPos.y;
        sendIntention();

        moveTo(targetPos.x, targetPos.y, targetPos.phi);
        return behTreeReturnEnum::RUNNING;
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
