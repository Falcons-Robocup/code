 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionMoveToFreeSpot.cpp
 *
 *  Created on: May 22, 2016
 *      Author: Tim Kouters
 */

#include "int/actions/cActionMoveToFreeSpot.hpp"

#include <string>
#include <vector>

#include "int/stores/ballStore.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/algorithms/cPFM.hpp"
#include "int/utilities/trace.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/cWorldStateFunctions.hpp"

using namespace teamplay;
using std::vector;

cActionMoveToFreeSpot::cActionMoveToFreeSpot()
{
    boost::assign::insert( _actionParameters )
        ("facing", std::make_pair(defaultPOI, true) )
        ("distanceThreshold", std::make_pair(std::vector<std::string>{"float"}, true) )
        ("angleThreshold", std::make_pair(std::vector<std::string>{"float"}, true))
        ("areaOfInterest", std::make_pair(defaultArea, true))
        ;

    intention.actionType = actionEnum::MOVE_TO_FREE_SPOT;
}

cActionMoveToFreeSpot::~cActionMoveToFreeSpot()
{

}

/* \brief Move to a free spot in a certain area of interest
 *
 * \param areaOfInterest      The applicable area
 * \param facing              [Optional] The target (x,y) to face. If no 'facing' was given, face the ball (if found).
 * \param distanceThreshold   [Optional] The allowed delta between the target position and current position
 * \param angleThreshold      [Optional] The allowed delta between the target angle and current angle
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionMoveToFreeSpot::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
    	double Line_y = 4.0; // Horizontal line on which robot is searching for a free spot
    	// TODO: add as parameter to trees (low number > more long (lob)shots; high number > longer passes, more accurate shooting)

    	double dist_to_move_away = 2.0; // move away x [m] when there are obstacles in shoot path
    	float shoot_path_width = 0.5; // 0.5 [m] shoot path width to check if obstacles are blocking shoot path

    	// determine intersect result on Line_y on which robot (attacker) moves to free spot
    	ball ball = ballStore::getBall(); // retrieve ball position information
    	Vector2D ballPos(ball.getPosition().x, ball.getPosition().y);

    	Point2D oppGoalCenter = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALLINE_CENTER); // retrieve opp_goalline_center point
    	Vector2D goalPos(oppGoalCenter.x, oppGoalCenter.y);

    	Vector2D leftPosLine_y((fieldDimensionsStore::getFieldDimensions().getWidth() / -2.0), Line_y);
        Vector2D rightPosLine_y(fieldDimensionsStore::getFieldDimensions().getWidth() / 2.0, Line_y);

        Vector2D intersectResult;
        intersect(ballPos, goalPos, leftPosLine_y, rightPosLine_y, intersectResult);

        // create Point2D's for using wsf:getObstructingObstaclesInPath
        Point2D ballPos2D;
        ballPos2D.x = ball.getPosition().x;
        ballPos2D.y = ball.getPosition().y;
        Point2D intersectResult2D;
        intersectResult2D.x = intersectResult.x;
        intersectResult2D.y = intersectResult.y;

        // check if obstacles blocking shoot path to intended intersect position
        std::vector<robotLocation> obstacles;
        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(ballPos2D, intersectResult2D, shoot_path_width, obstacles);

    	if (obstacles.size() > 0)
    	{
    		if (ball.isAtLeftSide())
			{
    			intersectResult.x = intersectResult.x + dist_to_move_away;
			}
    		else
    		{
    			intersectResult.x = intersectResult.x - dist_to_move_away;
    		}
    	}

    	// Compute target positions
    	Position2D targetPos;
    	targetPos.x = intersectResult.x;
    	targetPos.y = intersectResult.y;

    	// Compute angle from robot to ball to ensure robot is always looking at the ball to receive a pass
        Position2D myPos;
        cWorldModelInterface::getInstance().getOwnLocation(myPos);
        targetPos.phi = angle_between_two_points_0_2pi(myPos.x, myPos.y, ballPos.x, ballPos.y);

    	// parameter "facing" is the place to look at.
		std::string facingStr("facing");
		boost::optional<Position2D> facing = getPos2DFromStr(parameters, facingStr);

		// parameter "distanceThreshold" is the allowed delta between the target and the real position
		std::string distanceThresholdStr("distanceThreshold");
		double distanceThreshold = XYpositionTolerance;
		auto paramValPair = parameters.find(distanceThresholdStr);
		if (paramValPair != parameters.end())
		{
			std::string distanceThresholdVal = paramValPair->second;
			if (distanceThresholdVal.compare(emptyValue) != 0)
			{
				distanceThreshold = std::stod(distanceThresholdVal);
			}
		}

		// parameter "angleThreshold" is the allowed delta between the target and the real angle
		std::string angleThresholdStr("angleThreshold");
		double angleThreshold = PHIpositionTolerance;
		paramValPair = parameters.find(angleThresholdStr);
		if (paramValPair != parameters.end())
		{
			std::string angleThresholdVal = paramValPair->second;
			if (angleThresholdVal.compare(emptyValue) != 0)
			{
				angleThreshold = std::stod(angleThresholdVal);
			}
		}

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

        if (positionReached(targetPos.x, targetPos.y, targetPos.phi, distanceThreshold, angleThreshold))
        {
            // Target reached. Do nothing and return PASSED
            TRACE("cActionMoveToFreeSpot PASSED");
            moveTo(myPos.x, myPos.y, myPos.phi);
            return behTreeReturnEnum::PASSED;
        }
        else
        {
            // Target not reached. moveTo and return RUNNING
            moveTo(targetPos.x, targetPos.y, targetPos.phi);
            return behTreeReturnEnum::RUNNING;
        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
