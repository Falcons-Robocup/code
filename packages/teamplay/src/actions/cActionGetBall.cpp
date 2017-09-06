 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionGetBall.cpp
 *
 *  Created on: May 4, 2016
 *      Author: Erik Kouters
 */

#include "int/actions/cActionGetBall.hpp"
#include "int/stores/ballStore.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/utilities/trace.hpp"
#include "int/stores/ownRobotStore.hpp"

using namespace teamplay;

cActionGetBall::cActionGetBall()
{
    boost::assign::insert( _actionParameters )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    intention.actionType = actionEnum::GET_BALL;
}

cActionGetBall::~cActionGetBall()
{

}

// Move to ballPos (x,y).
// Turn to look towards 'facing' (x,y).
// If no 'facing' was given, turn to look towards the ball.
// If no ball was found, do not turn (keep current facing).
// Returns: RUNNING   if the robot is not at the target position
//          PASSED    if the robot reached the target position
//          FAILED    if something went wrong
behTreeReturnEnum cActionGetBall::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos;
        cWorldModelInterface::getInstance().getOwnLocation(myPos);
        ball ball = ballStore::getBall();

        // parameter "motionProfile" defines with which profile to move with (e.g., normal play or more careful during a setpiece)
        std::string motionProfileStr("motionProfile");
        std::string motionProfileValue = "normal";
        auto paramValPair = parameters.find(motionProfileStr);
        if (paramValPair != parameters.end())
        {
            motionProfileValue = paramValPair->second;
        }

        if (ball.isLocationKnown())
        {
            Point3D ballPosition = ball.getPosition();
            double angle = angle_between_two_points_0_2pi(myPos.x, myPos.y, ballPosition.x, ballPosition.y);
            Position2D targetPos = Position2D(ballPosition.x, ballPosition.y, angle);

            ballPossession_struct_t ballPossession;
            cWorldModelInterface::getInstance().getBallPossession(ballPossession);

            auto own_robot_id = teamplay::ownRobotStore::getOwnRobot().getNumber();
            if ((ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER) && (ballPossession.robotID == own_robot_id))
            {
                // Got ball. Do nothing and return PASSED
                TRACE("cActionGetBall PASSED");
                return behTreeReturnEnum::PASSED;
            }
            else
            {
            	intention.x = targetPos.x;
            	intention.y = targetPos.y;
            	sendIntention();

                // ensure that we move according to the rules
                if (!isCurrentPosValid())
                {
                    // Move towards the center, while maintaining angle and motion profile
                    moveTo(0.0, 0.0, targetPos.phi, motionProfileValue);
                    return behTreeReturnEnum::RUNNING;
                }

                if (!isTargetPosInsideSafetyBoundaries(targetPos))
                {
                    return behTreeReturnEnum::FAILED;
                }

                // if ball is outside field, stop chasing the ball
                if (!ball.isInsideField())
                {
                	return behTreeReturnEnum::FAILED;
                }

                // Do not have ball. moveTo ball and return RUNNING
                moveTo(targetPos.x, targetPos.y, targetPos.phi, motionProfileValue);
                return behTreeReturnEnum::RUNNING;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}

