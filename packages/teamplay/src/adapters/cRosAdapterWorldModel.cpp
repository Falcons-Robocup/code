 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterWorldModel.cpp
 *
 *  Created on: Oct 7, 2015
 *      Author: Jan Feitsma
 */

#include "int/cRosAdapterWorldModel.hpp"
#include "int/cWorldModelInterface.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/teamMatesStore.hpp"

#include "int/types/cPositionTypes.hpp"
#include "int/types/cBallPossessionTypes.hpp"
#include "int/types/cRobotLocationTypes.hpp"

#include "position2d.hpp"
#include "vector3d.hpp"

#include "FalconsCommon.h"
#include "cDiagnosticsEvents.hpp"

#include "WorldModelNames.h"

void cb_worldModelUpdated(const worldModel::t_wmInfo::ConstPtr& msg);

cRosAdapterWorldModel::cRosAdapterWorldModel()
{
    /* Subscribe to worldModel pulse to fetch the new world model information	 */
    _wmInfo = _hROS.subscribe<worldModel::t_wmInfo>(WorldModelInterface::t_wmInfo, 1, &cb_worldModelUpdated);
}

void cRosAdapterWorldModel::update(const worldModel::t_wmInfo::ConstPtr& msg)
{
    teamplay::ballStore::getBall().reset();
    teamplay::teamMatesStore::getTeamMatesIncludingGoalie().clear();

	updateOwnLocation(msg);
	updateTeammembers(msg);
	updateOpponents(msg);
	updateBallLocation(msg);
	updateBallPossession(msg);
}

void cRosAdapterWorldModel::updateOwnLocation(const worldModel::t_wmInfo::ConstPtr& msg)
{
    teamplay::teamMatesStore::getTeamMatesIncludingGoalie().clear();
	// create objects
	Position2D robotPosition;
	Velocity2D robotVelocity;

	robotPosition.x = msg->locationX;
	robotPosition.y = msg->locationY;
	robotPosition.phi = msg->locationTheta;

	robotVelocity.x = msg->locationVx;
	robotVelocity.y = msg->locationVy;
	robotVelocity.phi = msg->locationVtheta;

	// call the old-style setters
	cWorldModelInterface::getInstance().setOwnLocation(robotPosition);
	TRACE("setOwnLocation: (%3.3f, %3.3f, %3.3f)", robotPosition.x, robotPosition.y, robotPosition.phi);

	cWorldModelInterface::getInstance().setOwnVelocity(robotVelocity);
    TRACE("setOwnVelocity: (%3.3f, %3.3f, %3.3f)", robotVelocity.x, robotVelocity.y, robotVelocity.phi);

    // call the new-style setter
    cWorldModelInterface::getInstance().setOwnRobot(robotPosition, robotVelocity);
}

void cRosAdapterWorldModel::updateTeammembers(const worldModel::t_wmInfo::ConstPtr& msg)
{
	// create objects
	robotLocations teammembers;

	for (size_t i = 0; i < msg->nrOfTeamMembers; i++)
	{
		robotLocation teammember;
		teammember.position = geometry::Pose2D(
				msg->teamMemberX.at(i),
				msg->teamMemberY.at(i),
				msg->teamMemberTheta.at(i));
		teammember.velocity = geometry::Velocity2D(
				msg->teamMemberVx.at(i),
				msg->teamMemberVy.at(i),
				msg->teamMemberVtheta.at(i));
		teammembers.insert(std::pair<robotNumber, robotLocation>((robotNumber) msg->teamMemberRobotID.at(i), teammember));
	}
	// call the setter
	cWorldModelInterface::getInstance().setTeammembers(teammembers, msg->activeRobots);
}

void cRosAdapterWorldModel::updateOpponents(const worldModel::t_wmInfo::ConstPtr& msg)
{
	// create objects
    robotLocations opponents;

	for (size_t i = 0; i < msg->nrObstacleMeasurements; i++)
	{
		robotLocation opponent;
		opponent.position = geometry::Pose2D(
				msg->obstacleX.at(i),
				msg->obstacleY.at(i),
				0.0);
		opponent.velocity = geometry::Velocity2D(
				msg->obstacleVX.at(i),
				msg->obstacleVY.at(i),
				0.0);
		opponents.insert(std::pair<int, robotLocation>(i, opponent));
	}

	// call the setter
	cWorldModelInterface::getInstance().setOpponents(opponents);
}

void cRosAdapterWorldModel::updateBallLocation(const worldModel::t_wmInfo::ConstPtr& msg)
{
	// create objects
	ballLocations ball_locations;
    ballLocation last_known_ball_location;

	// call services and fill the objects
	if(msg->isBallValid)
	{
		ballLocation ball_location;

		ball_location.position.x = msg->ballX;
		ball_location.position.y = msg->ballY;
		ball_location.position.z = msg->ballZ;
		ball_location.velocity.x = msg->ballVX;
		ball_location.velocity.y = msg->ballVY;
		ball_location.velocity.z = msg->ballVZ;
		ball_location.confidence = msg->ballConfidence;
		ball_locations.push_back(ball_location);
		TRACE("ball: position: (%3.3f, %3.3f, %3.3f) velocity: (%3.3f, %3.3f, %3.3f) confidence: %3.3f",
		        ball_location.position.x, ball_location.position.y, ball_location.position.z,
		        ball_location.velocity.x, ball_location.velocity.y, ball_location.velocity.z,
		        ball_location.confidence);
	}
	else
	{
	    TRACE("ball: no valid ball received");
	}

    /* TODO
    key = WorldModelInterface::s_get_last_known_ball_location;
    if (!_serviceSubscriptions[key].call(srv_last_known_ball_location))
    {
        TRACE_ERROR("service call failed, key='%s'", key.c_str());
    }

    last_known_ball_location.position.x = srv_last_known_ball_location.response.ballPos.Pos.x;
    last_known_ball_location.position.y = srv_last_known_ball_location.response.ballPos.Pos.y;
    last_known_ball_location.position.z = srv_last_known_ball_location.response.ballPos.Pos.z;
    last_known_ball_location.velocity.x = srv_last_known_ball_location.response.ballPos.Vel.x;
    last_known_ball_location.velocity.y = srv_last_known_ball_location.response.ballPos.Vel.y;
    last_known_ball_location.velocity.z = srv_last_known_ball_location.response.ballPos.Vel.z;
    last_known_ball_location.confidence = srv_last_known_ball_location.response.ballPos.confidence;
    */

	// call the setter
	cWorldModelInterface::getInstance().setBallLocation(ball_locations, last_known_ball_location);
}

void cRosAdapterWorldModel::updateBallPossession(const worldModel::t_wmInfo::ConstPtr& msg)
{
	// create objects
	ballPossession_struct_t ball_possession;
	Point3D ball_claimed_location;

	ball_possession.robotID = 255;
	ball_claimed_location.x = 0.0;
	ball_claimed_location.y = 0.0;
	ball_claimed_location.z = 0.0;

	switch (msg->possession.type)
	{
    case rosMsgs::BallPossession::TYPE_INVALID:
        ball_possession.possessionType = ballPossessionEnum::INVALID;
        TRACE("setBallPossession: invalid ball possession type received");
        break;
    case rosMsgs::BallPossession::TYPE_FIELD:
        ball_possession.possessionType = ballPossessionEnum::FIELD;
        TRACE("setBallPossession: field");
        break;
    case rosMsgs::BallPossession::TYPE_TEAMMEMBER:
        ball_possession.possessionType = ballPossessionEnum::TEAMMEMBER;
        ball_possession.robotID = (robotNumber) msg->possession.robotID;
        ball_claimed_location.x = msg->possessionX;
        ball_claimed_location.y = msg->possessionY;
        ball_claimed_location.z = 0.0;
        TRACE("setBallPossession: teammember [%d]", ball_possession.robotID);
        TRACE("setBallClaimedLocation: (%3.3f, %3.3f, %3.3f)", ball_claimed_location.x, ball_claimed_location.y, ball_claimed_location.z);
        break;
    case rosMsgs::BallPossession::TYPE_OPPONENT:
        ball_possession.possessionType = ballPossessionEnum::FIELD;
        TRACE("setBallPossession: field");
        TRACE("TODO: do something smarter because this is actually OPPONENT ball possession");
        break;
    default:
        TRACE_ERROR("unknown ball possession type received, type='%d'", msg->possession.type);
        break;
	}

	// call the setters
	cWorldModelInterface::getInstance().setBallPossession(ball_possession);
	cWorldModelInterface::getInstance().setBallClaimedLocation(ball_claimed_location);
}
