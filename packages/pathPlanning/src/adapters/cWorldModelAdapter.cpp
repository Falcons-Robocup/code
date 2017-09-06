 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelAdapter.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/adapters/cWorldModelAdapter.hpp"

#include "FalconsCommon.h"
#include <cDiagnosticsEvents.hpp>

#include "WorldModelNames.h"

using std::runtime_error;

cWorldModelAdapter::cWorldModelAdapter()
{
    // Empty implementation. Defined to allow this class to be globally defined.
	_robotID = 0;
	_ppData = NULL;
}

cWorldModelAdapter::cWorldModelAdapter(cPathPlanningData &data, iterateFunctionType func)
{
    TRACE(">");
    _ppData = &data;
    _iterateFunc = func;
    _robotID = getRobotNumber();
    TRACE("< robotID: %d", _robotID);
}

cWorldModelAdapter::~cWorldModelAdapter()
{
    ;
}

void cWorldModelAdapter::initializeWM()
{
    TRACE(">");

    _hROS.reset(new ros::NodeHandle());
    _subWmInfo = _hROS->subscribe(WorldModelInterface::t_wmInfo, 1, &cWorldModelAdapter::worldModel_cb, this);

    TRACE("<");
}

void cWorldModelAdapter::worldModel_cb(const worldModel::t_wmInfo::ConstPtr& msg)
{
	fetchPositionAndVelocity(msg);
    fetchObstacles(msg);
    fetchRobotActive(msg);
    fetchBall(msg);
    fetchOwnRobotLowestActive(msg);
}

void cWorldModelAdapter::fetchPositionAndVelocity(const worldModel::t_wmInfo::ConstPtr& msg)
{
    //TRACE(">");

    Position2D pos(
    		msg->locationX,
    		msg->locationY,
    		msg->locationTheta);

    Velocity2D vel(
    		msg->locationVx,
    		msg->locationVy,
    		msg->locationVtheta);

    _ppData->setPosition(pos);
    _ppData->setVelocity(vel);
    TRACE("< pos = (%5.2f,%5.2f,%5.2f), vel = (%5.2f,%5.2f,%5.2f)", pos.x, pos.y, pos.phi, vel.x, vel.y, vel.phi);

    //TRACE("<");
}

void cWorldModelAdapter::projectObstacles(const pp_obstacle_struct_t& obst, const pp_limiters_struct_t& limits, std::vector<pp_obstacle_struct_t>& obstacles, std::vector<linepoint2D>& projectedSpeedPoint)
{
	// For teammembers, take the velocity into account and reposition them to the future position using a scalar.
	pp_obstacle_struct_t projectedObst;
	Position2D robotPos;
	_ppData->getPosition(robotPos);

	Vector2D obstPos = Vector2D(obst.location.x, obst.location.y);
	Vector2D obstVel = Vector2D(obst.velocity.x, obst.velocity.y);
	double distToObst = calc_hypothenusa( (robotPos.x - obstPos.x), (robotPos.y - obstPos.y) );
	distToObst = fmax(distToObst, 1.0);

	Vector2D projectionDistance = (limits.obstacleAvoidanceScalingFactor * (distToObst * limits.obstacleAvoidanceDistanceFactor) * obstVel);
	Vector2D projectedObstPos = obstPos + projectionDistance;
	projectedObst.location.x = projectedObstPos.x;
	projectedObst.location.y = projectedObstPos.y;
	projectedObst.location.phi = obst.location.phi;
	projectedObst.velocity.x = 0.0;
	projectedObst.velocity.y = 0.0;
	projectedObst.velocity.phi = 0.0;
	projectedObst.type = pp_obstacle_type::PROJECTED;

	obstacles.push_back(projectedObst);

	linepoint2D projectedLinepoint(obst.location.x, obst.location.y, projectedObstPos.x, projectedObstPos.y);
	projectedSpeedPoint.push_back(projectedLinepoint);

	// Fill the line from obstacle to projectedObstacle with obstacles such that the robot does not decide to cross a teammember's trajectory.
	Vector2D currentProjectionDistance;
	double distBetweenProjectedObstacles = (ROBOT_RADIUS);
	for (size_t j = 1; (currentProjectionDistance.size() + distBetweenProjectedObstacles) < projectionDistance.size(); j++)
	{
		//TRACE("j=%d ; currProjDist=%12.9f ; projDist=%12.9f", j, currentProjectionDistance.size(), projectionDistance.size());
		currentProjectionDistance = obstVel.normalized() * ( j * distBetweenProjectedObstacles );
		Vector2D currentProjectedObstPos = obstPos + currentProjectionDistance;

		pp_obstacle_struct_t currentProjectedObst;
		currentProjectedObst.location.x = currentProjectedObstPos.x;
		currentProjectedObst.location.y = currentProjectedObstPos.y;
		currentProjectedObst.location.phi = obst.location.phi;
		currentProjectedObst.velocity.x = 0.0;
		currentProjectedObst.velocity.y = 0.0;
		currentProjectedObst.velocity.phi = 0.0;
		projectedObst.type = pp_obstacle_type::PROJECTED;

		obstacles.push_back(currentProjectedObst);
	}
}

void cWorldModelAdapter::fetchObstacles(const worldModel::t_wmInfo::ConstPtr& msg)
{
    //TRACE(">");

    // Get scalar for taking velocity into account for obstacle avoidance.
    pp_limiters_struct_t limits;
    _ppData->getLimits(limits);

    std::vector<pp_obstacle_struct_t> obstacles;
    std::vector<linepoint2D> projectedSpeedVectors;

    for (size_t i = 0; i < msg->nrObstacleMeasurements; ++i)
    {
        pp_obstacle_struct_t newObst;
        newObst.location.x = msg->obstacleX.at(i);
        newObst.location.y = msg->obstacleY.at(i);
        newObst.location.phi = 0.0;
        newObst.velocity.x = msg->obstacleVX.at(i);
        newObst.velocity.y = msg->obstacleVY.at(i);
        newObst.velocity.phi = 0.0;
        newObst.type = pp_obstacle_type::WORLDMODEL;
        obstacles.push_back(newObst);

        // Based on the velocity of the obstacle, project obstacles to avoid its path
		projectObstacles(newObst, limits, obstacles, projectedSpeedVectors);
    }

    for (size_t i = 0; i < msg->nrOfTeamMembers; ++i)
    {
        pp_obstacle_struct_t newObst;
        newObst.location.x = msg->teamMemberX.at(i);
        newObst.location.y = msg->teamMemberY.at(i);
        newObst.location.phi = msg->teamMemberTheta.at(i);
        newObst.velocity.x = msg->teamMemberVx.at(i);
        newObst.velocity.y = msg->teamMemberVy.at(i);
        newObst.velocity.phi = msg->teamMemberVtheta.at(i);
        newObst.type = pp_obstacle_type::WORLDMODEL;
        obstacles.push_back(newObst);

        // Based on the velocity of the teammember, project obstacles to avoid its path
        projectObstacles(newObst, limits, obstacles, projectedSpeedVectors);
    }

    _ppData->setProjectedSpeedVectors(projectedSpeedVectors);

    // Update diagnosticsAdapter with latest version of which obstacles pathplanning sees
    std::vector<polygon2D> areas;
    _ppData->getForbiddenAreas(areas);
    _ppData->publishObstacles(obstacles, areas, projectedSpeedVectors);

    _ppData->setObstacles(obstacles);
    _ppData->setProjectedSpeedVectors(projectedSpeedVectors);

    //TRACE("<");
}

void cWorldModelAdapter::fetchRobotActive(const worldModel::t_wmInfo::ConstPtr& msg)
{
    //TRACE(">");
    bool isActive = false;

    if(msg->robotStatus.status_type == rosMsgs::s_robot_status::TYPE_INPLAY)
    {
    	isActive = true;
    }

    _ppData->setRobotActive(isActive);

    //TRACE("< isActive = %d", isActive);
}

void cWorldModelAdapter::fetchBall(const worldModel::t_wmInfo::ConstPtr& msg)
{
    //TRACE(">");

    bool hasBall = false;
    if ((msg->possession.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER) && (msg->possession.robotID == _robotID))
    {
        // robot has the ball
        hasBall = true;
    }

    _ppData->setHaveBall(hasBall);
    //TRACE("< hasBall = %d", hasBall);

    Position2D ballPos;
    ballPos.x = msg->ballX;
    ballPos.y = msg->ballY;
    _ppData->setBallPos(ballPos);

    _ppData->setBallValid(msg->isBallValid);

    //TRACE("<");
}

void cWorldModelAdapter::fetchOwnRobotLowestActive(const worldModel::t_wmInfo::ConstPtr& msg)
{
    try
    {
    	bool isCurrentLowestRobot = false;

    	if( (msg->activeRobots.size() > 0) && (_robotID == msg->activeRobots.at(0)))
    	{
    		isCurrentLowestRobot = true;
    	}

    	_ppData->setIsLowestActiveRobot(isCurrentLowestRobot);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}



