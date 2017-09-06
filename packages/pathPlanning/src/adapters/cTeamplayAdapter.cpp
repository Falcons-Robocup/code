 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTeamplayAdapter.cpp
 *
 *  Created on: Sept 06, 2015
 *      Author: Erik Kouters
 */

#include "int/adapters/cTeamplayAdapter.hpp"

#include "polygon2D.hpp"
#include "cDiagnosticsEvents.hpp"


static pp_algorithm_turn_type rosToTurnType(int rosTurnType)
{
   switch(rosTurnType)
    {
        case 0:
        {
            return normal;
            break;
        }
        case 1:
        {
            return tokyo_drift;
            break;
        }
        default:
        {
            ROS_ERROR("Invalid algorithm turn type received");
            break;
        }
    }
   return normal;
}

static pp_algorithm_coord_type rosToCoordType(int rosCoordType)
{
   switch(rosCoordType)
    {
        case 0:
        {
            return robot_coord;
            break;
        }
        case 1:
        {
            return field_coord;
            break;
        }
        default:
        {
            ROS_ERROR("Invalid algorithm turn type received");
            break;
        }
    }
   return robot_coord;
}

cTeamplayAdapter::cTeamplayAdapter()
{
    // Empty implementation. Defined to allow this class to be globally defined.
}

cTeamplayAdapter::cTeamplayAdapter(cPathPlanningData &data, iterateFunctionType func)
{
    TRACE(">");
    _ppData = &data;
    _iterateFunc = func;
    TRACE("<");
}

cTeamplayAdapter::~cTeamplayAdapter()
{
}

void cTeamplayAdapter::initializeTP()
{
    TRACE(">");

    _n.reset(new ros::NodeHandle());
    /*
    _subTarget.reset(new ros::Subscriber());
    _srvGetActive.reset(new ros::ServiceServer());
    _srvSetActive.reset(new ros::ServiceServer());
    _srvMoveWhileTurning.reset(new ros::ServiceServer());
    _srvTurnThenMove.reset(new ros::ServiceServer());
    _srvMoveThenTurn.reset(new ros::ServiceServer());
    _srvMoveAtSpeed.reset(new ros::ServiceServer());
    _srvTurn.reset(new ros::ServiceServer());
    */


    /* Initialize receiving new target positions */
    _subTarget = _n->subscribe("g_target", MOTION_FREQUENCY, &cTeamplayAdapter::targetCallback, this);
    _srvGetActive = _n->advertiseService("s_pathplanning_get_active", &cTeamplayAdapter::cb_get_active_service, this);
    _srvSetActive = _n->advertiseService("s_pathplanning_set_active", &cTeamplayAdapter::cb_set_active_service, this);
    _srvMoveWhileTurning = _n->advertiseService("s_pathplanning_move_while_turning", &cTeamplayAdapter::cb_move_while_turning_service, this);
    _srvTurnThenMove = _n->advertiseService("s_pathplanning_turn_then_move", &cTeamplayAdapter::cb_turn_then_move_service, this);
    _srvMoveThenTurn = _n->advertiseService("s_pathplanning_move_then_turn", &cTeamplayAdapter::cb_move_then_turn_service, this);
    _srvMoveAtSpeed = _n->advertiseService("s_pathplanning_move_at_speed", &cTeamplayAdapter::cb_move_at_speed_service, this);
    _srvTurn = _n->advertiseService("s_pathplanning_turn", &cTeamplayAdapter::cb_turn_service, this);

    TRACE("<");
}

bool cTeamplayAdapter::cb_get_active_service(pathPlanning::s_pathplanning_get_active::Request& request, pathPlanning::s_pathplanning_get_active::Response& response)
{

    bool activated;
    _ppData->getPathPlanningActivated(activated);
    response.active = activated;

    return true;
}

bool cTeamplayAdapter::cb_set_active_service(pathPlanning::s_pathplanning_set_active::Request& request, pathPlanning::s_pathplanning_set_active::Response& response)
{
    bool activated = request.active;
    _ppData->setPathPlanningActivated(activated);

    if (activated == false)
    {
        // Pathplanning being disabled. Publish velocity 0 m/s to stop the robot (and not use the watchdog from peripheralsInterface).
        Velocity2D vel(0.0, 0.0, 0.0);
        _ppData->publishSpeed(vel);
    }

    return true;
}

void cTeamplayAdapter::targetCallback(const rosMsgs::t_target::ConstPtr& msg)
{
    TRACE("> target = (%5.2f,%5.2f,%5.2f)", msg->x, msg->y, msg->phi);

    Position2D target(msg->x, msg->y, msg->phi);
    _ppData->setTarget(target);
    _ppData->setMotionProfileType((pp_motionProfile_type)msg->motion_profile);

    /* Fetch forbidden areas */
    std::vector<polygon2D> forbiddenAreas;
	for(auto it = msg->forbiddenAreas.begin(); it != msg->forbiddenAreas.end(); it++)
	{
		polygon2D polygon;
		for(auto polyIt = it->points.begin(); polyIt != it->points.end(); polyIt++)
		{
			polygon.addPoint(polyIt->x, polyIt->y);
		}
		forbiddenAreas.push_back(polygon);
	}
	_ppData->setDynamicForbiddenAreas(forbiddenAreas);

    _iterateFunc();

    TRACE("<");
}

bool cTeamplayAdapter::cb_move_while_turning_service(pathPlanning::s_pathplanning_move_while_turning::Request& request, pathPlanning::s_pathplanning_move_while_turning::Response& response)
{
    TRACE("> target = (%5.2f,%5.2f,%5.2f)", request.pos.x, request.pos.y, request.pos.theta);

    Position2D target(request.pos.x, request.pos.y, request.pos.theta);
    _ppData->setTarget(target);
    //_ppData->setAlgorithmType(moveWhileTurning);
    _iterateFunc();

    // Block service from returning until robot has reached target.
    // Warning: This does make pathplanning inresponsive.
    Position2D targetPosition;
    _ppData->getTarget(targetPosition);

    Position2D currentPosition;
    _ppData->getPosition(currentPosition);

    //pp_limiters_struct_t limits;
   // _ppData->getLimits(limits);

    /* Fetch forbidden areas */
    std::vector<polygon2D> forbiddenAreas;
	for(auto it = request.forbiddenAreas.begin(); it != request.forbiddenAreas.end(); it++)
	{
		polygon2D polygon;
		for(auto polyIt = it->points.begin(); polyIt != it->points.end(); polyIt++)
		{
			polygon.addPoint(polyIt->x, polyIt->y);
		}
		forbiddenAreas.push_back(polygon);
	}
	_ppData->setDynamicForbiddenAreas(forbiddenAreas);

	_ppData->setMotionProfileType((pp_motionProfile_type)request.motion_profile);
    // Sleep while target not reached
    while ( ((targetPosition.xy() - currentPosition.xy()).size() > 0.1) ||
            (fabs(project_angle_mpi_pi(targetPosition.phi - currentPosition.phi)) > 0.03) )
    {
        usleep(1000 * 33); // 33ms
        _ppData->getPosition(currentPosition);
        _ppData->getTarget(targetPosition);
        //_ppData->getLimits(limits);

        bool ppActive = false;
        _ppData->getPathPlanningActivated(ppActive);
        if (!ppActive)
        {
            _ppData->setPathPlanningActivated(true);
        }

        _iterateFunc();
    }

    // Reached target, set target to current position to trigger speed setpoint of 0 m/s.
    _ppData->getPosition(currentPosition);
    _ppData->setTarget(currentPosition);

    TRACE("<");
    return true;
}

bool cTeamplayAdapter::cb_turn_then_move_service(pathPlanning::s_pathplanning_turn_then_move::Request& request, pathPlanning::s_pathplanning_turn_then_move::Response& response)
{
    TRACE(">");
    Position2D target(request.pos.x, request.pos.y, request.pos.theta);
    _ppData->setTarget(target);
    _ppData->setTurnType(rosToTurnType(request.turn_type));

    /* Fetch forbidden areas */
    std::vector<polygon2D> forbiddenAreas;
	for(auto it = request.forbiddenAreas.begin(); it != request.forbiddenAreas.end(); it++)
	{
		polygon2D polygon;
		for(auto polyIt = it->points.begin(); polyIt != it->points.end(); polyIt++)
		{
			polygon.addPoint(polyIt->x, polyIt->y);
		}
		forbiddenAreas.push_back(polygon);
	}
	_ppData->setDynamicForbiddenAreas(forbiddenAreas);

//    _targetPosition.x = request.pos.x;
//    _targetPosition.y = request.pos.y;
//    _targetPosition.phi = request.pos.theta;
//    _turnType = ;

    //_callbackFunc(turnThenMove);
    TRACE("<");
    return true;
}

bool cTeamplayAdapter::cb_move_then_turn_service(pathPlanning::s_pathplanning_move_then_turn::Request& request, pathPlanning::s_pathplanning_move_then_turn::Response& response)
{
    TRACE(">");
    Position2D target(request.pos.x, request.pos.y, request.pos.theta);
    _ppData->setTarget(target);
    _ppData->setTurnType(rosToTurnType(request.turn_type));

    /* Fetch forbidden areas */
    std::vector<polygon2D> forbiddenAreas;
	for(auto it = request.forbiddenAreas.begin(); it != request.forbiddenAreas.end(); it++)
	{
		polygon2D polygon;
		for(auto polyIt = it->points.begin(); polyIt != it->points.end(); polyIt++)
		{
			polygon.addPoint(polyIt->x, polyIt->y);
		}
		forbiddenAreas.push_back(polygon);
	}
	_ppData->setDynamicForbiddenAreas(forbiddenAreas);

//    _targetPosition.x = request.pos.x;
//    _targetPosition.y = request.pos.y;
//    _targetPosition.phi = request.pos.theta;
//    _turnType = rosToTurnType(request.turn_type);

    //_callbackFunc(moveThenTurn);
    TRACE("<");
    return true;
}

bool cTeamplayAdapter::cb_move_at_speed_service(pathPlanning::s_pathplanning_move_at_speed::Request& request, pathPlanning::s_pathplanning_move_at_speed::Response& response)
{
    TRACE(">");
    Position2D target(request.vel.x, request.vel.y, request.vel.z);
    _ppData->setTarget(target);
    _ppData->setCoordType(rosToCoordType(request.coord_type));

    /* Fetch forbidden areas */
    std::vector<polygon2D> forbiddenAreas;
	for(auto it = request.forbiddenAreas.begin(); it != request.forbiddenAreas.end(); it++)
	{
		polygon2D polygon;
		for(auto polyIt = it->points.begin(); polyIt != it->points.end(); polyIt++)
		{
			polygon.addPoint(polyIt->x, polyIt->y);
		}
		forbiddenAreas.push_back(polygon);
	}
	_ppData->setDynamicForbiddenAreas(forbiddenAreas);
//    _targetPosition.x = request.vel.x;
//    _targetPosition.y = request.vel.y;
//    _targetPosition.phi = request.vel.z;
//    _coordType = ;

    //_callbackFunc(moveAtSpeed);
    TRACE("<");
    return true;
}

bool cTeamplayAdapter::cb_turn_service(pathPlanning::s_pathplanning_turn::Request& request, pathPlanning::s_pathplanning_turn::Response& response)
{
    TRACE(">");
    Position2D target(0, 0, request.pos);
   _ppData->setTarget(target);
   _ppData->setTurnType(rosToTurnType(request.turn_type));

   /* Fetch forbidden areas */
    std::vector<polygon2D> forbiddenAreas;
	for(auto it = request.forbiddenAreas.begin(); it != request.forbiddenAreas.end(); it++)
	{
		polygon2D polygon;
		for(auto polyIt = it->points.begin(); polyIt != it->points.end(); polyIt++)
		{
			polygon.addPoint(polyIt->x, polyIt->y);
		}
		forbiddenAreas.push_back(polygon);
	}
	_ppData->setDynamicForbiddenAreas(forbiddenAreas);
//    _targetPosition.x = 0;
//    _targetPosition.y = 0;
//    _targetPosition.phi = request.pos;
//    _turnType = rosToTurnType(request.turn_type);

    //_callbackFunc(turn);
   TRACE("<");
    return true;
}
