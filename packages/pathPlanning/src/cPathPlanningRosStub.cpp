 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningRosStub.cpp
 *
 * Implementations for cPathPlanningRosStub.
 *
 *  Created on: Dec 21, 2015
 *      Author: Coen Tempelaars
 */


#include "ext/cPathPlanningNames.hpp"
#include "ext/cPathPlanningRosStub.hpp"

#include "pathPlanning/s_pathplanning_move_at_speed.h"
#include "pathPlanning/s_pathplanning_move_then_turn.h"
#include "pathPlanning/s_pathplanning_move_while_turning.h"
#include "pathPlanning/s_pathplanning_turn.h"
#include "pathPlanning/s_pathplanning_turn_then_move.h"
#include "rosMsgs/t_target.h"

#include "tracing.hpp"

bool _isactive = false;
geometry::Pose2D _pose = geometry::Pose2D();

bool moveAtSpeedCallback(pathPlanning::s_pathplanning_move_at_speed::Request &req,
        pathPlanning::s_pathplanning_move_at_speed::Response &resp)
{
    TRACE("ros stub moveAtSpeedCallback(coord_type: %d, vel: (%3.3f, %3.3f, %3.3f))", req.coord_type, req.vel.x, req.vel.y, req.vel.z);
    //todo: store something
    return true;
}

bool moveThenTurnCallback(pathPlanning::s_pathplanning_move_then_turn::Request &req,
        pathPlanning::s_pathplanning_move_then_turn::Response &resp)
{
    TRACE("ros stub moveThenTurnCallback(turn_type: %d, pos: (%3.3f, %3.3f, %3.3f))", req.turn_type, req.pos.x, req.pos.y, req.pos.theta);
    //todo: store something
    return true;
}

bool moveWhileTurningCallback(pathPlanning::s_pathplanning_move_while_turning::Request &req,
        pathPlanning::s_pathplanning_move_while_turning::Response &resp)
{
    TRACE("ros stub moveWhileTurningCallback(pos: (%3.3f, %3.3f, %3.3f))", req.pos.x, req.pos.y, req.pos.theta);
    _pose = geometry::Pose2D(req.pos.x, req.pos.y, req.pos.theta);
    return true;
}

bool turnCallback(pathPlanning::s_pathplanning_turn::Request &req,
        pathPlanning::s_pathplanning_turn::Response &resp)
{
    TRACE("ros stub turnCallback(turn_type: %d, target angle: %3.3f)", req.turn_type, req.pos);
    //todo: store something
    return true;
}

bool turnThenMoveCallback(pathPlanning::s_pathplanning_turn_then_move::Request &req,
        pathPlanning::s_pathplanning_turn_then_move::Response &resp)
{
    TRACE("ros stub turnThenMoveCallback(turn_type, pos: (%3.3f, %3.3f, %3.3f))", req.turn_type, req.pos.x, req.pos.y, req.pos.theta);
    //todo: store something
    return true;
}

void targetCallback(const rosMsgs::t_target::ConstPtr& target)
{
    TRACE("ros stub targetCallback(%3.3f, %3.3f, %3.3f)", target->x, target->y, target->phi);
    _pose = geometry::Pose2D(target->x, target->y, target->phi);
    return;
}


void cPathPlanningRosStub::setupServices()
{
    TRACE("setting up services...");
    _services.push_back(_nh.advertiseService(PathPlanningInterface::s_pathplanning_move_at_speed, moveAtSpeedCallback));
    _services.push_back(_nh.advertiseService(PathPlanningInterface::s_pathplanning_move_then_turn, moveThenTurnCallback));
    _services.push_back(_nh.advertiseService(PathPlanningInterface::s_pathplanning_move_while_turning, moveWhileTurningCallback));
    _services.push_back(_nh.advertiseService(PathPlanningInterface::s_pathplanning_turn, turnCallback));
    _services.push_back(_nh.advertiseService(PathPlanningInterface::s_pathplanning_turn_then_move, turnThenMoveCallback));
    TRACE("done with setting up services");
}

void cPathPlanningRosStub::subscribeToTopics()
{
    TRACE("subscribing to topics...");
    _topics.push_back(_nh.subscribe("g_target", 1000, targetCallback));
    TRACE("done subscribing to topics");
}

void cPathPlanningRosStub::reset()
{
    _isactive = false;
    _pose = geometry::Pose2D();
}

geometry::Pose2D cPathPlanningRosStub::getPose()
{
    return _pose;
}

