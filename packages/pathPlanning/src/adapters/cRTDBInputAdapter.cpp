 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/cRTDBInputAdapter.hpp"

#include "FalconsCommon.h" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(cPathPlanningData &data, iterateFunctionType func)
{
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _ppData = &data;
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
    _iterateFunc = func;
    _wmClient = new cWorldModelClient();
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForMotionSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(MOTION_SETPOINT);
        getMotionSetpoint();
        getForbiddenAreas();
        getWorldModelData();

        _iterateFunc();
    }
}

void cRTDBInputAdapter::getMotionSetpoint()
{
    TRACE_FUNCTION("");
    T_MOTION_SETPOINT motionSetpoint;

    int r = _rtdb->get(MOTION_SETPOINT, &motionSetpoint);

    if (r == RTDB2_SUCCESS)
    {
        tprintf("get MOTION_SETPOINT action=%d pos=[%6.2f, %6.2f, %6.2f] slow=%d", (int)motionSetpoint.action, motionSetpoint.position.x, motionSetpoint.position.y, motionSetpoint.position.z, motionSetpoint.slow);
        switch(motionSetpoint.action)
        {
            case actionTypeEnum::STOP:
            {
                Position2D robotPos;
                _ppData->getPosition(robotPos);


                std::stringstream str;
                str << "target=" << robotPos.tostr();
                TRACE_SCOPE("PP_TARGET_STOP", str.str().c_str());


                _ppData->setRobotStop(true);

                // Set target to current position
                _ppData->setTarget(robotPos);

                break;
            }
            case actionTypeEnum::MOVE:
            {
                Position2D target(motionSetpoint.position.x, motionSetpoint.position.y, motionSetpoint.position.z);


                std::stringstream str;
                str << "target=" << target.tostr();
                TRACE_SCOPE("PP_TARGET_MOVE", str.str().c_str());


                _ppData->setRobotStop(false);

                _ppData->setTarget(target);

                if (motionSetpoint.slow)
                {
                    _ppData->setMotionProfileType( pp_motionProfile_type::SETPIECE );
                }
                else
                {
                    _ppData->setMotionProfileType( pp_motionProfile_type::NORMAL );
                }

                break;
            }
            default:
            {
                TRACE_SCOPE("PP_TARGET_UNKNOWN", "");
                _ppData->setRobotStop(true);

                // Set target to current position
                Position2D robotPos;
                _ppData->getPosition(robotPos);
                _ppData->setTarget(robotPos);

                break;
            }
        }
    }
}

void cRTDBInputAdapter::getForbiddenAreas()
{
    TRACE_FUNCTION("");
    T_FORBIDDEN_AREAS forbiddenAreas;

    int r = _rtdb->get(FORBIDDEN_AREAS, &forbiddenAreas);

    if (r == RTDB2_SUCCESS)
    {
        std::vector<polygon2D> ppForbiddenAreas;

        T_FORBIDDEN_AREAS::const_iterator itArea;
        for (itArea = forbiddenAreas.begin(); itArea != forbiddenAreas.end(); ++itArea)
        {
            polygon2D poly;
            std::vector<vec2d>::const_iterator itPoly;
            for (itPoly = itArea->points.begin(); itPoly != itArea->points.end(); ++itPoly)
            {
                poly.addPoint( Point2D(itPoly->x, itPoly->y) );
            }

            ppForbiddenAreas.push_back(poly);
        }

        _ppData->setDynamicForbiddenAreas(ppForbiddenAreas);
    }
}

void cRTDBInputAdapter::getWorldModelData()
{
    TRACE_FUNCTION("");

    // poke worldModel to update
    _wmClient->update();

    // store own robot position and velocity
    bool active = _wmClient->isActive();
    auto pos = _wmClient->getPosition();
    auto vel = _wmClient->getVelocity();
    _ppData->setRobotActive(active);
    _ppData->setPosition(pos);
    _ppData->setVelocity(vel);

    // ball
    _ppData->setHaveBall(_wmClient->hasBall());
    auto ballPos = _wmClient->ballPosition();
    _ppData->setBallPos(Position2D(ballPos.x, ballPos.y, 0.0)); // why use Position2D data type? a ball has no orientation, I would expect vec3d
    _ppData->setBallValid(!_wmClient->noBall());

    tprintf("get WORLD_MODEL pos=[%6.2f, %6.2f, %6.2f] vel=[%6.2f, %6.2f, %6.2f]", pos.x, pos.y, pos.phi, vel.x, vel.y, vel.phi);

    // determine if this robot is the lowest active robot
    int lowestActiveRobotId = 100;
    for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
    {
        robotState robot;
        if (_wmClient->getRobotState(robot, agentId))
        {
            if (agentId < lowestActiveRobotId)
            {
                lowestActiveRobotId = agentId;
            }
        }
    }
    if (lowestActiveRobotId == _myRobotId)
    {
        _ppData->setIsLowestActiveRobot(true);
    }
    else
    {
        _ppData->setIsLowestActiveRobot(false);
    }


    fetchObstacles();
    // TODO below: obstacles ... we should get rid of algorithms and diagnostics, does not belong here (see below)
}

void cRTDBInputAdapter::projectObstacles(const pp_obstacle_struct_t& obst, const pp_limiters_struct_t& limits, std::vector<pp_obstacle_struct_t>& obstacles, std::vector<linepoint2D>& projectedSpeedPoint)
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

    // Calculate distance line end point
    Vector2D projectedLine = obst.location.xy() + (projectionDistance / float(ROBOT_RADIUS));
    linepoint2D projectedLinepoint(obst.location.xy().x, obst.location.xy().y, projectedLine.x, projectedLine.y);
    projectedSpeedPoint.push_back(projectedLinepoint);

    // Add obstacle on the line
    projectedObst.location.x = projectedObstPos.x;
    projectedObst.location.y = projectedObstPos.y;
    projectedObst.location.phi = obst.location.phi;
    projectedObst.velocity.x = 0.0;
    projectedObst.velocity.y = 0.0;
    projectedObst.velocity.phi = 0.0;
    projectedObst.type = pp_obstacle_type::PROJECTED;

    obstacles.push_back(projectedObst);

    // Fill the line from obstacle to projectedObstacle with obstacles such that the robot does not decide to cross a teammember's trajectory.
    Vector2D currentProjectionDistance;
    double distBetweenProjectedObstacles = (ROBOT_RADIUS);
    Vector2D currentProjectedObstPos;
    for (size_t j = 1; (currentProjectionDistance.size() + distBetweenProjectedObstacles) < projectionDistance.size(); j++)
    {
        //TRACE("j=%d ; currProjDist=%12.9f ; projDist=%12.9f", j, currentProjectionDistance.size(), projectionDistance.size());
        currentProjectionDistance = obstVel.normalized() * ( j * distBetweenProjectedObstacles );
        currentProjectedObstPos = obstPos + currentProjectionDistance;

        pp_obstacle_struct_t currentProjectedObst;
        currentProjectedObst.location.x = currentProjectedObstPos.x;
        currentProjectedObst.location.y = currentProjectedObstPos.y;
        currentProjectedObst.location.phi = obst.location.phi;
        currentProjectedObst.velocity.x = 0.0;
        currentProjectedObst.velocity.y = 0.0;
        currentProjectedObst.velocity.phi = 0.0;
        currentProjectedObst.type = pp_obstacle_type::PROJECTED;

        obstacles.push_back(currentProjectedObst);
    }
}

void cRTDBInputAdapter::fetchObstacles()
{
    TRACE_FUNCTION("");
    //TRACE(">");

    // Get scalar for taking velocity into account for obstacle avoidance.
    pp_limiters_struct_t limits;
    _ppData->getLimits(limits);

    std::vector<pp_obstacle_struct_t> obstacles;
    std::vector<linepoint2D> projectedSpeedVectors;

    Vector3D ballPos = _wmClient->ballPosition();

    T_OBSTACLES wmObstacles = _wmClient->getObstacles();
    T_OBSTACLES::const_iterator itObst;

    for (itObst = wmObstacles.begin(); itObst != wmObstacles.end(); ++itObst)
    {
        pp_obstacle_struct_t newObst;
        newObst.location.x = itObst->position.x;
        newObst.location.y = itObst->position.y;
        newObst.location.phi = 0.0;
        newObst.velocity.x = itObst->velocity.x;
        newObst.velocity.y = itObst->velocity.y;
        newObst.velocity.phi = 0.0;
        newObst.type = pp_obstacle_type::WORLDMODEL_OBSTACLES;
        obstacles.push_back(newObst);

        // If obstacle is within radius of ball, ignore the velocity vector of the obstacle.
        double currDistToBall = calc_distance(newObst.location, Position2D(ballPos.x, ballPos.y, 0.0));
        if (currDistToBall > 1.0)
        {
            // Based on the velocity of the teammember, project obstacles to avoid its path
            projectObstacles(newObst, limits, obstacles, projectedSpeedVectors);
        }
        bool b;
        _ppData->getHaveBall(b);
        if (b)
        {
            // check if we have a (projected) obstacle closeby
            float closest = 999;
            Position2D robotPos;
            _ppData->getPosition(robotPos);
            for (auto it = obstacles.begin(); it != obstacles.end(); ++it)
            {
                float d = calc_distance(it->location, robotPos);
                if (d < closest) closest = d;
            }
            tprintf("closest (projected) obstacle distance: %.2f", closest);
        }
    }

    std::vector<T_ROBOT_STATE> wmTeamMembers = _wmClient->getTeamMembersExcludingSelf();
    std::vector<T_ROBOT_STATE>::const_iterator itTeam;

    for (itTeam = wmTeamMembers.begin(); itTeam != wmTeamMembers.end(); ++itTeam)
    {
        pp_obstacle_struct_t newObst;
        newObst.location.x = itTeam->position.x;
        newObst.location.y = itTeam->position.y;
        newObst.location.phi = itTeam->position.Rz;
        newObst.velocity.x = itTeam->velocity.x;
        newObst.velocity.y = itTeam->velocity.y;
        newObst.velocity.phi = itTeam->velocity.Rz;
        newObst.type = pp_obstacle_type::WORLDMODEL_TEAMMEMBERS;
        obstacles.push_back(newObst);

        // Based on the velocity of the teammember, project obstacles to avoid its path
        projectObstacles(newObst, limits, obstacles, projectedSpeedVectors);
    }

    _ppData->setProjectedSpeedVectors(projectedSpeedVectors);

    // Remove projected obstacles within 1m of the ball
    std::vector<pp_obstacle_struct_t>::iterator it;
    for (it = obstacles.begin(); it != obstacles.end(); /* do not increase iterator */)
    {
        double currDistToBall = calc_distance(it->location, Position2D(ballPos.x, ballPos.y, 0.0));
        if (it->type == pp_obstacle_type::PROJECTED && currDistToBall <= 1.0)
        {
            it = obstacles.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Update diagnosticsAdapter with latest version of which obstacles pathplanning sees
    std::vector<polygon2D> areas;
    _ppData->getForbiddenAreas(areas);
    _ppData->publishObstacles(areas, projectedSpeedVectors);

    _ppData->setObstacles(obstacles);
    _ppData->setProjectedSpeedVectors(projectedSpeedVectors);

    //TRACE("<");
}
