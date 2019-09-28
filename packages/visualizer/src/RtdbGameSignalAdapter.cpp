 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RtdbGameSignalAdapter.cpp
 *
 *  Created on: Aug 13, 2018
 *      Author: Jan Feitsma
 */

#include "int/RtdbGameSignalAdapter.h"
#include "tracing.hpp"
#include "linalgcv.hpp" // from package geometry, for objectCandidateToFcs()
#include "ctime"
#include "chrono"
#include "int/types/RobotColor.h"

double lastEvents [7][2] = {{0,0},{0,0}, {0,0},{0,0},{0,0}, {0,0}, {0,0}};

RtdbGameSignalAdapter::RtdbGameSignalAdapter()
{
    TRACE(">");
    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    _timer->start(int(1000 * 1.0 / 30)); // number in milliseconds
    _startTimestamp = rtime::now();
    TRACE("<");
}

RtdbGameSignalAdapter::~RtdbGameSignalAdapter()
{
}

void RtdbGameSignalAdapter::emitMatchState(int agentId)
{
    int r = 0;
    auto db = _rtdb.at(agentId);
    T_MATCH_STATE s;

    r = db->get(MATCH_STATE, &s, _matchAge, agentId);

    if (r == RTDB2_SUCCESS)
    {
        _currentTimestamp = s.currentTime; // store for internal use (e.g. robot status timeout)
        double logElapsedTime = double(s.currentTime - _startTimestamp);
        emit signalElapsedTimeChanged(logElapsedTime); // to renderer, for hiding objects
        emit signalClockTick(logElapsedTime, double(s.currentTime)); // to LCDwidget
        emit signalGoal(agentId, s.goalsOwn);
        emit signalPhase(agentId, (int)s.phase);
        emit signalRefBoxCommand(agentId, s.lastRefboxCommand);
        if (s.lastRefboxCommandTime.toDouble()!=0)
        {
            emit signalRefBoxCommandTime(agentId, s.lastRefboxCommandTime.toDouble());
        }
    }
}

robotStatusEnum RtdbGameSignalAdapter::emitRobotState(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_ROBOT_STATE robot;
    r = db->get(ROBOT_STATE, &robot, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        if (robot.status != robotStatusEnum::OUTOFPLAY)
        {
            PositionVelocity posvel(robot.position.x, robot.position.y, 0, robot.position.Rz, robot.velocity.x, robot.velocity.y, 0, robot.velocity.Rz);
            emit signalOwnTeamPositionChanged(agentId, WORLD, agentId, posvel);
            // determine color based on age and inplay state
            int status = RobotColor::BLUE;
            double robotTimestamp = robot.timestamp;
            double age = robotTimestamp - double(_currentTimestamp);

            if (age > 7.0)
            {
                status = RobotColor::RED;
            }
            if (robot.status != robotStatusEnum::INPLAY)
            {
                status = RobotColor::RED;
            }

            emit signalRobotStatusChanged(agentId, WORLD, agentId, status);

        }
        return robot.status;
    }
    else
    {
        return robotStatusEnum::UNKNOWN;
    }
}

void RtdbGameSignalAdapter::emitBallResults(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_BALLS balls;
    r = db->get(BALLS, &balls, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        for (size_t b = 0; b < balls.size(); ++b)
        {
            auto ball = balls[b];
            PositionVelocity posvel(ball.position.x, ball.position.y, ball.position.z, 0, ball.velocity.x, ball.velocity.y, ball.velocity.z, 0);
            emit signalBallPositionChanged(ObjectId(agentId, b), WORLD, posvel, ball.confidence, 0, OMNIVISION);
        }
    }
}

void RtdbGameSignalAdapter::emitObstacleResults(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_OBSTACLES obstacles;
    r = db->get(OBSTACLES, &obstacles, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        for (size_t n = 0; n < obstacles.size(); ++n)
        {
            auto obstacle = obstacles[n];
            PositionVelocity posvel(obstacle.position.x, obstacle.position.y, 0, 0, obstacle.velocity.x, obstacle.velocity.y, 0, 0);
            emit signalObstaclePositionChanged(ObjectId(agentId, n), WORLD, posvel);
        }
    }
}

void RtdbGameSignalAdapter::emitGaussianObstacleResults(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_DIAG_WORLDMODEL_LOCAL worldmodel_local;
    r = db->get(DIAG_WORLDMODEL_LOCAL, &worldmodel_local, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        emit signalGaussianObstaclesUpdate(agentId, GAUSSIAN_WORLD, worldmodel_local);
    }
}

template<typename T>
Vector3D objectCandidateToFcs(T const &obj)
{
    // function is provided by geometry, linalgcv.hpp
    return object2fcs(obj.cameraX, obj.cameraY, obj.cameraZ, obj.cameraPhi, obj.azimuth, obj.elevation, obj.radius);
}


void RtdbGameSignalAdapter::emitBallCandidates(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_BALL_CANDIDATES_FCS balls;
    r = db->get(BALL_CANDIDATES_FCS, &balls, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        for (size_t b = 0; b < balls.size(); ++b)
        {
            auto ball = balls.at(b);
            auto posFcs = objectCandidateToFcs(ball);
            PositionVelocity posvel(posFcs.x, posFcs.y, posFcs.z, 0, 0, 0, 0, 0);
            emit signalBallPositionChanged(ObjectId(agentId, b), VISION, posvel, ball.confidence, 0, OMNIVISION);
        }
    }
}

void RtdbGameSignalAdapter::emitObstacleCandidates(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_OBSTACLE_CANDIDATES_FCS obstacles;
    r = db->get(OBSTACLE_CANDIDATES_FCS, &obstacles, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        for (size_t n = 0; n < obstacles.size(); ++n)
        {
            auto obstacle = obstacles.at(n);
            auto posFcs = objectCandidateToFcs(obstacle);
            PositionVelocity posvel(posFcs.x, posFcs.y, posFcs.z, 0, 0, 0, 0, 0);
            emit signalObstaclePositionChanged(ObjectId(agentId, n), VISION, posvel);
        }
    }
}

void RtdbGameSignalAdapter::emitEvents(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    bool eventListNew = true;
    T_EVENT_LIST events;
    r = db->get(EVENT_LIST, &events, life, agentId);
    if ((r == RTDB2_SUCCESS) && (events.size()!=0))
    {
        if ((events.size()==lastEvents[agentId-1][0]) && (events.at(0).timeStamp.toDouble()==lastEvents[agentId-1][1]))
        {
            eventListNew = false;
        }

        for (size_t n = 0; n < events.size(); ++n)
        {
            auto event = events.at(n);
            LogEvent e(event.message);
            e.robotId = agentId;
            e.fileName = event.fileName;
            e.funcName = event.funcName;
            e.lineNumber = event.lineNumber;
            e.timeStamp = event.timeStamp.toDouble();
            e.type = LogLevel(event.severity);

            double event_time_elapsed = double(_currentTimestamp) - e.timeStamp;

            if ((e.type != 0) && ( (life < 3000) || (event_time_elapsed < 0.1 && event_time_elapsed >=0 )))
            {
                //color when in warning or error for 3 seconds
                emit signalRobotStatusChanged(agentId, WORLD, agentId, e.type);
            }

            //log new events
            if (eventListNew)
            {
                emit signalLog(e);
            }
        }

        if (eventListNew)
        {
            lastEvents[agentId-1][0] = events.size();
            lastEvents[agentId-1][1] = events.at(0).timeStamp.toDouble();
        }
    }
}

void RtdbGameSignalAdapter::emitWorldModelData(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_DIAG_WORLDMODEL_SHARED diag_worldmodel;
    r = db->get(DIAG_WORLDMODEL_SHARED, &diag_worldmodel, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        emit signalValue(agentId, "PRIMARY_INFO", "validLoc", (bool) diag_worldmodel.locationValid);
        emit signalValue(agentId, "PRIMARY_INFO", "inplay", (bool)diag_worldmodel.inplay);
        emit signalValue(agentId, "PRIMARY_INFO", "teamActivity", diag_worldmodel.teamActivity);
        emit signalValue(agentId, "PRIMARY_INFO", "visionClaimed", (bool)diag_worldmodel.ballPossessionVision);
        emit signalValue(agentId, "PRIMARY_INFO", "BhClaimed", (bool)diag_worldmodel.ballPossessionBallHandlers);
        emit signalValue(agentId, "WORLDMODEL", "numVisCand", (float)diag_worldmodel.numVisionCandidates);
        emit signalValue(agentId, "WORLDMODEL", "numMotorSamples",(float)diag_worldmodel.numMotorDisplacementSamples);
        emit signalValue(agentId, "WORLDMODEL", "visScore", diag_worldmodel.visionConfidence);
        emit signalValue(agentId, "WORLDMODEL", "visNoiseXY", diag_worldmodel.visionNoiseXY);
        emit signalValue(agentId, "WORLDMODEL", "visNoisePhi", diag_worldmodel.visionNoisePhi);
        emit signalValue(agentId, "WORLDMODEL", "validLoc", (bool) diag_worldmodel.locationValid);
        emit signalValue(agentId, "WORLDMODEL", "numBallTrackers", (float)diag_worldmodel.numBallTrackers);
        emit signalValue(agentId, "WORLDMODEL", "numObstacleTrackers", (float)diag_worldmodel.numObstacleTrackers);
        emit signalValue(agentId, "WORLDMODEL", "inplay", (bool)diag_worldmodel.inplay);
        emit signalValue(agentId, "WORLDMODEL", "teamActivity", diag_worldmodel.teamActivity);
        emit signalValue(agentId, "WORLDMODEL", "visionClaimed", (bool)diag_worldmodel.ballPossessionVision);
        emit signalValue(agentId, "WORLDMODEL", "BhClaimed", (bool)diag_worldmodel.ballPossessionBallHandlers);
        emit signalValue(agentId, "WORLDMODEL", "duration", diag_worldmodel.duration);
        emit signalValue(agentId, "WORLDMODEL", "bestTracker", (float)diag_worldmodel.bestTrackerId);
        emit signalValue(agentId, "WORLDMODEL", "bestVisCand_x", diag_worldmodel.bestVisionCandidate.x);
        emit signalValue(agentId, "WORLDMODEL", "bestVisCand_y", diag_worldmodel.bestVisionCandidate.y);
        emit signalValue(agentId, "WORLDMODEL", "bestVisCand_Rz", diag_worldmodel.bestVisionCandidate.Rz);
        emit signalValue(agentId, "WORLDMODEL", "ownBallsFirst", (bool)diag_worldmodel.ownBallsFirst);
    }
}

void RtdbGameSignalAdapter::emitHealthData(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_DIAG_HEALTH_FAST diag_health_fast;
    r = db->get(DIAG_HEALTH_FAST, &diag_health_fast, life, agentId);
    if (r == RTDB2_SUCCESS)
    {
        emit signalValue(agentId, "HEALTH", "networkLoad", diag_health_fast.networkLoad);
        emit signalValue(agentId, "HEALTH", "cpuLoad", diag_health_fast.cpuLoad);
    }

    T_DIAG_HEALTH_SLOW diag_health_slow;
    r = db->get(DIAG_HEALTH_SLOW, &diag_health_slow, life, agentId);

    if (r == RTDB2_SUCCESS)
    {
        emit signalValue(agentId, "HEALTH", "diskUsage", (float)diag_health_slow.diskUsage);
        emit signalValue(agentId, "PRIMARY_INFO", "diskUsage", (float)diag_health_slow.diskUsage);
    }

}

void RtdbGameSignalAdapter::emitTeamPlayData(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_DIAG_TEAMPLAY diag_teamplay;
    r = db->get(DIAG_TEAMPLAY, &diag_teamplay, life, agentId);

    if (r == RTDB2_SUCCESS)
    {
        emit signalValue(agentId, "TEAMPLAY", "gamestate", (diag_teamplay.state.empty()) ? "---" : diag_teamplay.state);
        emit signalValue(agentId, "TEAMPLAY", "role", (diag_teamplay.role.empty()) ? "---" : diag_teamplay.role);
        emit signalValue(agentId, "TEAMPLAY", "behavior", (diag_teamplay.behavior.empty()) ? "---" : diag_teamplay.behavior);
        emit signalValue(agentId, "TEAMPLAY", "action", (diag_teamplay.action.empty()) ? "---" : diag_teamplay.action);
        emit signalValue(agentId, "PRIMARY_INFO", "gamestate", (diag_teamplay.state.empty()) ? "---" : diag_teamplay.state);
        emit signalValue(agentId, "PRIMARY_INFO", "role", (diag_teamplay.role.empty()) ? "---" : diag_teamplay.role);
        emit signalValue(agentId, "PRIMARY_INFO", "behavior", (diag_teamplay.behavior.empty()) ? "---" : diag_teamplay.behavior);
        emit signalValue(agentId, "PRIMARY_INFO", "action", (diag_teamplay.action.empty()) ? "---" : diag_teamplay.action);
        emit signalValue(agentId, "TEAMPLAY", "aiming", (bool)diag_teamplay.aiming);
        emit signalValue(agentId, "TEAMPLAY", "shootTargetX", (bool)diag_teamplay.shootTargetX);
        emit signalValue(agentId, "TEAMPLAY", "shootTargetY", (bool)diag_teamplay.shootTargetY);
    }

    T_MATCH_STATE match_state;
    r = db->get(MATCH_STATE, &match_state, life, agentId);

    if (r == RTDB2_SUCCESS)
    {
        emit signalValue(agentId, "TEAMPLAY", "refboxCmd", match_state.lastRefboxCommand);
    }
}

void RtdbGameSignalAdapter::emitHalmwData(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_DIAG_PERIPHERALSINTERFACE diag_peripheralsinterface;
    r = db->get(DIAG_PERIPHERALSINTERFACE, &diag_peripheralsinterface, life, agentId);

    if (r == RTDB2_SUCCESS)
    {
        emit signalValue(agentId, "HALMW", "feedback_m1_vel", diag_peripheralsinterface.feedback_vel[0]);
        emit signalValue(agentId, "HALMW", "feedback_m2_vel", diag_peripheralsinterface.feedback_vel[1]);
        emit signalValue(agentId, "HALMW", "feedback_m3_vel", diag_peripheralsinterface.feedback_vel[2]);
        emit signalValue(agentId, "HALMW", "speed_m1_vel", diag_peripheralsinterface.speed_vel[0]);
        emit signalValue(agentId, "HALMW", "speed_m2_vel", diag_peripheralsinterface.speed_vel[1]);
        emit signalValue(agentId, "HALMW", "speed_m3_vel", diag_peripheralsinterface.speed_vel[2]);
        emit signalValue(agentId, "HALMW", "has ball", (bool)diag_peripheralsinterface.hasball);
        emit signalValue(agentId, "PRIMARY_INFO", "has ball", (bool)diag_peripheralsinterface.hasball);
        emit signalValue(agentId, "HALMW", "bh_left_angle", diag_peripheralsinterface.bh_left_angle);
        emit signalValue(agentId, "HALMW", "bh_right_angle", diag_peripheralsinterface.bh_right_angle);
        emit signalValue(agentId, "HALMW", "voltage", diag_peripheralsinterface.voltage);
        emit signalValue(agentId, "PRIMARY_INFO", "voltage", diag_peripheralsinterface.voltage);
        emit signalValue(agentId, "HALMW", "temp_rear", diag_peripheralsinterface.motion_temperature[1]);
        emit signalValue(agentId, "HALMW", "temp_right", diag_peripheralsinterface.motion_temperature[0]);
        emit signalValue(agentId, "HALMW", "temp_left", diag_peripheralsinterface.motion_temperature[2]);
    }
}

void RtdbGameSignalAdapter::emitPathPlanningDiagnostics(int agentId)
{
    int r = 0;
    int life = 0;
    auto db = _rtdb.at(agentId);
    T_DIAG_PATHPLANNING diag_pp;
    r = db->get(DIAG_PATHPLANNING, &diag_pp, life, agentId);

    if (r == RTDB2_SUCCESS)
    {
        std::vector<PositionVelocity> path;
        for (auto it = diag_pp.path.begin(); it != diag_pp.path.end(); ++it)
        {
            path.push_back(PositionVelocity(it->pos.x, it->pos.y, 0, it->pos.Rz, it->vel.x, it->vel.y));
        }
        emit signalPathPlanningInProgress(agentId, path);

        for (size_t i = 0; i < diag_pp.forbiddenAreas.size(); i++)
        {
            polygon2D forbiddenArea;
            for(auto it = diag_pp.forbiddenAreas.at(i).points.begin(); it != diag_pp.forbiddenAreas.at(i).points.end(); it++)
            {
                forbiddenArea.addPoint(it->x, it->y);
            }

            emit signalForbiddenAreaChanged(ObjectId(agentId, diag_pp.forbiddenAreas.at(i).id), PATHPLANNING, forbiddenArea);
        }
    }
}

void RtdbGameSignalAdapter::emitOutofPlay(int agentId)
{
    // log out-of-play on battery widget

    emit signalOutOfPlay(agentId, true);
}

void RtdbGameSignalAdapter::spinOnce()
{
    TRACE_FUNCTION("");

    // clear current state
    // TODO: this is only needed when we scroll back in time, so for performance reasons we then could use actor visibility instead
    emit signalClearAll();
    
    // coach data
    {
        int agentId = 0;
        
        // coach match state
        emitMatchState(agentId);

        // worldModel ball result
        emitBallResults(agentId);

        // worldModel obstacle result
        emitObstacleResults(agentId);

        // worldModel local data update
        emitGaussianObstacleResults(agentId);
                
        // vision ball candidates
        emitBallCandidates(agentId);

        // vision obstacle candidates
        emitObstacleCandidates(agentId);

    }
    // per-robot data
    for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
    {
        int agentId = *it;

        if (agentId != 0)
        {
            // worldModel robot state
            robotStatusEnum robotState = emitRobotState(agentId);

            if(robotState!=robotStatusEnum::OUTOFPLAY)
            {

                // worldModel ball result
                emitBallResults(agentId);

                // worldModel obstacle result
                emitObstacleResults(agentId);

                // worldModel local data update
                emitGaussianObstacleResults(agentId);

                // vision ball candidates
                emitBallCandidates(agentId);

                // vision obstacle candidates
                emitObstacleCandidates(agentId);

                // pathPlanning diagnostics
                emitPathPlanningDiagnostics(agentId);

                //if match state age gets old, may be paused or stopped. Therefore stop emitting signals.

                // worldModel results
                emitWorldModelData(agentId);

                //health situation
                emitHealthData(agentId);

                //team play data
                emitTeamPlayData(agentId);

                //peripheral interface data
                emitHalmwData(agentId);
            }
            else
            {
                emitOutofPlay(agentId);
            }

            // event
            emitEvents(agentId);
        }
    }
}

/*
// TODO: below were all emits from RosGameSignalAdapter.cpp
emit signalBallPositionChanged(ObjectId(senderRobotId, i), WORLD, posvel, ball.confidence, 0, OMNIVISION);
emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);
emit signalObstaclePositionChanged(ObjectId(senderRobotId, obstacle.id), WORLD, posvel);
emit signalOwnTeamPositionChanged(senderRobotId, WORLD, robot.id, posvel); 
emit signalBallPositionChanged(ObjectId(senderRobotId, ball.id), WORLD, posvel, ball.confidence, 0, OMNIVISION);
emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);
emit signalObstaclePositionChanged(ObjectId(senderRobotId, obstacle.id), WORLD, posvel);
emit signalOwnTeamPositionChanged(senderRobotId, WORLD, i, posvel); 
emit signalElapsedTimeChanged(logElapsedTime); // to renderer, for hiding objects
emit signalClockTick(logElapsedTime, msg->currentTime); // to LCDwidget
emit signalLog(convertRosEvent(msg));

emit signalClearRobot(robotId);

emit signalBallPositionChanged(ObjectId(senderRobotId, 0), WORLD, posvel, ball.confidence, 0, OMNIVISION);
emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);
emit signalObstaclePositionChanged(ObjectId(senderRobotId, 0), WORLD, posvel);
emit signalOwnTeamPositionChanged(senderRobotId, WORLD, msg->friends[i].id, posvel); 
emit signalOwnTeamPositionChanged(senderRobotId, WORLD, senderRobotId, posvel); 
emit signalOwnTeamPositionChanged(senderRobotId, WORLD, senderRobotId, posvel); 

emit signalBallPositionChanged(ObjectId(senderRobotId, ball.id), WORLD, posvel, ball.confidence, 0, OMNIVISION);

emit signalObstaclePositionChanged(ObjectId(senderRobotId, obst.id), WORLD, posvel);

emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);

emit signalBallPositionChanged(ObjectId(robotId, msg->balls[i].objectID), VISION, posvel, 1.0, age, camType);
emit signalObstaclePositionChanged(ObjectId(robotId, msg->obstacles[i].objectID), VISION, posvel);

emit signalPathPlanningInProgress(robotId, path);
emit signalForbiddenAreaChanged(ObjectId(robotId, msg->forbiddenAreas.at(i).id), PATHPLANNING, forbiddenArea);
emit signalProjectSpeedChanged(ObjectId(robotId, it->id), PATHPLANNING, speedVector);

emit signalShootTargetChanged(robotId, TEAMPLAY, posvel, msg->aiming);

emit signalValue(robotId, "COMPASS", "orientation", msg->theta);    -- No
emit signalValue(robotId, "FRONTVISION", "fps", msg->fps);          -- No
emit signalValue(robotId, "CONTROL", "claimedby", msg->claimedby.c_str());  -- Not needed

*/

