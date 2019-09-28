 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <vector>
#include <string>
#include <math.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

// Internal:
#include "int/RosGameSignalAdapter.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "falconsRosTopics.h"
#include "position2d.hpp"
#include "linepoint2D.hpp"
#include "polygon2D.hpp"
#include "tracing.hpp"

using namespace falconsRosTopicsInterface;

RosGameSignalAdapter::RosGameSignalAdapter()
{
    _rosNode = new ros::NodeHandle(); 

    // Subscribe to ros topics
    int bufferSize = 5;
    ros::Subscriber sub;

    // Subscribe to team-shared topics
    subscribe<rosMsgs::t_ana_matchstate>(analytics_matchstate, bufferSize, &RosGameSignalAdapter::teamAnalyticsMatchStateCallback);
    subscribe<rosMsgs::t_ana_online>(analytics_online, bufferSize, &RosGameSignalAdapter::teamAnalyticsOnlineCallback);
    subscribe<rosMsgs::t_event>(analytics_event, bufferSize, &RosGameSignalAdapter::teamAnalyticsEventCallback);
    subscribe<rosMsgs::t_worldmodel_team>(team_worldmodel, bufferSize, &RosGameSignalAdapter::teamWorldModelCallback);
    subscribe<rosMsgs::t_worldmodel>(team_worldmodel_old, bufferSize, &RosGameSignalAdapter::teamWorldModelOldCallback);

    // Subscribe to topics per robot
    boost::format format_worldmodel(robot_worldmodel);
    boost::format format_wm_loc(robot_wm_loc);
    boost::format format_wm_ball(robot_wm_ball);
    boost::format format_wm_obst(robot_wm_obst);
    boost::format format_wm_top(robot_wm_top);
    boost::format format_control(robot_control);
    boost::format format_vision(robot_vision);
    boost::format format_vision_v2(robot_vision_v2);
    boost::format format_health_slow(robot_health_slow);
    boost::format format_health_mid(robot_health_mid);
    boost::format format_health_fast(robot_health_fast);
    boost::format format_pathplanning(robot_pathplanning);
    boost::format format_teamplay(robot_teamplay);
    boost::format format_error(robot_error);
    boost::format format_info(robot_info);
    boost::format format_halmw(robot_halmw);
    boost::format format_frontvision(robot_frontvision);
    boost::format format_refbox(robot_refbox);
    boost::format format_ana_comm(robot_ana_comm);
    for (int id = 1; id < _NR_OF_ROBOTS_PER_TEAM + 1; id++)
    {
        // before worldModelV2, we had a worldModel on coach and lumped worldModel diagnostics
        // also, vision diagnostics was shown in visualizer 
        // with worldModelV2 however, vision is reduced and worldModel diagnostics is split
        // for compatibility with old matchlogs, we keep the legacy data handlers for now
        {
            // wmV1: Subscribe world callback per robot
            std::string message_worldmodel = boost::str(format_worldmodel % id);
            subscribe<rosMsgs::t_diag_worldmodel>(id, message_worldmodel, bufferSize, &RosGameSignalAdapter::robotWorldModelCallback);

            // wmV1: Subscribe vision callback per robot
            std::string message_vision = boost::str(format_vision % id);
            subscribe<rosMsgs::t_diag_vision>(id, message_vision, bufferSize, &RosGameSignalAdapter::robotVisionCallback);
        }
        
        {
            // wmV2: Subscribe worldmodel localization diagnostics per robot
            std::string message_wm_loc = boost::str(format_wm_loc % id);
            subscribe<rosMsgs::t_diag_wm_loc>(id, message_wm_loc, bufferSize, &RosGameSignalAdapter::robotWorldModelLocalizationCallback);
            
            // wmV2: Subscribe worldmodel ballTracking diagnostics per robot
            std::string message_wm_ball = boost::str(format_wm_ball % id);
            subscribe<rosMsgs::t_diag_wm_ball>(id, message_wm_ball, bufferSize, &RosGameSignalAdapter::robotWorldModelBallTrackingCallback);

            // wmV2: Subscribe worldmodel obstacles diagnostics per robot
            std::string message_wm_obst = boost::str(format_wm_obst % id);
            subscribe<rosMsgs::t_diag_wm_obstacles>(id, message_wm_obst, bufferSize, &RosGameSignalAdapter::robotWorldModelObstaclesCallback);

            // wm+visionV2: Raw vision data per robot
            std::string message_vision_v2 = boost::str(format_vision_v2 % id);
            subscribe<rosMsgs::t_diag_vision_v2>(id, message_vision_v2, bufferSize, &RosGameSignalAdapter::robotVisionV2Callback);
            
            // wmV2: Subscribe worldmodel top diagnostics per robot
            std::string message_wm_top = boost::str(format_wm_top % id);
            subscribe<rosMsgs::t_diag_wm_top>(id, message_wm_top, bufferSize, &RosGameSignalAdapter::robotWorldModelTopCallback);
        }
        
        // Subscribe control callback per robot
        std::string message_control = boost::str(format_control % id);
        subscribe<rosMsgs::t_diag_control>(id, message_control, bufferSize, &RosGameSignalAdapter::robotControlCallback);

        // Subscribe health slow callback per robot
        std::string message_health_slow = boost::str(format_health_slow % id);
        subscribe<rosMsgs::t_diag_health_slow>(id, message_health_slow, bufferSize, &RosGameSignalAdapter::robotHealthSlowCallback);

        // Subscribe health mid callback per robot
        std::string message_health_mid = boost::str(format_health_mid % id);
        subscribe<rosMsgs::t_diag_health_mid>(id, message_health_mid, bufferSize, &RosGameSignalAdapter::robotHealthMidCallback);

        // Subscribe health fast callback per robot
        std::string message_health_fast = boost::str(format_health_fast % id);
        subscribe<rosMsgs::t_diag_health_fast>(id, message_health_fast, bufferSize, &RosGameSignalAdapter::robotHealthFastCallback);

        // Subscribe pathplanning callback per robot
        std::string message_pathplanning = boost::str(format_pathplanning % id);
        subscribe<rosMsgs::t_diag_pathpl>(id, message_pathplanning, bufferSize, &RosGameSignalAdapter::robotPathPlanningCallback);

        // Subscribe teamplay callback per robot
        std::string message_teamplay = boost::str(format_teamplay % id);
        subscribe<rosMsgs::t_diag_teamplay>(id, message_teamplay, bufferSize, &RosGameSignalAdapter::robotTeamPlayCallback);

        // Subscribe error callback per robot
        std::string message_error = boost::str(format_error % id);
        subscribe<rosMsgs::t_diag_error>(id, message_error, bufferSize, &RosGameSignalAdapter::robotErrorCallback);

        // Subscribe info callback per robot
        std::string message_info = boost::str(format_info % id);
        subscribe<rosMsgs::t_diag_info>(id, message_info, bufferSize, &RosGameSignalAdapter::robotInfoCallback);

        // Subscribe halmw callback per robot
        std::string message_halmw = boost::str(format_halmw % id);
        subscribe<rosMsgs::t_diag_halmw>(id, message_halmw, bufferSize, &RosGameSignalAdapter::robotHalMWCallback);
        
        // Subscribe frontvision callback per robot
        std::string message_frontvision = boost::str(format_frontvision % id);
        subscribe<rosMsgs::t_diag_frontvision>(id, message_frontvision, bufferSize, &RosGameSignalAdapter::robotFrontVisionCallback);

        // Subscribe refbox callback per robot
        std::string message_refbox = boost::str(format_refbox % id);
        subscribe<rosMsgs::t_diag_refbox>(id, message_refbox, bufferSize, &RosGameSignalAdapter::robotRefboxCallback);
        
        // Subscribe refbox callback per robot
        std::string analytics_comm = boost::str(format_ana_comm % id);
        subscribe<rosMsgs::t_ana_comm>(id, analytics_comm, bufferSize, &RosGameSignalAdapter::robotAnalyticsCommCallback);
        
    }

    _rosSpinTimer = new QTimer(this);
    connect(_rosSpinTimer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    _rosSpinTimer->start(10);
}

RosGameSignalAdapter::~RosGameSignalAdapter()
{
    if (_rosNode != NULL)
    {
        delete _rosNode;
        _rosNode = NULL;
    }
    if (_rosSpinTimer != NULL)
    {
        _rosSpinTimer->stop();
        disconnect(_rosSpinTimer, SIGNAL(timeout()), this, SLOT(spinOnce()));
        delete _rosSpinTimer;
        _rosSpinTimer = NULL;
    }
    for (ros::Subscriber sub : _rosSubscribers)
    {
        sub.shutdown();
    }
    _rosSubscribers.clear();
}

void RosGameSignalAdapter::spinOnce()
{
    ros::spinOnce();
}

double RosGameSignalAdapter::getCurrentTimestamp()
{
    return _currentTimestamp;
}

void RosGameSignalAdapter::setCurrentTimestamp(double t)
{
    _currentTimestamp = t;
}

LogEvent RosGameSignalAdapter::convertRosEvent(const rosMsgs::t_event::ConstPtr& rosEvent)
{
    LogEvent result(rosEvent->eventString);
    result.robotId     = rosEvent->robotId;
    result.fileName    = rosEvent->fileName;
    result.funcName    = rosEvent->funcName;
    result.lineNumber  = rosEvent->lineNumber;
    result.timeStamp   = rosEvent->timeStamp;
    result.type        = static_cast<LogLevel>(rosEvent->eventType);
    return result;
}

/* 
* ========================================
*           Coach level
* ======================================== 
*/

void RosGameSignalAdapter::teamWorldModelOldCallback(const rosMsgs::t_worldmodel::ConstPtr& msg)
{
    // legacy wmV1 (2016_Leipzig data and older) 
    uint8_t senderRobotId = 0;
    
    // Ball position
    if (msg->ballpos.size())
    {
        // for now simply draw all
        for (size_t i = 0; i < msg->ballpos.size(); ++i)
        {
            rosMsgs::t_ball ball = msg->ballpos[i];
            PositionVelocity posvel(ball.x, ball.y, ball.z, ball.vx, ball.vy, ball.vz);
            emit signalBallPositionChanged(ObjectId(senderRobotId, i), WORLD, posvel, ball.confidence, 0, OMNIVISION);
        }
    }
    
    // Ball possession
    emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);

    // Iterate over enemies and signal position changes
    for (size_t i = 0; i < msg->enemies.size(); ++i)
    {
        rosMsgs::t_object obstacle = msg->enemies[i];
        PositionVelocity posvel(obstacle.x, obstacle.y, 0, 0, obstacle.vx, obstacle.vy, 0, 0);
        emit signalObstaclePositionChanged(ObjectId(senderRobotId, obstacle.id), WORLD, posvel);
    }

    // Iterate over friends and signal position changes
    for (size_t i = 0; i < msg->friends.size(); ++i)
    {
        rosMsgs::t_object robot = msg->friends[i];
        PositionVelocity posvel(robot.x, robot.y, 0, robot.phi, robot.vx, robot.vy, 0, robot.vphi);
        emit signalOwnTeamPositionChanged(senderRobotId, WORLD, robot.id, posvel); 
    }

    // ownpos is irrelevant/meaningless here.
}

void RosGameSignalAdapter::teamWorldModelCallback(const rosMsgs::t_worldmodel_team::ConstPtr& msg)
{
    TRACE("got new team avg data");
    // Team-average view is associated with robotID==0
    uint8_t senderRobotId = 0;
    
    // Ball position
    for (size_t i = 0; i < msg->balls.size(); ++i)
    {
        rosMsgs::t_ball ball = msg->balls[i];
        PositionVelocity posvel(ball.x, ball.y, ball.z, ball.vx, ball.vy, ball.vz);
        emit signalBallPositionChanged(ObjectId(senderRobotId, ball.id), WORLD, posvel, ball.confidence, 0, OMNIVISION);
    }
    
    // Ball possession
    emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);

    // Iterate over enemies and signal position changes
    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        rosMsgs::t_object obstacle = msg->obstacles[i];
        PositionVelocity posvel(obstacle.x, obstacle.y, 0, 0, obstacle.vx, obstacle.vy, 0, 0);
        emit signalObstaclePositionChanged(ObjectId(senderRobotId, obstacle.id), WORLD, posvel);
    }

    // Iterate over friends and signal position changes
    for (size_t i = 0; i < msg->robots.size(); ++i)
    {
        if (msg->active[i])
        {
            rosMsgs::t_posvel robot = msg->robots[i];
            PositionVelocity posvel(robot.x, robot.y, 0, robot.phi, robot.vx, robot.vy, 0, robot.vphi);
            emit signalOwnTeamPositionChanged(senderRobotId, WORLD, i, posvel); 
        }
    }
}

void RosGameSignalAdapter::teamAnalyticsMatchStateCallback(const rosMsgs::t_ana_matchstate::ConstPtr& msg)
{
    double logElapsedTime = msg->currentTime - msg->startTime;
    TRACE("elapsed=%.3f curr=%.3f", logElapsedTime, msg->currentTime);

    emit signalElapsedTimeChanged(logElapsedTime); // to renderer, for hiding objects
    emit signalClockTick(logElapsedTime, msg->currentTime); // to LCDwidget
    // TODO also emit and show time since start of play (refbox event, match only)
    
    // TODO in case time slider is pulled back, then we can detect it here by comparing logElapsedTime against previous
    // FieldWidget3D does the same inside already, but we should also call EventLogger::clear 
    
    // store current time for age calculations
    setCurrentTimestamp(msg->currentTime - EPOCH);
}

void RosGameSignalAdapter::teamAnalyticsEventCallback(const rosMsgs::t_event::ConstPtr& msg)
{
    double delay = getCurrentTimestamp() - msg->timeStamp;
    TRACE("received event (delay=%.2fs) '%s'", delay, msg->eventString.c_str());
    emit signalLog(convertRosEvent(msg));
}

void RosGameSignalAdapter::teamAnalyticsOnlineCallback(const rosMsgs::t_ana_online::ConstPtr& msg)
{
    const std::string enumState2Str[] = {"offline", "online", "starting", "active", "inplay", "ingame"};
    for (uint8_t robotId = 1; robotId <= MAX_ROBOTS; ++ robotId)
    {
        emit signalValue(robotId, "HEALTH", "state", enumState2Str[msg->state[robotId]]);
        if (enumState2Str[msg->state[robotId]] == "offline")
        {
            TRACE("emit signalClearRobot(%d)", (int)robotId);
            emit signalClearRobot(robotId);
        }
    }
}

/* 
* ========================================
*           Robot view
* ======================================== 
*/

void RosGameSignalAdapter::robotWorldModelCallback(uint8_t senderRobotId, const rosMsgs::t_diag_worldmodel::ConstPtr& msg)
{
    emit signalValue(senderRobotId, "WORLDMODEL", "numberOfBalls", (float)msg->ballpos.size());
    emit signalValue(senderRobotId, "WORLDMODEL", "numberOfFriends", (float)msg->friends.size());
    emit signalValue(senderRobotId, "WORLDMODEL", "numberOfObstacles", (float)msg->enemies.size());
    
    // Ball position
    for (size_t i = 0; i < msg->ballpos.size(); ++i)
    {
        rosMsgs::t_ball ball = msg->ballpos[i];
        PositionVelocity posvel(ball.x, ball.y, ball.z, ball.vx, ball.vy, ball.vz);
        emit signalBallPositionChanged(ObjectId(senderRobotId, 0), WORLD, posvel, ball.confidence, 0, OMNIVISION);
    }
    
    // Ball possession
    emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);

    // Iterate over enemies and signal position changes
    for (size_t i = 0; i < msg->enemies.size(); ++i)
    {
        PositionVelocity posvel(msg->enemies[i].x, msg->enemies[i].y, 0, msg->enemies[i].phi, msg->enemies[i].vx, msg->enemies[i].vy, 0, msg->enemies[i].vphi);
        emit signalObstaclePositionChanged(ObjectId(senderRobotId, 0), WORLD, posvel);
    }

    // Iterate over friends and signal position changes
    for (size_t i = 0; i < msg->friends.size(); ++i)
    {
        PositionVelocity posvel(msg->friends[i].x, msg->friends[i].y, 0, msg->friends[i].phi, msg->friends[i].vx, msg->friends[i].vy, 0, msg->friends[i].vphi);
        emit signalOwnTeamPositionChanged(senderRobotId, WORLD, msg->friends[i].id, posvel); 
    }

    // Signal own position
    PositionVelocity posvel(msg->ownpos.x, msg->ownpos.y, 0, msg->ownpos.phi, msg->ownpos.vx, msg->ownpos.vy, 0);
    emit signalOwnTeamPositionChanged(senderRobotId, WORLD, senderRobotId, posvel); 
}

void RosGameSignalAdapter::robotWorldModelLocalizationCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_loc::ConstPtr& msg)
{

    // Signal own position
    PositionVelocity posvel(msg->ownpos.x, msg->ownpos.y, 0, msg->ownpos.phi, msg->ownpos.vx, msg->ownpos.vy, 0);
    emit signalOwnTeamPositionChanged(senderRobotId, WORLD, senderRobotId, posvel); 
    
    // TODO: if bestVisionCandidate does not match with result, then plot it in red
    
    // Raw values
    // TODO: shouldn't we introduce signalValue for type int?
    emit signalValue(senderRobotId, "WORLDMODEL", "numVisCand", (float)msg->numVisionCandidates);
    emit signalValue(senderRobotId, "WORLDMODEL", "numMotorSamples", (float)msg->numMotorDisplacementSamples);
    emit signalValue(senderRobotId, "WORLDMODEL", "visScore", msg->confidence);
    emit signalValue(senderRobotId, "WORLDMODEL", "visNoiseXY", msg->visionNoiseXY);
    emit signalValue(senderRobotId, "WORLDMODEL", "visNoisePhi", msg->visionNoisePhi);
    emit signalValue(senderRobotId, "WORLDMODEL", "validLoc", (bool) msg->isLocationValid);
}

void RosGameSignalAdapter::robotWorldModelBallTrackingCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_ball::ConstPtr& msg)
{
    emit signalValue(senderRobotId, "WORLDMODEL", "numTrackers", (float)msg->numTrackers);
    emit signalValue(senderRobotId, "WORLDMODEL", "numBalls", (float)msg->balls.size());
    // Ball position
    for (size_t i = 0; i < msg->balls.size(); ++i)
    {
        rosMsgs::t_ball ball = msg->balls[i];
        PositionVelocity posvel(ball.x, ball.y, ball.z, ball.vx, ball.vy, ball.vz);
        emit signalBallPositionChanged(ObjectId(senderRobotId, ball.id), WORLD, posvel, ball.confidence, 0, OMNIVISION);
        // TODO draw confidence details as a label next to actor if requested (menu checkbox "ballTrackingDetails" ?)
    }
    
    // TODO: multiple typically send almost-equal ball results; in visualizer we should draw only one
    // except when balls are not matching (analyzerRealtime could help)
}

void RosGameSignalAdapter::robotWorldModelObstaclesCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_obstacles::ConstPtr& msg)
{
    emit signalValue(senderRobotId, "WORLDMODEL", "numObstacles", (float)msg->numTrackers);
    // Iterate over enemies and signal position changes
    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        rosMsgs::t_obstacle obst = msg->obstacles[i];
        PositionVelocity posvel(obst.x, obst.y, 0, 0, obst.vx, obst.vy, 0, 0);
        emit signalObstaclePositionChanged(ObjectId(senderRobotId, obst.id), WORLD, posvel);
    }
}

void RosGameSignalAdapter::robotWorldModelTopCallback(uint8_t senderRobotId, const rosMsgs::t_diag_wm_top::ConstPtr& msg)
{
    emit signalValue(senderRobotId, "WORLDMODEL", "inplay", (bool)msg->inplay);
    emit signalValue(senderRobotId, "WORLDMODEL", "teamActivity", msg->teamActivity);
    std::string ballPossessionString = "field";
    if (msg->ballPossession.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER)
    {
        ballPossessionString = "r" + boost::lexical_cast<std::string>((int)msg->ballPossession.robotID);
    }
    else if (msg->ballPossession.type == rosMsgs::BallPossession::TYPE_OPPONENT)
    {
        ballPossessionString = "opponent";
    }
    emit signalValue(senderRobotId, "WORLDMODEL", "ballPossession", ballPossessionString);
    emit signalBallPossessionChanged(senderRobotId, WORLD, (BallPossessionType)msg->ballPossession.type, msg->ballPossession.robotID);
    emit signalValue(senderRobotId, "WORLDMODEL", "visionClaimed", (bool)msg->ballPossessionVision);
    emit signalValue(senderRobotId, "WORLDMODEL", "BhClaimed", (bool)msg->ballPossessionBallHandlers);
}

void RosGameSignalAdapter::robotControlCallback(uint8_t robotId, const rosMsgs::t_diag_control::ConstPtr& msg)
{
    // events now handled directly via teamAnalyticsEventCallback 
    if (!msg->claimedby.empty())
    {
        std::string claimedby = boost::str(boost::format("Robot control claimer hostname: %1%") % msg->claimedby);
        emit signalValue(robotId, "CONTROL", "claimedby", msg->claimedby.c_str());
    }
}

void RosGameSignalAdapter::robotVisionCallback(uint8_t robotId, const rosMsgs::t_diag_vision::ConstPtr& msg)
{
    emit signalValue(robotId, "VISION", "fps", msg->fps);
    emit signalValue(robotId, "VISION", "linePoints", (float)msg->linePoints);
    emit signalValue(robotId, "VISION", "numBalls", (float)msg->numBalls);
    emit signalValue(robotId, "VISION", "numObstacles", (float)msg->numObstacles);
    
    // legacy (pre-worldModelV2)
    // TODO emit these but only if data if applicable (old bag) - how to determine nicely?
    
    // no use case anymore -- remove
}

void RosGameSignalAdapter::robotVisionV2Callback(uint8_t robotId, const rosMsgs::t_diag_vision_v2::ConstPtr& msg)
{
    // Iterate over ball positions and signal
    double tNow = getCurrentTimestamp();
    for (size_t i = 0; i < msg->balls.size(); ++i)
    {
        PositionVelocity posvel(msg->balls[i].x, msg->balls[i].y, msg->balls[i].z);
        TRACE("tNow=%.3f ball.timestamp=%.3f", tNow, msg->balls[i].timestamp);
        float age = tNow - msg->balls[i].timestamp;
        CameraType camType = (CameraType)(int)msg->balls[i].cameraType.camera_type;
        emit signalBallPositionChanged(ObjectId(robotId, msg->balls[i].objectID), VISION, posvel, 1.0, age, camType);
    }

    // Iterate over obstacles and signal 
    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        PositionVelocity posvel(msg->obstacles[i].x, msg->obstacles[i].y);
        emit signalObstaclePositionChanged(ObjectId(robotId, msg->obstacles[i].objectID), VISION, posvel);
    }
}

void RosGameSignalAdapter::robotHealthSlowCallback(uint8_t robotId, const rosMsgs::t_diag_health_slow::ConstPtr& msg)
{
    try
    {
        float cpuFreq = std::stof(msg->cpuFreq);
        emit signalValue(robotId, "HEALTH", "cpuFreq", (float)cpuFreq);
    }
    catch (std::out_of_range)
    {
        emit signalValue(robotId, "HEALTH", "cpuFreq", (float)-1);
    }
    catch (std::invalid_argument)
    {
        emit signalValue(robotId, "HEALTH", "cpuFreq", (float)-1);
    }

    emit signalValue(robotId, "HEALTH", "diskUsage", (float)msg->diskUsage);

    if (!msg->gitBranch.empty())
    {
        emit signalValue(robotId, "HEALTH", "gitBranch", msg->gitBranch);
    }

    bool gitDirty = msg->gitDirty.compare("dirty") == 0;
    emit signalValue(robotId, "HEALTH", "gitDirty", gitDirty);

    if (!msg->globalConfig.empty())
    {
        emit signalValue(robotId, "HEALTH", "globalConfig", msg->globalConfig);
    }
}

void RosGameSignalAdapter::robotHealthMidCallback(uint8_t robotId, const rosMsgs::t_diag_health_mid::ConstPtr& msg)
{
    std::vector<std::string> deadProcesses;
    boost::split(deadProcesses, msg->deadProcesses, boost::is_any_of(","));
    if (deadProcesses.size() > 0)
    {
        emit signalValue(robotId, "HEALTH", "deadProcesses", deadProcesses); 
    }

    if (!msg->badOutput.empty())
    {
        emit signalValue(robotId, "HEALTH", "badoutput", msg->badOutput);
    }

    if (!msg->accessPoint.empty())
    {
        emit signalValue(robotId, "HEALTH", "accessPoint", msg->accessPoint);
    }

    if (!msg->ipAddress.empty())
    {
        emit signalValue(robotId, "HEALTH", "ipAddress", msg->ipAddress);
    }

    if (!msg->badPeripherals.empty())
    {
        emit signalValue(robotId, "HEALTH", "badPeripherals", msg->badPeripherals); 
    }
}

void RosGameSignalAdapter::robotHealthFastCallback(uint8_t robotId, const rosMsgs::t_diag_health_fast::ConstPtr& msg)
{
    emit signalValue(robotId, "HEALTH", "networkLoad", msg->networkLoad); // kb/sec
    emit signalValue(robotId, "HEALTH", "cpuLoad", msg->cpuLoad);
}

void RosGameSignalAdapter::robotPathPlanningCallback(uint8_t robotId, const rosMsgs::t_diag_pathpl::ConstPtr& msg)
{
    if (msg->active)
    {
        std::vector<PositionVelocity> path;
        if (!std::isnan(msg->subtarget.x) && !std::isnan(msg->subtarget.y))
        {
            path.push_back(PositionVelocity(msg->subtarget.x, msg->subtarget.y, 0, msg->subtarget.phi, msg->subtarget.vx, msg->subtarget.vy));
        }
        path.push_back(PositionVelocity(msg->target.x, msg->target.y, 0, msg->target.phi, msg->target.vx, msg->target.vy));
        emit signalPathPlanningInProgress(robotId, path);


        for (size_t i = 0; i < msg->forbiddenAreas.size(); i++)
        {
            polygon2D forbiddenArea;
            for(auto it = msg->forbiddenAreas.at(i).points.begin(); it != msg->forbiddenAreas.at(i).points.end(); it++)
            {
                forbiddenArea.addPoint(it->x, it->y);
            }

            emit signalForbiddenAreaChanged(ObjectId(robotId, msg->forbiddenAreas.at(i).id), PATHPLANNING, forbiddenArea);
        }

        for (auto it = msg->speedArrowEndPoints.begin(); it != msg->speedArrowEndPoints.end(); it++)
        {

            linepoint2D speedVector(it->src.x, it->src.y, it->dst.x, it->dst.y);
            //linepoint2D speedVector(1.0, 2.0, -3.0, -4.0);

            emit signalProjectSpeedChanged(ObjectId(robotId, it->id), PATHPLANNING, speedVector);
        }
    }
}

void RosGameSignalAdapter::robotTeamPlayCallback(uint8_t robotId, const rosMsgs::t_diag_teamplay::ConstPtr& msg)
{
    // Process new teamplay data, which is a list of trees (names)
    // Discussed with Erik: trees is guaranteed to contain as first element the gameplay, second element the role, followed by one or more behaviors and lastly the action. 
    emit signalValue(robotId, "TEAMPLAY", "gamestate", (msg->state.empty()) ? "---" : msg->state);
    emit signalValue(robotId, "TEAMPLAY", "role", (msg->role.empty()) ? "---" : msg->role);
    emit signalValue(robotId, "TEAMPLAY", "behavior-1", (msg->behavior.empty()) ? "---" : msg->behavior);
    emit signalValue(robotId, "TEAMPLAY", "action", (msg->action.empty()) ? "---" : msg->action);

    // Process aiming and shooting data
    PositionVelocity posvel(msg->shootTargetX, msg->shootTargetY);
    emit signalShootTargetChanged(robotId, TEAMPLAY, posvel, msg->aiming);
}

void RosGameSignalAdapter::robotErrorCallback(uint8_t robotId, const rosMsgs::t_diag_error::ConstPtr& msg)
{
    LogEvent e(msg->error);
    e.robotId = robotId;
    e.type = ERROR;
    emit signalLog(e);
}

void RosGameSignalAdapter::robotInfoCallback(uint8_t robotId, const rosMsgs::t_diag_info::ConstPtr& msg)
{
    LogEvent e(msg->info);
    e.robotId = robotId;
    e.type = INFO;
    emit signalLog(e);
}

void RosGameSignalAdapter::robotHalMWCallback(uint8_t robotId, const rosMsgs::t_diag_halmw::ConstPtr& msg)
{
    emit signalValue(robotId, "HALMW", "feedback_m1_vel", msg->feedback_m1_vel);
    emit signalValue(robotId, "HALMW", "feedback_m2_vel", msg->feedback_m2_vel);
    emit signalValue(robotId, "HALMW", "feedback_m3_vel", msg->feedback_m3_vel);
    emit signalValue(robotId, "HALMW", "speed_m1_vel", msg->speed_m1_vel);
    emit signalValue(robotId, "HALMW", "speed_m2_vel", msg->speed_m2_vel);
    emit signalValue(robotId, "HALMW", "speed_m3_vel", msg->speed_m3_vel);
    emit signalValue(robotId, "HALMW", "has ball", (bool)msg->hasball);
    emit signalValue(robotId, "HALMW", "bh_left_angle", msg->bh_left_angle);
    emit signalValue(robotId, "HALMW", "bh_right_angle", msg->bh_right_angle);
    emit signalValue(robotId, "HALMW", "voltage", msg->voltage);
    emit signalValue(robotId, "HALMW", "temp_rear", msg->motion_rear_temperature);
    emit signalValue(robotId, "HALMW", "temp_right", msg->motion_right_temperature);
    emit signalValue(robotId, "HALMW", "temp_left", msg->motion_left_temperature);
}

void RosGameSignalAdapter::robotFrontVisionCallback(uint8_t robotId, const rosMsgs::t_diag_frontvision::ConstPtr& msg)
{
    emit signalValue(robotId, "FRONTVISION", "fps", msg->fps);

    // Iterate over ball positions and signal
    for (size_t i = 0; i < msg->balls.size(); ++i)
    {
        Position2D position;
        position.x = cos(msg->balls[i].angle) * msg->balls[i].radius;
        position.y = sin(msg->balls[i].angle) * msg->balls[i].radius;

        // TODO: Need ownpos in t_diag_frontvision, like in t_diag_vision in order to determine ball front vision ball position
        // position.transform_rcs2fcs(Position2D(msg->ownpos.x, msg->ownpos.y, msg->ownpos.phi));
    }
}

void RosGameSignalAdapter::robotRefboxCallback(uint8_t robotId, const rosMsgs::t_diag_refbox::ConstPtr& msg)
{
    if (!msg->refboxCmd.empty())
    {
        emit signalValue(robotId, "REFBOX", "refboxCmd", msg->refboxCmd);
    }
}

/* 
* ========================================
*           Robot analytics
* ======================================== 
*/

void RosGameSignalAdapter::robotAnalyticsCommCallback(uint8_t robotId, const rosMsgs::t_ana_comm::ConstPtr& msg)
{
    emit signalValue(robotId, "HEALTH", "latency", boost::str(boost::format("%dms") % int(1000.0 * msg->latency))); 
    emit signalValue(robotId, "HEALTH", "eventFreq", boost::str(boost::format("%.1fHz") % (msg->frequency))); 
    emit signalValue(robotId, "HEALTH", "packetLoss", boost::str(boost::format("%.1f%%") % (msg->packetLoss * 100.0))); 
}


