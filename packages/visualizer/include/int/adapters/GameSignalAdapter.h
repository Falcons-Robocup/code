// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef GAMESIGNALADAPTER_H
#define GAMESIGNALADAPTER_H

#include <QtGui>

#include "int/types/PositionVelocity.h"
#include "int/types/LogEvent.h"
#include "int/types/ObjectId.h"

#include "polygon2D.hpp"
#include "rtdbStructs.hpp"

enum BallPossessionType : uint8_t
{
    TYPE_INVALID = 0,
    TYPE_FIELD = 1,
    TYPE_TEAMMEMBER = 2,
    TYPE_OPPONENT = 3
};

enum SignalMode
{
    WORLD,
    VISION,
    PATHPLANNING,
    TEAMPLAY,
    GAUSSIAN_WORLD,
    BALL_MEASUREMENTS,
    OBSTABLE_MEASUREMENTS,
    TRUE_BALL
};

enum DataType
{
    UNDEFINED,
    ROBOTPOSITION,
    BALLPOSITION,
    BALLPOSSESSION,
    OBSTACLEPOSITION,
    FORBIDDENAREA,
    SHOOTTARGET,
    PROJECTSPEEDVECTOR,
    PATHPLANNINGPROGRESS,
    WORLDMODEL_LOCAL
};

enum CameraType
{
    OMNIVISION,
    FRONTVISION,
    MULTIVISION,
};

/*
* Pure virtual interface for signals
* i.e. robot motion, world updates, etc. 
* DKPJ: Note, do not expose any Falcons-specific types anywhere but in the Adapter implementation.
* Goal is to have visualizer as complete standalone application that other teams merely need to write a new adapter for.
*/
class GameSignalAdapter : public QObject
{
    Q_OBJECT

Q_SIGNALS:
    /* == Robot view  == */
    void signalBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence, float age, CameraType camera); // Ball position according to one robot, also used for vision measurements
    void signalBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId); // Ball possession according to one robot
    void signalOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel); // Team member position according to one robot
    void signalRobotStatusChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, int status); // indicator for robot status
    void signalRobotRoleChanged(uint8_t robotId, std::string role);
    void signalObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel); // Obstacle position according to one robot
    void signalForbiddenAreaChanged(ObjectId _t1, SignalMode _t2, polygon2D & _t3); // Forbidden area according to one robot
    void signalShootTargetChanged(uint8_t id, SignalMode mode, PositionVelocity& posvel, bool aiming);
    void signalProjectSpeedChanged(ObjectId id, SignalMode mode, linepoint2D& speedVector);
    void signalClearRobot(uint8_t robotId);
    void signalGaussianObstaclesUpdate(uint8_t senderRobotId, SignalMode mode, T_DIAG_WORLDMODEL_LOCAL& worldmodel_local);
    void signalTrueBallUpdate(uint8_t senderRobotId, SignalMode mode, T_DIAG_TRUE_BALL& true_ball);
    /*
    * Time since start of data log, used in renderer for hiding objects
    */
    void signalClearAll();
    void signalElapsedTimeChanged(double logElapsedTime);
    
    /*
    * Pathplanning. 
    * path = list of coordinates, first is first on path, last is the final target on the path
    */
    void signalPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path);

    /*
    * Robot- and team event logging
    */
    void signalLog(LogEvent event); 

    /*
    * Free form text logging
    */
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, float value);
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, bool value);
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, std::string value);
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, std::vector<std::string> value);
    /*
    * Match analytics
    * logElapsedTime: The time elapsed during this logging (t=0 is the start of the logfile)
    * actualTime: The time on the clock at the current match state (e.g. 17:23:46.889), can be related to on-robot tracing
    */
    void signalClockTick(double logElapsedTime, double actualTime);
    // TODO: signalMatchTick: The time elapsed during match (t=0 at refbox half start)

    void signalRefBoxCommand(uint8_t senderRobotId, std::string command);
    void signalRefBoxCommandTime(uint8_t senderRobotId, double commandTime);
    void signalGoal(uint8_t senderRobotId, int goals);
    void signalPhase(uint8_t senderRobotId, int phase);
    void signalOutOfPlay(uint8_t senderRobotId, bool outofplay);
};

#endif // GAMESIGNALADAPTER_H
