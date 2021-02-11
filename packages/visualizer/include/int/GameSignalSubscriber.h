// Copyright 2016-2020 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameSignalSubscriber.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef GAMESIGNALSUBSCRIBER_H
#define GAMESIGNALSUBSCRIBER_H

#include "GameSignalAdapter.h" // For enum BallPossessionType and SignalMode definition
#include <time.h>

/*
* Subscriber interface for signals
* i.e. robot motion, world updates, etc. 
*/
class GameSignalSubscriber
{  
public:
    virtual ~GameSignalSubscriber()
    {
        if (_signalAdapter != NULL)
        {
            _signalAdapter = NULL;
        }
    }

    // Derived classes must call base subscribe first.
    virtual void subscribe(GameSignalAdapter* signalAdapter)
    {
        _signalAdapter = signalAdapter;
    }

    virtual void setSignalMode(SignalMode mode)
    {
        _viewSignalMode = mode;
    }

protected:
    SignalMode _viewSignalMode = WORLD; // Whether to handle integrated world view or per-robot vision view

private:
    GameSignalAdapter* _signalAdapter;

public:
    /* == Robot view  == */
    virtual void onBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence, float age, CameraType camType) {}; // Ball position according to one robot
    virtual void onBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId) {}; // Ball possession according to one robot
    virtual void onRobotStatusChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, int status) {};
    virtual void onOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel) {}; // Team member position according to one robot
    virtual void onObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel) {}; // Obstacle position according to one robot
    virtual void onForbiddenAreaChanged(ObjectId id, SignalMode signalMode, polygon2D& area) {}; // Obstacle position according to one robot
    virtual void onShootTargetChanged(uint8_t senderRobotId, SignalMode mode, PositionVelocity& posvel, bool aiming) {}; // Shoot target according to one robot
    virtual void onProjectSpeedChanged(ObjectId senderRobotId, SignalMode mode, linepoint2D& speedVector) {}; // Shoot target according to one robot
    virtual void onRobotClear(uint8_t robotID) {};
    virtual void onGaussianObstacleUpdate(uint8_t senderRobotId, SignalMode signalMode, T_DIAG_WORLDMODEL_LOCAL& worldmodel_local) {};
    virtual void onTrueBallUpdate(uint8_t senderRobotId, SignalMode signalMode, T_DIAG_TRUE_BALL& worldmodel_local) {};

    /*
    * FPS = vision FPS
    * linePoints = Detected number of white points that constitute a soccer field line
    * age = Vision 'locks' onto the robots own position. Every time it re-establishes its position, lastActive is reset to 0. So, larger is better.
    * lastActive = 
    */
    virtual void onVisionMetaDataChanged(uint8_t senderRobotId, SignalMode mode, float FPS, int linePoints, float age, float lastActive) {};

    /*
    * Pathplanning. 
    * path = list of coordinates, first is first on path, last is the final target on the path
    */
    virtual void onPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path) {};

    /*
    * Robot- and team event logging
    */
    virtual void onLog(LogEvent event) {};

    /*
    * Free form value logging
    */
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) {};
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) {};
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value) {};
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::vector<std::string> value) {};

    /*
    * Match analytics
    * logElapsedTime: The time elapsed during this logging (t=0 is the start of the logfile)
    * actualTime: The time on the clock at the current match state (e.g. 17:23:46.889), can be related to on-robot tracing
    */
    virtual void onClockTick(double logElapsedTime, double actualTime) {}; 

    virtual void onCommandChanged(uint8_t senderRobotId, std::string command) {};
    virtual void onCommandTimeChanged(uint8_t senderRobotId, double commandTime) {};
    virtual void onGoal(uint8_t senderRobotId, int goals) {};
    virtual void onPhaseChanged(uint8_t senderRobotId, int phase) {};
    virtual void onOutofPlay(uint8_t senderRobotId, bool outofplay) {};
};

#endif // GAMESIGNALSUBSCRIBER_H
