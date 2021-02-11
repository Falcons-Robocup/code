// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameSignalSubscriberStub.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef GAMESIGNALSUBSCRIBERSTUB_H
#define GAMESIGNALSUBSCRIBERSTUB_H

#include <QObject>

#include <vector>

#include "int/GameSignalAdapter.h"
#include "int/GameSignalSubscriber.h"

/*
* Helper class that subscribes to all game signals and keeps a list of string representations of the received signals
* Note: Rounds to 2 decimal places
*/
class GameSignalSubscriberStub : public QObject, public GameSignalSubscriber
{
    Q_OBJECT
public:
    GameSignalSubscriberStub(GameSignalAdapter* adapter);
    ~GameSignalSubscriberStub();
    void subscribeBallPositionChanged();
    void subscribeBallPossessionChanged();
    void subscribeObstaclePositionChanged();
    void subscribeOwnTeamPositionChanged();
    void subscribeValue();
    void subscribeTime();
    void subscribePathPlanning();

    std::vector<std::string> receivedMessages;

private:
    GameSignalAdapter* signalAdapter;
    

public Q_SLOTS:
    virtual void onBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence, float age, CameraType camType) override; // Ball position according to one robot
    virtual void onBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId) override; // Ball possession according to one robot
    virtual void onOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel) override; // Team member position according to one robot
    virtual void onObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel) override; // Obstacle position according to one robot

    virtual void onLog(LogEvent event) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value) override;

    virtual void onClockTick(double elapsedTime, double actualTime) override;

    virtual void onPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path) override;
};

#endif // GAMESIGNALSUBSCRIBERSTUB_H
