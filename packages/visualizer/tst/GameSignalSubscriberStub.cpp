// Copyright 2016-2020 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameSignalSubscriberStub.cpp
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include "GameSignalSubscriberStub.h"

#include "falconsCommon.hpp"

GameSignalSubscriberStub::GameSignalSubscriberStub(GameSignalAdapter* adapter)
      : signalAdapter(adapter)
{
}

void GameSignalSubscriberStub::subscribeBallPositionChanged()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalBallPositionChanged(ObjectId, SignalMode, PositionVelocity&, float, float, CameraType)), 
        this, SLOT(onBallPositionChanged(ObjectId, SignalMode, PositionVelocity&, float, float, CameraType)));
}

void GameSignalSubscriberStub::subscribeBallPossessionChanged()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalBallPossessionChanged(uint8_t, SignalMode, BallPossessionType, uint8_t)), 
        this, SLOT(onBallPossessionChanged(uint8_t, SignalMode, BallPossessionType, uint8_t)));
}

void GameSignalSubscriberStub::subscribeObstaclePositionChanged()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalObstaclePositionChanged(ObjectId, SignalMode, PositionVelocity&)), 
        this, SLOT(onObstaclePositionChanged(ObjectId, SignalMode, PositionVelocity&)));
}

void GameSignalSubscriberStub::subscribeOwnTeamPositionChanged()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalOwnTeamPositionChanged(uint8_t, SignalMode, uint8_t, PositionVelocity&)), 
        this, SLOT(onOwnTeamPositionChanged(uint8_t, SignalMode, uint8_t, PositionVelocity&)));
}

void GameSignalSubscriberStub::subscribeValue()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, float)), 
        this, SLOT(onValue(uint8_t, std::string, std::string, float)));
    QObject::connect(
        signalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, bool)), 
        this, SLOT(onValue(uint8_t, std::string, std::string, bool)));
    QObject::connect(
        signalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, std::string)), 
        this, SLOT(onValue(uint8_t, std::string, std::string, std::string)));
}

void GameSignalSubscriberStub::subscribeTime()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalClockTick(double, double)), 
        this, SLOT(onClockTick(double, double)));
}

void GameSignalSubscriberStub::subscribePathPlanning()
{
    QObject::connect(
        signalAdapter, SIGNAL(signalPathPlanningInProgress(uint8_t, std::vector<PositionVelocity>&)), 
        this, SLOT(onPathPlanningInProgress(uint8_t, std::vector<PositionVelocity>&)));
}

GameSignalSubscriberStub::~GameSignalSubscriberStub()
{
    // Unsubscribe
    QObject::disconnect(signalAdapter, 0, 0, 0);
    signalAdapter = NULL;
}

/* 
* ========================================
*           Robot view
* ======================================== 
*/

void GameSignalSubscriberStub::onBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence, float age, CameraType camera)
{
    std::stringstream stream;
    stream << "onBallPositionChanged { " << 
        "id : " << (int)id << ", " << 
        "x : " << std::fixed << std::setprecision(2) << posvel.x << ", " << 
        "y : " << posvel.y << ", " << 
        "z : " << posvel.z << ", " << 
        "vx : " << posvel.vx << ", " << 
        "vy : " << posvel.vy << ", " << 
        "vz : " << posvel.vz << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId)
{
    std::stringstream stream;
    stream << "onBallPossessionChanged { " << 
        "senderRobotId : " << (int)senderRobotId << ", " << 
        "ballPossessionType : " << type << ", " << 
        "robotId : " << (int)robotId << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel)
{
    std::stringstream stream;
    stream << "onOwnTeamPositionChanged { " << 
        "senderRobotId : " << (int)senderRobotId << ", " << 
        "robotId : " << (int)robotId << ", " << 
        "x : " << std::fixed << std::setprecision(2) << posvel.x << ", " << 
        "y : " << posvel.y << ", " << 
        "phi : " << posvel.phi << ", " << 
        "vx : " << posvel.vx << ", " << 
        "vy : " << posvel.vy << ", " << 
        "vphi : " << posvel.vphi << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel)
{
    std::stringstream stream;
    stream << "onObstaclePositionChanged { " << 
        "id : " << (int)id << ", " << 
        "x : " << std::fixed << std::setprecision(2) << posvel.x << ", " << 
        "y : " << posvel.y << ", " << 
        "phi : " << posvel.phi << ", " << 
        "vx : " << posvel.vx << ", " << 
        "vy : " << posvel.vy << ", " << 
        "vphi : " << posvel.vphi << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onLog(LogEvent event)
{
    std::stringstream stream;
    stream << "onLog { " << 
        "senderRobotId : " << event.robotId << ", " << 
        "LogLevel : " << event.type << ", " << 
        "message : " << event.message << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onValue(uint8_t senderRobotId, std::string category, std::string key, float value)
{
    std::stringstream stream;
    stream << "onValue { " << 
        "senderRobotId : " << (int)senderRobotId << ", " << 
        "category : " << category << ", " << 
        "key : " << key << ", " << 
        "value : " << value << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) 
{
    std::stringstream stream;
    stream << "onValue { " << 
        "senderRobotId : " << (int)senderRobotId << ", " << 
        "category : " << category << ", " << 
        "key : " << key << ", " << 
        "value : " << value << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value)
{
    std::stringstream stream;
    stream << "onValue { " << 
        "senderRobotId : " << (int)senderRobotId << ", " << 
        "category : " << category << ", " << 
        "key : " << key << ", " << 
        "value : " << value << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onClockTick(double logElapsedTime, double actualTime)
{
    // convert times
    struct tm sts = {0};
    struct tm cts = {0};
    sts.tm_sec = (int)logElapsedTime;
    cts.tm_sec = (int)actualTime;
    time_t matchStartTime = mktime(&sts);
    time_t currentTime = mktime(&cts);

    char mst[10];
    char ct[10];

    std::stringstream stream;
    stream << "onClockTick { " << 
        "matchStartTime : ";
    if (std::strftime(mst, sizeof(mst), "%H:%M", std::localtime(&matchStartTime)))
    {
        stream << mst;
    }
    else
    {
        stream << "ERR";
    }
    stream << ", " << 
        "currentTime : ";
    if (std::strftime(ct, sizeof(ct), "%H:%M", std::localtime(&currentTime)))
    {
        stream << ct;
    }
    else
    {
        stream << "ERR";
    }
    stream << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

void GameSignalSubscriberStub::onPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path)
{
    std::stringstream stream;
    stream << "onPathPlanningInProgress { " << 
        "senderRobotId : " << (int)senderRobotId;

    for (size_t i = 0; i < path.size(); ++i)
    {
        stream << ", { " << 
        "x : " << path[i].x << ", " << 
        "y : " << path[i].y << ", " << 
        "z : " << path[i].z << ", " << 
        "phi : " << path[i].phi << ", " << 
        "vx : " << path[i].vx << ", " << 
        "vy : " << path[i].vy << ", " << 
        "vz : " << path[i].vz << ", " << 
        "vphi : " << path[i].vphi << " }";
    }

    stream << " }";
    std::string str = stream.str();
    receivedMessages.push_back(str);
}

