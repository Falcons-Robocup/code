// Copyright 2017-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TableWidgetGameSignalSubscriber.h
 *
 *  Created on: June 30, 2016
 *      Author: Diana Koenraadt
 */

#include <vector>
#include <tuple>

// Internal:
#include "int/widgets/Table/TableWidget.h"

void TableWidgetGameSignalSubscriber::subscribe(GameSignalAdapter* gameSignalAdapter)
{
    WidgetGameSignalSubscriber::subscribe(gameSignalAdapter);

    QObject::connect(gameSignalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, float)), this, SLOT(onValue(uint8_t, std::string, std::string, float)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, bool)), this, SLOT(onValue(uint8_t, std::string, std::string, bool)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, std::string)), this, SLOT(onValue(uint8_t, std::string, std::string, std::string)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalClearRobot(uint8_t)), this, SLOT(clearColumn(uint8_t)));

    // TODO: why maintain these lists instead of not just showing all?
    // TODO: Because this will become configurable. 

    _widget->_model->add("HEALTH", "cpuLoad");
    _widget->_model->add("HEALTH", "networkLoad");
    _widget->_model->add("HEALTH", "diskUsage");
    _widget->_model->add("PRIMARY_INFO", "diskUsage");

    _widget->_model->add("ACTIVE", "inplay");
    _widget->_model->add("ACTIVE", "pathPlanning");
    _widget->_model->add("ACTIVE", "shootPlanning");

    _widget->_model->add("HALMW", "voltage");
    _widget->_model->add("PRIMARY_INFO", "voltage");
    _widget->_model->add("HALMW", "bh_left_angle");
    _widget->_model->add("HALMW", "bh_right_angle");
    _widget->_model->add("HALMW", "temp_rear");
    _widget->_model->add("HALMW", "temp_left");
    _widget->_model->add("HALMW", "temp_right");
    _widget->_model->add("HALMW", "speed_m1_vel");
    _widget->_model->add("HALMW", "speed_m2_vel");
    _widget->_model->add("HALMW", "speed_m3_vel");
    _widget->_model->add("HALMW", "feedback_m1_vel");
    _widget->_model->add("HALMW", "feedback_m2_vel");
    _widget->_model->add("HALMW", "feedback_m3_vel");
    _widget->_model->add("HALMW", "has ball");
    _widget->_model->add("PRIMARY_INFO", "has ball");

//    _widget->_model->add("COMPASS", "orientation"); // in degrees

    _widget->_model->add("PRIMARY_INFO", "inplay");
    _widget->_model->add("WORLDMODEL", "inplay");
    _widget->_model->add("WORLDMODEL", "teamActivity");
    _widget->_model->add("PRIMARY_INFO", "teamActivity");
    _widget->_model->add("WORLDMODEL", "BhClaimed");
    _widget->_model->add("PRIMARY_INFO", "BhClaimed");
    _widget->_model->add("WORLDMODEL", "visionClaimed");
    _widget->_model->add("PRIMARY_INFO", "visionClaimed");
    _widget->_model->add("WORLDMODEL", "BhClaimed");
    _widget->_model->add("WORLDMODEL", "numObstacleTrackers");
    _widget->_model->add("WORLDMODEL", "numBallTrackers");
    _widget->_model->add("WORLDMODEL", "numVisCand");
    _widget->_model->add("WORLDMODEL", "numMotorSamples");
    _widget->_model->add("WORLDMODEL", "visScore");
    _widget->_model->add("WORLDMODEL", "visNoiseXY");
    _widget->_model->add("WORLDMODEL", "visNoisePhi");
    _widget->_model->add("WORLDMODEL", "validLoc");
    _widget->_model->add("PRIMARY_INFO", "validLoc");
    _widget->_model->add("WORLDMODEL", "duration");
    _widget->_model->add("WORLDMODEL", "bestTracker");
    _widget->_model->add("WORLDMODEL", "bestVisCand_x");
    _widget->_model->add("WORLDMODEL", "bestVisCand_y");
    _widget->_model->add("WORLDMODEL", "bestVisCand_Rz");
    _widget->_model->add("WORLDMODEL", "ownBallsFirst");

    _widget->_model->add("TEAMPLAY", "gamestate");
    _widget->_model->add("TEAMPLAY", "behavior");
    _widget->_model->add("TEAMPLAY", "role");
    _widget->_model->add("TEAMPLAY", "action");
    _widget->_model->add("PRIMARY_INFO", "gamestate");
    _widget->_model->add("PRIMARY_INFO", "behavior");
    _widget->_model->add("PRIMARY_INFO", "role");
    _widget->_model->add("PRIMARY_INFO", "action");
    _widget->_model->add("TEAMPLAY", "aiming");
    _widget->_model->add("TEAMPLAY", "refboxCmd");
    _widget->_model->add("TEAMPLAY", "shootTargetX");
    _widget->_model->add("TEAMPLAY", "shootTargetY");
}

void TableWidgetGameSignalSubscriber::onValue(uint8_t senderRobotId, std::string category, std::string key, float value)
{
    onValue<float>(senderRobotId, category, key, value);
}

void TableWidgetGameSignalSubscriber::onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) 
{
    onValue<bool>(senderRobotId, category, key, value);
}

void TableWidgetGameSignalSubscriber::onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value)
{
    onValue<const char*>(senderRobotId, category, key, value.c_str());
}

void TableWidgetGameSignalSubscriber::clearColumn(uint8_t robotId)
{
    _widget->_model->clearColumn((int)(robotId)-1);
}

