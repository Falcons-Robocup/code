// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * MatchStateGameSignalSubscriber.h
 *
 *  Created on: May 9, 2019
 *      Author: Lu Dai
 */

// Internal:
#include "int/widgets/MatchState/MatchState.h"

// QT:

void MatchStateGameSignalSubscriber::subscribe(GameSignalAdapter* gameSignalAdapter)
{
    WidgetGameSignalSubscriber::subscribe(gameSignalAdapter);

    QObject::connect(gameSignalAdapter, SIGNAL(signalRefBoxCommand(uint8_t, std::string)), this, SLOT(onCommandChanged(uint8_t, std::string)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalRefBoxCommandTime(uint8_t, double)), this, SLOT(onCommandTimeChanged(uint8_t, double)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalGoal(uint8_t, int)), this, SLOT(onGoal(uint8_t, int)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalPhase(uint8_t, int)), this, SLOT(onPhaseChanged(uint8_t, int)));
}

void MatchStateGameSignalSubscriber::onCommandChanged(uint8_t senderRobotId, std::string command)
{
    QString commandQStr = QString::fromStdString(command);
    _widget->setCommand(commandQStr);
}

void MatchStateGameSignalSubscriber::onCommandTimeChanged(uint8_t senderRobotId, double commandTime)
{
    QDateTime commandQDatetime;
    commandQDatetime.setTime_t((int)commandTime);
    QTime commandQTime = commandQDatetime.time();
    int msecs = 1000 * (commandTime - (int)commandTime);
    commandQTime = commandQTime.addMSecs(msecs);
    QString commandTimeQStr = commandQTime.toString("hh:mm:ss");

    _widget->setCommandTime(commandTimeQStr);
}

void MatchStateGameSignalSubscriber::onGoal(uint8_t senderRobotId, int goals)
{
    QString goalQStr = QString::number(goals);
    _widget->setGoal(goalQStr);
}

void MatchStateGameSignalSubscriber::onPhaseChanged(uint8_t senderRobotId, int phase)
{
    std::string phaseStr = "";
    switch(phase)
    {
    case 0:
        phaseStr = "UNKNOWN";
        break;
    case 1:
        phaseStr = "FIRST_HALF";
        break;
    case 2:
        phaseStr = "HALF_TIME";
        break;
    case 3:
        phaseStr = "SECOND_HALF";
        break;
    case 4:
        phaseStr = "FIRST_HALF_OVERTIME";
        break;
    case 5:
        phaseStr = "SECOND_HALF_OVERTIME";
        break;
    case 6:
        phaseStr = "END_GAME";
        break;
    }
    QString phaseQStr = QString::fromStdString(phaseStr);
    _widget->setPhase(phaseQStr);

}
