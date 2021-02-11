// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameTimeClockGameSignalSubscriber.h
 *
 *  Created on: October 23, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/GameTimeClock.h"

// QT:
#include <QDateTime>

void GameTimeClockGameSignalSubscriber::subscribe(GameSignalAdapter* gameSignalAdapter)
{
    WidgetGameSignalSubscriber::subscribe(gameSignalAdapter);

    QObject::connect(gameSignalAdapter, SIGNAL(signalClockTick(double, double)), this, SLOT(onClockTick(double, double)));
}

QTime convertElapsed(double t)
{
    int h = (int)(t) / 3600;
    t -= 3600 * h;
    int m = (int)(t) / 60;
    t -= 60 * m;
    int s = (int)(t);
    t -= s;
    int ms = (int)(1000 * t);
    return QTime(h, m, s, ms);
}

QTime convertActual(double t)
{
    QDateTime datetime;
    datetime.setTime_t((int)t);
    QTime result = datetime.time();
    int msecs = 1000 * (t - (int)t);
    return result.addMSecs(msecs);
}

void GameTimeClockGameSignalSubscriber::onClockTick(double elapsedTime, double actualTime)
{
    QTime elapsedQTime = convertElapsed(elapsedTime);
    QTime actualQTime = convertActual(actualTime);
    _widget->setTime(elapsedQTime, actualQTime);
}

