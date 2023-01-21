// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameTimeClock.h
 *
 *  Created on: October 23, 2016
 *      Author: Diana Koenraadt
 */

#ifndef GAMETIMECLOCK_H
#define GAMETIMECLOCK_H

#include <time.h>
#include <QLCDNumber>

// Internal:
#include "int/widgets/Widget.h"

class GameSignalAdapter;
class GameTimeClock;

/*
* Class that handles subscriber data for the FieldWidget
* Use delegation to avoid diamond of death on QObject
*/
class GameTimeClockGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<GameTimeClock>
{
    Q_OBJECT
public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;

public Q_SLOTS:
    virtual void onClockTick(double elapsedTime, double actualTime) override;
};

class GameTimeClock : public QLCDNumber, public Widget<GameTimeClockGameSignalSubscriber, GameTimeClock>
{
    Q_OBJECT
friend GameTimeClockGameSignalSubscriber;

public:
    explicit GameTimeClock(QWidget *parent = 0);

private:
    void clear();
    void setTime(QTime elapsedTime, QTime actualTime);
};

#endif // GAMETIMECLOCK_H
