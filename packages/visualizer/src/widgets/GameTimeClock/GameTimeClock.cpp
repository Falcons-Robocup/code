// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GameTimeClock.h
 *
 *  Created on: October 23, 2016
 *      Author: Diana Koenraadt
 */

#include <QTimer>
#include <QTime>

// Internal:
#include "int/widgets/GameTimeClock/GameTimeClock.h"

GameTimeClock::GameTimeClock(QWidget* parent)
      : Widget()
{
    setFixedHeight(50);
    setSegmentStyle(Filled);
    setDigitCount(11);

    clear();
}

void GameTimeClock::clear()
{
    display("--:--");
}

void GameTimeClock::setTime(QTime elapsedTime, QTime actualTime)
{
    QString text = actualTime.toString("hh:mm:ss.zzz");
    // milliseconds are a bit overkill
    text.remove(text.size()-1, 1);

    // TODO also show elapsed time and playtime since match start, 
    // but that would not fit in a single QLCDnumber widget
    // so we need first align on desired layout 

/*
    if ((time.second() % 2) == 0)
    {
        text[2] = ' '; // Blinks the colon every second
    }
*/
    display(text);
}

