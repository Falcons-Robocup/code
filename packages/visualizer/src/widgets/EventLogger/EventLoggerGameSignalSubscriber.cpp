// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * EventLoggerGameSignalSubscriber.cpp
 *
 *  Created on: August 2, 2016
 *      Author: Diana Koenraadt
 */

#include <iostream>

// Internal:
#include "int/widgets/EventLogger/EventLogger.h"

void EventLoggerGameSignalSubscriber::subscribe(GameSignalAdapter* gameSignalAdapter)
{
    WidgetGameSignalSubscriber::subscribe(gameSignalAdapter);

    QObject::connect(gameSignalAdapter, SIGNAL(signalLog(LogEvent)), this, SLOT(onLog(LogEvent)));
}

void EventLoggerGameSignalSubscriber::onLog(LogEvent event)
{
    _widget->log(event);
}
