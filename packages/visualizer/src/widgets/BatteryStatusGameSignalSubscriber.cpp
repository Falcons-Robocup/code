// Copyright 2019 sgurunar (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BatteryStatusGameSignalSubscriber.h
 *
 *  Created on: May 22, 2019
 *      Author: Surya Gurunarayanan
 */

// Internal:
#include "int/widgets/BatteryStatus.h"
#include <iostream>
// QT:

void BatteryStatusGameSignalSubscriber::subscribe(GameSignalAdapter* gameSignalAdapter)
{
    WidgetGameSignalSubscriber::subscribe(gameSignalAdapter);

    QObject::connect(gameSignalAdapter, SIGNAL(signalValue(uint8_t, std::string, std::string, float)), this, SLOT(onValue(uint8_t, std::string, std::string, float)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalOutOfPlay(uint8_t, bool)), this, SLOT(onOutofPlay(uint8_t, bool)));

}

void BatteryStatusGameSignalSubscriber::onValue(uint8_t senderRobotId, std::string category, std::string key, float value)
{
    if(key == "voltage")
    {
        _widget->setBatteryStatus(senderRobotId, value);
    }
}

void BatteryStatusGameSignalSubscriber::onOutofPlay(uint8_t senderRobotId, bool outofplay)
{
    if(outofplay)
    {
        _widget->setOutofPlay(senderRobotId);
    }
}
