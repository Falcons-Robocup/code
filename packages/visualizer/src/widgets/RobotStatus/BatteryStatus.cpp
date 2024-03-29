// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BatteryStatus.cpp
 *
 *  Created on: May 23, 2019
 *      Author: Surya Gurunarayanan
 */

#include <QString>
#include <string>

#include "falconsCommonLegacy.hpp" // MAX_ROBOTS

// Internal:
#include "int/widgets/RobotStatus/BatteryStatus.h"

BatteryStatus::BatteryStatus(QWidget* parent)
      : Widget()
{
    setSegmentStyle(Filled);
    setDigitCount(12);



    QGridLayout *batteryStatusLayout = new QGridLayout(this);

    clear();

//
    QList<QLabel *> batteryStatusLabel;
    for (int i = 0; i < MAX_ROBOTS; i++)
    {
        std::ostringstream id;
        id << (i+1);
        std::string robotId = "R"+id.str()+ " : ";

        batteryStatusLabel << new QLabel(QString("R%1 : ").arg(i+1));
        batteryStatusLabel.at(i)->resize(this->width()/3, this->height());

        _batteryStatus << new QLabel();
        _batteryStatus.at(i)->setStyleSheet("font: 10pt; color: white; text-align: center;");

        _batteryStatusBar << new QProgressBar();
        _batteryStatusBar.at(i)->setOrientation(Qt::Horizontal);
        _batteryStatusBar.at(i)->setRange(_minVoltage,_maxVoltage);
        _batteryStatusBar.at(i)->setValue(_minVoltage);
        _batteryStatusBar.at(i)->setStyleSheet(_default);
//        _batteryStatusBar.at(i)->setTextVisible(true);
        _batteryStatusBar.at(i)->setFormat("N.A.");

        batteryStatusLayout->addWidget(batteryStatusLabel.at(i),0, i*2, 1, 1);
        batteryStatusLayout->addWidget(_batteryStatus.at(i), 0, i*2+1, 1, 1);
        batteryStatusLayout->addWidget(_batteryStatusBar.at(i), 1, i*2, 1, 2);
    }
}

void BatteryStatus::setBatteryStatus(uint8_t senderRobotId, float voltageValue)
{
    std::ostringstream ss;
    ss.precision(4);
    ss << voltageValue;
    std::string val = ss.str();

    float batteryLeft = ((voltageValue-_minVoltage)/(_maxVoltage-_minVoltage)) * 100;

    _batteryStatus.at(senderRobotId-1)->setText(QString::fromStdString(val+"V"));
    _batteryStatusBar.at(senderRobotId-1)->setValue(voltageValue);
    _batteryStatusBar.at(senderRobotId-1)->setFormat(QString::number(round(batteryLeft))+"%");

    if(round(voltageValue)<22)
    {
        _batteryStatusBar.at(senderRobotId-1)->setStyleSheet(_red);
    }

    if(round(voltageValue)>=22 && round(voltageValue)<23)
        _batteryStatusBar.at(senderRobotId-1)->setStyleSheet(_amber);

    if(round(voltageValue)>=23)
        _batteryStatusBar.at(senderRobotId-1)->setStyleSheet(_green);
}

void BatteryStatus::setOutofPlay(uint8_t senderRobotId)
{
    _batteryStatusBar.at(senderRobotId-1)->setFormat("OUT");
    _batteryStatusBar.at(senderRobotId-1)->setStyleSheet(_red);
}

void BatteryStatus::clear()
{
    display("");
}
