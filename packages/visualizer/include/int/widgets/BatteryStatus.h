// Copyright 2019-2020 sgurunar (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BatteryStatus.h
 *
 *  Created on: May 23, 2019
 *      Author: Surya Gurunarayanan
 */

#ifndef BATTERYSTATUS_H
#define BATTERYSTATUS_H

#include <QtGui>
#include <QLCDNumber>
#include "FalconsRtDB2.hpp" // for rtime
#include <qgridlayout.h>
#include <qcheckbox.h>
#include <qlabel.h>
#include <qfont.h>
#include <QWidget>
#include <QProgressBar>

// Internal:
#include "int/widgets/Widget.h"


class GameSignalAdapter;
class BatteryStatus;

class BatteryStatusGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<BatteryStatus>
{
    Q_OBJECT
public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;

public Q_SLOTS:
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) override;
    virtual void onOutofPlay(uint8_t senderRobotId, bool outofplay) override;
};

/*
* BATTERY STATUS class
*/

class BatteryStatus : public QLCDNumber, public Widget<BatteryStatusGameSignalSubscriber, BatteryStatus>
{
    Q_OBJECT
friend BatteryStatusGameSignalSubscriber;

public:
    BatteryStatus(QWidget* parent = 0);

private:

    float _minVoltage = 20.0;
    float _maxVoltage = 28.0;
    QList<QLabel *> _batteryStatus;
    QList<QProgressBar *> _batteryStatusBar;

    QString _red   ="QProgressBar{border-style: outset;border-width: 2px;border-color: #74c8ff;border-radius: 7px;text-align: center; background-color: white;} QProgressBar::chunk{background-color: #FF0000;}";
    QString _amber ="QProgressBar{border-style: outset;border-width: 2px;border-color: #74c8ff;border-radius: 7px;text-align: center; background-color: white;} QProgressBar::chunk{background-color: #FF8000;}";
   	QString _green ="QProgressBar{border-style: outset;border-width: 2px;border-color: #74c8ff;border-radius: 7px;text-align: center; background-color: white;} QProgressBar::chunk{background-color: #80FF00;}";
   	QString _default = "QProgressBar{background-color: #000000;color: white; border-style: outset;border-width: 2px;border-color: #74c8ff;border-radius: 7px;text-align: center; } QProgressBar::chunk{background-color: #5bea22;}";


    void clear();
    void setBatteryStatus(uint8_t senderRobotId, float voltageValue);
    void setOutofPlay(uint8_t senderRobotId);
};
/*
* Class that handles subscriber data for the Match State
*/
#endif // BATTERYSTATUS_H

