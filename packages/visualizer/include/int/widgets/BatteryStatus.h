 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
   	QString _default = "QProgressBar{background-color: #000000;color: #;border-style: outset;border-width: 2px;border-color: #74c8ff;border-radius: 7px;text-align: center; } QProgressBar::chunk {background-color: 5bea22)}";


    void clear();
    void setBatteryStatus(uint8_t senderRobotId, float voltageValue);
    void setOutofPlay(uint8_t senderRobotId);
};
/*
* Class that handles subscriber data for the Match State
*/
#endif // BATTERYSTATUS_H

