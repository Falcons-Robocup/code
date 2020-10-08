 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * BatteryStatus.cpp
 *
 *  Created on: May 23, 2019
 *      Author: Surya Gurunarayanan
 */

#include <QString>
#include <string>

// Internal:
#include "int/widgets/BatteryStatus.h"

BatteryStatus::BatteryStatus(QWidget* parent)
      : Widget()
{
    setSegmentStyle(Filled);
    setDigitCount(12);



    QGridLayout *batteryStatusLayout = new QGridLayout(this);

    clear();

//
    QList<QLabel *> batteryStatusLabel;
    int NUM_OF_ROBOTS = 7;
    for (int i = 0; i < NUM_OF_ROBOTS; i++)
    {
        std::ostringstream id;
        id << (i+1);
        std::string robotId = "R"+id.str()+ " : ";

        batteryStatusLabel << new QLabel(QString("R%1 : ").arg(i+1));
        batteryStatusLabel.at(i)->setStyleSheet("font: 10pt; color: white; ");
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
