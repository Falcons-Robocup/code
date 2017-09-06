 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
