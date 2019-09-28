 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

