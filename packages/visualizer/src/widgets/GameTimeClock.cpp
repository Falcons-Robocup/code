 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include <QTimer>
#include <QTime>

// Internal:
#include "int/widgets/GameTimeClock.h"

GameTimeClock::GameTimeClock(QWidget* parent)
      : Widget()
{
    setSegmentStyle(Filled);
    setDigitCount(12);

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

