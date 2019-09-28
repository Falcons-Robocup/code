 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * timer.cpp
 *
 *  Created on: 2018-12-09
 *      Author: Jan Feitsma
 */


#include "timer.hpp"
#include "../utils/tprintf.hpp"


Timer::Timer(float duration)
{
    _duration = duration;
    _t0 = rtime::now();
}

Timer::~Timer()
{
}

void Timer::setDuration(float duration)
{
    _duration = duration;
}

void Timer::sleep()
{
    rtime t = rtime::now();
    float elapsed = double(t) - double(_t0);
    if (elapsed < 0)
    {
        elapsed = 0; // be robust for CPU clock corrections
    }
    int usecToSleep = int(1e6 * (_duration - elapsed));
    if (usecToSleep >= 0)
    {
        usleep(usecToSleep);
    }
    else
    {
        tprintf("usecToSleep=%d elapsed=%.6f _duration=%.3f _t0=%s t=%s", usecToSleep, elapsed, _duration, _t0.toStr().c_str(), t.toStr().c_str());
    }
    // prepare for next - typically exactly one iteration, but in case previous one took too long, we align on next beat
    int steps = 0;
    while (_t0 < t)
    {
        steps += 1;
        _t0 = _t0 + _duration;
    }
    // warn if steps > 1
    if (steps > 1)
    {
        tprintf("WARNING: iteration took too long (elapsed=%.3fs, steps=%d)", elapsed, steps);
    }
}

