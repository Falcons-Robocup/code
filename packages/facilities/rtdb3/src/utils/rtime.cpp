 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * rtime.cpp
 *
 * Timestamp utilities, conversions, standards.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */


#include "rtime.hpp"

rtime::rtime(double d)
{
    fromDouble(d);
}

rtime::~rtime()
{
}

rtime rtime::now()
{
    struct timeval timeNow = {0,0};
    gettimeofday(&timeNow, NULL);
    rtime t = rtime();
    t.fromTimeval(timeNow);
    return t;
}

void rtime::loop(float frequency, boost::function<bool(void)> callback)
{
    // TODO consider to replace this hand-written timer with boost::asio
    // https://www.boost.org/doc/libs/master/doc/html/boost_asio/tutorial/tuttimer2.html
    rtime t0 = now();
    float dt = 1.0 / frequency;
    int iteration = 0;
    bool ok = true;
    while (ok)
    {
        rtime t = now();
        // call the given function
        ++iteration;
        ok = callback();
        if (ok)
        {
            // calculate elapsed and time to sleep
            rtime elapsed = now() - t;
            rtime next = t0 + rtime().fromDouble(dt * iteration);
            double sleepTime = (double)next - (double)t - (double)elapsed;
            if (sleepTime > 0)
            {
                int usec = round(1e6 * sleepTime);
                usleep(usec);
            }
            else
            {
                // if the delay is structural, this will spam!
                fprintf(stderr, "warning: callback took to long (%.3fs of allowed %.3fs)\n", dt - sleepTime, dt);
                fflush(stderr);
            }
        }
    }
}

rtime &rtime::fromTimeval(const struct timeval tv)
{
    _tvSec = tv.tv_sec;
    _tvUsec = tv.tv_usec;
    return *this;
}

struct timeval rtime::toTimeval() const
{
    struct timeval tv = {_tvSec, _tvUsec};
    return tv;
}

std::string rtime::toStr() const
{
    struct timeval tv = toTimeval();
    char timebuf[80] = {0};
    struct tm *tm;
    time_t t = tv.tv_sec;
    tm = localtime(&t);
    sprintf(timebuf, "%04d-%02d-%02d,%02d:%02d:%02d.%06d",
        1900 + tm->tm_year, 1 + tm->tm_mon, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec, (int)tv.tv_usec);
    // tm->tm_isdst
    return timebuf;
}

std::string rtime::toStrDate() const
{
    struct timeval tv = toTimeval();
    char timebuf[80] = {0};
    struct tm *tm;
    time_t t = tv.tv_sec;
    tm = localtime(&t);
    sprintf(timebuf, "%04d%02d%02d_%02d%02d%02d",
        1900 + tm->tm_year, 1 + tm->tm_mon, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec);
    // tm->tm_isdst
    return timebuf;
}

rtime &rtime::fromDouble(double d)
{
    _tvSec = floor(d);
    _tvUsec = round(1e6 * (d - _tvSec));
    return *this;
}

double rtime::toDouble() const
{
    return _tvSec + 1e-6 * _tvUsec;
}

bool rtime::operator==(const rtime &other)
{
    return (_tvSec == other._tvSec) && (_tvUsec == other._tvUsec);
}

rtime &rtime::operator+=(const rtime &other)
{
    _tvSec += other._tvSec;
    _tvUsec += other._tvUsec;
    if (_tvUsec >= 1000000)
    {
        _tvUsec -= 1000000;
        _tvSec++;
    }
    return *this;
}

rtime &rtime::operator-=(const rtime &other)
{
    if (_tvUsec < other._tvUsec)
    {
        _tvSec--;
        _tvUsec += 1000000 - other._tvUsec;
    }
    else
    {
        _tvUsec -= other._tvUsec;
    }
    _tvSec -= other._tvSec;
    return *this;
}

