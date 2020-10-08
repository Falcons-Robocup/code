 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include <cmath>

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
    rtime t;
    t._timepoint = std::chrono::system_clock::now();
    return t;
}

void rtime::loop(float frequency, boost::function<bool(void)> callback, std::string const &id)
{
    // TODO consider to replace this hand-written timer with boost::asio
    // https://www.boost.org/doc/libs/master/doc/html/boost_asio/tutorial/tuttimer2.html
    rtime t0 = now();
    float dt = 1.0 / frequency;
    int iteration = 0;
    bool ok = true;
    std::string s = id;
    if (s.size()) s += " ";
    while (ok)
    {
        rtime t = now();
        // call the given function
        ++iteration;
        ok = callback();
        if (!ok) break;
        // calculate elapsed and time to sleep
        double elapsed = now() - t;
        rtime next = t0 + dt * iteration;
        double sleepTime = next - t - elapsed;
        if (sleepTime >= 0)
        {
            int usec = round(1e6 * sleepTime);
            usleep(usec);
        }
        else
        {
            // NOTE: if system load is high, it can occur that usleep of previous iteration takes significantly longer than expected, triggering this warning (should we fine-tune the warning?)
            fprintf(stderr, "warning: sleep and/or callback %stook to long (elapsed %.3fs, allowed %.3fs)\n", s.c_str(), elapsed, dt);
            fflush(stderr);
            // re-anchor zero timestamp to prevent spam
            t0 = now();
            iteration = 0;
        }
    }
}

struct timeval convertTimePointToTimeval(rtdb_timepoint_type t)
{
    struct timeval result = {0, 0};
    auto tt = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch());
    result.tv_sec = tt.count() / 1000000000;
    result.tv_usec = (tt.count() % 1000000000) / 1000;
    return result;
}

rtdb_timepoint_type convertTimevalToTimePoint(struct timeval t)
{
    rtdb_timepoint_type converted{std::chrono::nanoseconds{t.tv_sec*1000000000 + t.tv_usec*1000}};
    return converted;
}

rtime &rtime::fromTimeval(const struct timeval tv)
{
    _timepoint = convertTimevalToTimePoint(tv);
    return *this;
}

struct timeval rtime::toTimeval() const
{
    return convertTimePointToTimeval(_timepoint);
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
    struct timeval tv;
    tv.tv_sec = floor(d);
    tv.tv_usec = round(1e6 * (d - tv.tv_sec));
    return fromTimeval(tv);
}

double rtime::toDouble() const
{
    struct timeval tv = toTimeval();
    return tv.tv_sec + 1e-6 * tv.tv_usec;
}

rtime &rtime::operator+=(const double &d)
{
    auto duration = std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(d));
    _timepoint += duration;
    return *this;
}

rtime &rtime::operator-=(const double &d)
{
    auto duration = std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(d));
    _timepoint -= duration;
    return *this;
}

