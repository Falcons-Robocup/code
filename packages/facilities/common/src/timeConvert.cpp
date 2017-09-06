 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * timeConvert.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: Jan Feitsma
 */

#include "ext/timeConvert.hpp"

#include <cmath>
#include <cstdlib>
#include <sys/time.h>

// 1-1-2014 is our "epoch", so we can use microseconds resolution for the coming years
#define EPOCH 1388534400


double timeval2double(const struct timeval t)
{
	return (t.tv_sec - EPOCH) + 1e-6 * t.tv_usec;
}

struct timeval double2timeval(double t)
{
    struct timeval result;
    result.tv_sec = unsigned(floor(t)) + EPOCH;
    result.tv_usec = unsigned(round((t - floor(t)) * 1e6));   
    return result;
}

double getTimeNow()
{
	struct timeval timeNow;
	gettimeofday(&timeNow, NULL);
	return timeval2double(timeNow);
}

std::string timeToString(double t)
{
    return timeToString(double2timeval(t));
}

std::string timeToString(const struct timeval tv)
{
   char timebuf[80];
   struct tm *tm;
   tm = localtime(&tv.tv_sec);
   sprintf(timebuf, "%04d-%02d-%02d,%02d:%02d:%02d.%06d",
      1900 + tm->tm_year, 1 + tm->tm_mon, tm->tm_mday,
      tm->tm_hour, tm->tm_min, tm->tm_sec, (int)tv.tv_usec);
   return timebuf;
}

