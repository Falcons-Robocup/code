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
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

#include <stddef.h>
#include <sys/time.h>

#include "int/utilities/timer.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;

timer::timer()
{
    gettimeofday(&_start_time, NULL);
}

timer::~timer()
{
}

void timer::reset()
{
    gettimeofday(&_start_time, NULL);
}

bool timer::hasElapsed (const double nrOfSeconds) const
{
    struct timeval now;
    gettimeofday(&now, NULL);

    double nrOfSecondsElapsed = (now.tv_sec - _start_time.tv_sec) + ((now.tv_usec - _start_time.tv_usec) / 1000000.0);
    TRACE("Timer start: ") << std::to_string(_start_time.tv_sec)
       << " timer now: " << std::to_string(now.tv_sec)
       << " number of seconds elapsed: " << std::to_string(nrOfSecondsElapsed)
       << " number of seconds requested: " << std::to_string(nrOfSeconds);

    return (nrOfSecondsElapsed > nrOfSeconds);
}
