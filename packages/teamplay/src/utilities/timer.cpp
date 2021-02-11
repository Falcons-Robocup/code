// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * timer.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

#include <stddef.h>
#include <sys/time.h>

#include "int/utilities/timer.hpp"
#include "tracing.hpp"
#include "ftime.hpp"

using namespace teamplay;

timer::timer()
{
    _has_started = false;
    _start_time = ftime::now();
}

timer::~timer()
{
    _has_started = false;
}

void timer::reset()
{
    _start_time = ftime::now();
    _has_started = true;
}

bool timer::hasElapsed (const double nrOfSeconds) const
{
    rtime time_now = ftime::now();

    double nrOfSecondsElapsed = ( time_now.toDouble() - _start_time.toDouble() );
    TRACE("Timer start: ") << _start_time.toStr()
       << " timer now: " << time_now.toStr()
       << " number of seconds elapsed: " << std::to_string(nrOfSecondsElapsed)
       << " number of seconds requested: " << std::to_string(nrOfSeconds);

    return (nrOfSecondsElapsed > nrOfSeconds);
}

bool timer::hasStarted () const
{
    return _has_started;
}
