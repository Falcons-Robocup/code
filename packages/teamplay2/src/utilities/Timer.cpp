// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Timer.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

#include <stddef.h>
#include <sys/time.h>

#include "int/utilities/Timer.hpp"
#include "tracing.hpp"
#include "ftime.hpp"

using namespace teamplay;

Timer::Timer()
{
    _has_started = false;
    _start_time = ftime::now();
}

Timer::~Timer()
{
    _has_started = false;
}

void Timer::reset()
{
    _start_time = ftime::now();
    _has_started = true;
}

bool Timer::hasElapsed (const double nrOfSeconds) const
{
    rtime time_now = ftime::now();

    double nrOfSecondsElapsed = ( time_now.toDouble() - _start_time.toDouble() );
    TRACE("Timer start: ") << _start_time.toStr()
       << " Timer now: " << time_now.toStr()
       << " number of seconds elapsed: " << std::to_string(nrOfSecondsElapsed)
       << " number of seconds requested: " << std::to_string(nrOfSeconds);

    return (nrOfSecondsElapsed > nrOfSeconds);
}

bool Timer::hasStarted () const
{
    return _has_started;
}
