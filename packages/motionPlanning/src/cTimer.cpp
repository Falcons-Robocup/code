// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTimer.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: Jan Feitsma
 */

#include "int/cTimer.hpp"
#include "ftime.hpp"


cTimer::cTimer()
{
    _duration = 0;
    reset();
}

cTimer::~cTimer()
{
}

void cTimer::reset()
{
    _t0 = ftime::now();
}

float cTimer::elapsed()
{
    return (double)(ftime::now() - _t0);
}

void cTimer::setDuration(float duration)
{
    _duration = duration;
}

bool cTimer::expired()
{
    return elapsed() >= _duration;
}

