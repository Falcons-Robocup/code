// Copyright 2017-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTimer.hpp
 *
 *  Created on: Nov 23, 2017
 *      Author: Jan Feitsma
 */

#ifndef CTIMER_HPP_
#define CTIMER_HPP_

#include "FalconsRTDB.hpp" // for rtime

class cTimer
{
public:
    cTimer();
    ~cTimer();
    
    void reset();
    double elapsed();
    void setDuration(double duration);
    bool expired();

private:
    double _t0;
    double _duration;
};

#endif /* CTIMER_HPP_ */

