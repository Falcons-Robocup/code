// Copyright 2017-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTimer.hpp
 *
 *  Created on: Nov 23, 2017
 *      Author: Jan Feitsma
 */

#ifndef CTIMER_HPP_
#define CTIMER_HPP_

#include "FalconsRtDB2.hpp" // for rtime

class cTimer
{
public:
    cTimer();
    ~cTimer();
    
    void reset();
    float elapsed();
    void setDuration(float duration);
    bool expired();

private:
    double _t0;
    float _duration;
};

#endif /* CTIMER_HPP_ */

