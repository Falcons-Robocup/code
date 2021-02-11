// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * timer.hpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

#ifndef TIMER_HPP_
#define TIMER_HPP_

#include "FalconsRtDB2.hpp" // for rtime

namespace teamplay
{

class timer {
public:
    timer ();
    virtual ~timer();

    virtual void reset();
    virtual bool hasElapsed( const double nrOfSeconds ) const;
    virtual bool hasStarted() const;

private:
    rtime _start_time;
    bool _has_started;

};

} /* namespace teamplay */

#endif /* TIMER_HPP_ */
