// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Timer.hpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

#ifndef TIMER_HPP_
#define TIMER_HPP_

#include "FalconsRTDB.hpp" // for rtime

namespace teamplay
{

class Timer {
public:
    Timer ();
    virtual ~Timer();

    virtual void reset();
    virtual bool hasElapsed( const double nrOfSeconds ) const;
    virtual bool hasStarted() const;

private:
    rtime _start_time;
    bool _has_started;

};

} /* namespace teamplay */

#endif /* TIMER_HPP_ */
