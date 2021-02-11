// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * stoppingController.hpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Coen Tempelaars
 */

#ifndef STOPPINGCONTROLLER_HPP_
#define STOPPINGCONTROLLER_HPP_

#include "boost/signals2.hpp"

#include "arbiterGameData.hpp"

class StoppingController {
public:
    typedef boost::signals2::signal<void ()> signal_t;
    boost::signals2::connection stoppedSignalSubscribe (const signal_t::slot_type&);

    /* Control a game in 'stopping' state. The only possible transition in this state
     * is towards the 'stopped' state. This transition must eventually be taken.
     *
     * If and only if the controller declares the game stopped, it must raise the
     * stopped signal before it returns from the control method. */
    void control (const ArbiterGameData&);
    void control (const ArbiterGameData&, const float secondsSinceLastTransition);

private:
    void declareGameStopped();

    signal_t _stoppedSignal;
    bool _signalSent;
};

#endif /* STOPPINGCONTROLLER_HPP_ */
