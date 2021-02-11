// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * stoppedController.hpp
 *
 *  Created on: Jan 9, 2019
 *      Author: Coen Tempelaars
 */

#ifndef STOPPEDCONTROLLER_HPP_
#define STOPPEDCONTROLLER_HPP_

#include "boost/signals2.hpp"

#include "abstractGameDataAdapter.hpp"
#include "abstractRefBoxAdapter.hpp"
#include "arbiterGameData.hpp"
#include "judgement.hpp"

class StoppedController {
public:
    typedef boost::signals2::signal<void ()> signal_t;
    boost::signals2::connection preparingSignalSubscribe (const signal_t::slot_type&);

    /* Control a game in 'stopped' state. In this state, the controller shall execute
     * given judgement. The only possible transition in this state is towards the
     * 'preparing' state. This transition must eventually be taken.
     *
     * If and only if the controller declares the game preparing, it must raise the
     * preparing signal before it returns from the control method. */
    void control (const ArbiterGameData&);

private:
    void declareGamePreparing();

    signal_t _preparingSignal;
    bool _signalSent;
};

#endif /* STOPPEDCONTROLLER_HPP_ */
