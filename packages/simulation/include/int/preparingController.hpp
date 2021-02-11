// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * preparingController.hpp
 *
 *  Created on: Jan 8, 2019
 *      Author: Coen Tempelaars
 */

#ifndef PREPARINGCONTROLLER_HPP_
#define PREPARINGCONTROLLER_HPP_

#include "boost/signals2.hpp"

#include "abstractRefBoxAdapter.hpp"
#include "arbiterGameData.hpp"

class PreparingController {
public:
    typedef boost::signals2::signal<void ()> signal_t;
    boost::signals2::connection preparedSignalSubscribe (const signal_t::slot_type&);
    boost::signals2::connection stoppingSignalSubscribe (const signal_t::slot_type&);

    /* Control a game in 'preparing for setpiece' state. Once all robots have positioned
     * themselves without moving the ball, the controller should start the game and
     * raise the prepared signal. If robots move the ball while positioning themselves,
     * the controller should stop the game and raise the stopping signal.
     *
     * If and only if the controller declares the game prepared or stopping, it must
     * raise the appropriate signal before it returns from the control method. */
    void control (const ArbiterGameData&);
    void control (const ArbiterGameData&, const float secondsSinceLastTransition);

private:
    void declareGamePrepared();
    void declareGameStopping();

    signal_t _preparedSignal;
    signal_t _stoppingSignal;
    bool _signalSent;
};

#endif /* PREPARINGCONTROLLER_HPP_ */
