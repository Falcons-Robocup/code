// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * runningController.hpp
 *
 *  Created on: Dec 29, 2018
 *      Author: Coen Tempelaars
 */

#ifndef RUNNINGCONTROLLER_HPP_
#define RUNNINGCONTROLLER_HPP_

#include "boost/signals2.hpp"

#include "abstractRefBoxAdapter.hpp"
#include "arbiterGameData.hpp"
#include "judgement.hpp"

class RunningController {
public:
    typedef boost::signals2::signal<void (TeamID)> goalSignal_t;
    boost::signals2::connection goalSignalSubscribe (const goalSignal_t::slot_type&);

    typedef boost::signals2::signal<void (Judgement)> signal_t;
    boost::signals2::connection stoppingSignalSubscribe (const signal_t::slot_type&);

    /* Control a game in 'playing' state. The controller can either allow the game
     * to continue or stop the game (i.e. trigger the refbox 'stop' command).
     *
     * If and only if the controller stops the game, it must raise the stopping signal
     * before it returns from the control method. */
    void control (const ArbiterGameData&);
    void control (const ArbiterGameData&, const float secondsSinceLastTransition);

private:
    void declareGameStopping(const Judgement&);

    goalSignal_t _goalSignal;
    signal_t _stoppingSignal;
    bool _signalSent;
};

#endif /* RUNNINGCONTROLLER_HPP_ */
