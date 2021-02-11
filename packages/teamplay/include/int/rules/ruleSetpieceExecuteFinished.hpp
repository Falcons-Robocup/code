// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleSetpieceExecuteFinished.hpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#ifndef RULESETPIECEEXECUTEFINISHED_HPP_
#define RULESETPIECEEXECUTEFINISHED_HPP_

#include "int/types/ball.hpp"
#include "int/types/gameState.hpp"
#include "int/utilities/timer.hpp"

#include <boost/shared_ptr.hpp>

class ruleSetpieceExecuteFinished
{
public:
    ruleSetpieceExecuteFinished(boost::shared_ptr<teamplay::timer>);
    virtual ~ruleSetpieceExecuteFinished();

    virtual void resetRule();
    virtual bool isRuleValid();

private:
    teamplay::ball _ballAtStartRule;
    double _ballMinDistance;
    teamplay::gameState _gameStateAtStartRule;
    boost::shared_ptr<teamplay::timer> _timerAtStartRule;
    double _timerMaxValue;
};

#endif /* RULESETPIECEEXECUTEFINISHED_HPP_ */
