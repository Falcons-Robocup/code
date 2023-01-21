// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleSetpieceExecuteFinished.hpp
 *
 *  Created on: Nov 6, 2016
 *      Author: Coen Tempelaars
 */

#ifndef RULESETPIECEEXECUTEFINISHED_HPP_
#define RULESETPIECEEXECUTEFINISHED_HPP_

#include "int/types/Ball.hpp"
#include "int/types/GameState.hpp"
#include "int/utilities/Timer.hpp"

#include <boost/shared_ptr.hpp>

class RuleSetpieceExecuteFinished
{
public:
    RuleSetpieceExecuteFinished(boost::shared_ptr<teamplay::Timer>);
    virtual ~RuleSetpieceExecuteFinished();

    virtual void resetRule();
    virtual bool isRuleValid();

private:
    teamplay::Ball _ballAtStartRule;
    double _ballMinDistance;
    teamplay::GameState _gameStateAtStartRule;
    boost::shared_ptr<teamplay::Timer> _timerAtStartRule;
    double _timerMaxValue;
};

#endif /* RULESETPIECEEXECUTEFINISHED_HPP_ */
