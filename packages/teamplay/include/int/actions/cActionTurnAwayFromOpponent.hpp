// Copyright 2018 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionTurnAwayFromOpponent.hpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Erik Kouters
 */

#ifndef CACTIONTURNAWAYFROMOPPONENT_HPP_
#define CACTIONTURNAWAYFROMOPPONENT_HPP_

#include "int/actions/cAbstractAction.hpp"
#include "int/utilities/timer.hpp"

class cActionTurnAwayFromOpponent : public cAbstractAction
{
public:
    cActionTurnAwayFromOpponent();
    ~cActionTurnAwayFromOpponent();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONTURNAWAYFROMOPPONENT_HPP_ */
