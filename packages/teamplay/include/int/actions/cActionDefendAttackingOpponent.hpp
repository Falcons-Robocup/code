// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionDefendAttackingOpponent.hpp
 *
 *  Created on: Jun 13, 2018
 *      Author: Coen Tempelaars
 */

#ifndef CACTIONDEFENDATTACKINGOPPONENT_HPP_
#define CACTIONDEFENDATTACKINGOPPONENT_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionDefendAttackingOpponent : public cAbstractAction
{
public:
    cActionDefendAttackingOpponent();
    ~cActionDefendAttackingOpponent();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONDEFENDATTACKINGOPPONENT_HPP_ */
