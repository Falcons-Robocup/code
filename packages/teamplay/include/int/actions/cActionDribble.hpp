// Copyright 2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionDribble.hpp
 *
 *  Created on: Feb 15, 2020
 *      Author: Coen Tempelaars
 */

#ifndef CACTIONDRIBBLE_HPP_
#define CACTIONDRIBBLE_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionDribble : public cAbstractAction
{
public:
    cActionDribble();
    ~cActionDribble();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);
};

#endif /* CACTIONDRIBBLE_HPP_ */
