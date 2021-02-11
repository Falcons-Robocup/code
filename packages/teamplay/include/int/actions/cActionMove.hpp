// Copyright 2016-2018 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionMove.hpp
 *
 *  Created on: Jan 4, 2016
 *      Author: Michel Koenen
 */

#ifndef CACTIONMOVE_HPP_
#define CACTIONMOVE_HPP_

#include "int/actions/cAbstractAction.hpp"
#include "int/utilities/timer.hpp"

class cActionMove : public cAbstractAction
{
public:
    cActionMove();
    ~cActionMove();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONMOVE_HPP_ */
