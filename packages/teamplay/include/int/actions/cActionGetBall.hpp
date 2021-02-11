// Copyright 2016 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionGetBall.hpp
 *
 *  Created on: May 4, 2016
 *      Author: Erik Kouters
 */

#ifndef CACTIONGETBALL_HPP_
#define CACTIONGETBALL_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionGetBall : public cAbstractAction
{
public:
    cActionGetBall();
    ~cActionGetBall();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONGETBALL_HPP_ */
