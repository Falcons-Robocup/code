// Copyright 2016-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionInterceptBall.hpp
 *
 *  Created on: Jun 11, 2016
 *      Author: Jan Feitsma
 */

#ifndef CACTIONINTERCEPTBALL_HPP_
#define CACTIONINTERCEPTBALL_HPP_

#include "int/actions/cAbstractAction.hpp"
#include "position2d.hpp"

class cActionInterceptBall : public cAbstractAction
{
public:
    cActionInterceptBall();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:
};

#endif /* CACTIONINTERCEPTBALL_HPP_ */
