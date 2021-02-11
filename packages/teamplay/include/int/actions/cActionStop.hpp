// Copyright 2015-2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionStop.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CACTIONSTOP_HPP_
#define CACTIONSTOP_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionStop : public cAbstractAction
{
public:
    cActionStop();
    ~cActionStop();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);
    void stopRobot();

private:

};


#endif /* CACTIONSTOP_HPP_ */
