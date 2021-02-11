// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionMoveToFreeSpot.hpp
 *
 *  Created on: May 22, 2016
 *      Author: Tim Kouters
 */

#ifndef CACTIONMOVETOFREESPOT_HPP_
#define CACTIONMOVETOFREESPOT_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionMoveToFreeSpot : public cAbstractAction
{
public:
    cActionMoveToFreeSpot();
    ~cActionMoveToFreeSpot();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONMOVETOFREESPOT_HPP_ */
