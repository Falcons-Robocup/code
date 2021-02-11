// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPositionForOppSetpiece.hpp
 *
 *  Created on: Jun 10, 2018
 *      Author: Coen Tempelaars
 */

#ifndef CACTIONPOSITIONFOROPPSETPIECE_HPP_
#define CACTIONPOSITIONFOROPPSETPIECE_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionPositionForOppSetpiece : public cAbstractAction
{
public:
    cActionPositionForOppSetpiece();
    ~cActionPositionForOppSetpiece();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONPOSITIONFOROPPSETPIECE_HPP_ */
