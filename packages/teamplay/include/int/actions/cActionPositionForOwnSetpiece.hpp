// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPositionForOwnSetpiece.hpp
 *
 *  Created on: Apr 24, 2018
 *      Author: Coen Tempelaars
 */

#ifndef CACTIONPOSITIONFOROWNSETPIECE_HPP_
#define CACTIONPOSITIONFOROWNSETPIECE_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionPositionForOwnSetpiece : public cAbstractAction
{
public:
    cActionPositionForOwnSetpiece();
    ~cActionPositionForOwnSetpiece();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONPOSITIONFOROWNSETPIECE_HPP_ */
