// Copyright 2016 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPositionBehindPOI.hpp
 *
 *  Created on: May 4, 2016
 *      Author: Erik Kouters
 */

#ifndef CACTIONPOSITIONBEHINDPOI_HPP_
#define CACTIONPOSITIONBEHINDPOI_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionPositionBehindPOI : public cAbstractAction
{
public:
    cActionPositionBehindPOI();
    ~cActionPositionBehindPOI();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONPOSITIONBEHINDPOI_HPP_ */
