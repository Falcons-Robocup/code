// Copyright 2016 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPositionBeforePOI.hpp
 *
 *  Created on: May 3, 2016
 *      Author: Erik Kouters
 */

#ifndef CACTIONPOSITIONBEFOREPOI_HPP_
#define CACTIONPOSITIONBEFOREPOI_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionPositionBeforePOI : public cAbstractAction
{
public:
    cActionPositionBeforePOI();
    ~cActionPositionBeforePOI();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONPOSITIONBEFOREPOI_HPP_ */
