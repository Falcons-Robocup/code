// Copyright 2016 martijn van veen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionAvoidPOI.hpp
 *
 *  Created on: June 27, 2016
 *      Author: Martijn van Veen
 */

#ifndef CACTIONAVOIDPOI_HPP_
#define CACTIONAVOIDPOI_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionAvoidPOI : public cAbstractAction
{
public:
	cActionAvoidPOI();
    ~cActionAvoidPOI();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONAVOIDPOI_HPP_ */
