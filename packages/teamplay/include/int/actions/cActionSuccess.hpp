// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionSuccess.hpp
 *
 *  Created on: Jun 11, 2016
 *      Author: Coen Tempelaars
 */

#ifndef CACTIONSUCCESS_HPP_
#define CACTIONSUCCESS_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionSuccess : public cAbstractAction
{
public:
    cActionSuccess();
    ~cActionSuccess();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};


#endif /* CACTIONSUCCESS_HPP_ */
