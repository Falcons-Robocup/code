// Copyright 2016-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionSuccess.cpp
 *
 * As its name implies, this action always succeeds. This action is useful e.g. when you are
 * only interested in one of the two outcomes of a (boolean) decision.
 *
 *  Created on: Jun 11, 2016
 *      Author: Coen Tempelaars
 */

#include "int/actions/cActionSuccess.hpp"


cActionSuccess::cActionSuccess()
{
    _intention.action = actionTypeEnum::UNKNOWN;
}

cActionSuccess::~cActionSuccess()
{

}

behTreeReturnEnum cActionSuccess::execute(const std::map<std::string, std::string> &parameters)
{
    sendIntention();
    return behTreeReturnEnum::PASSED;
}
