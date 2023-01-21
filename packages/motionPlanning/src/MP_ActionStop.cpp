// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionStop.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#include "../include/int/MP_ActionStop.hpp"

using namespace std;

void MP_ActionStop::initialize()
{
    TRACE_FUNCTION("");
}

actionResultTypeEnum MP_ActionStop::execute()
{
    TRACE_FUNCTION("");
    unpackParameters();
    stopMoving();
    _rtdbOutput->setBallHandlersSetpoint(_ballHandlersEnabled);
    return actionResultTypeEnum::PASSED;
}

void MP_ActionStop::unpackParameters()
{
    _ballHandlersEnabled = false;
    if (_params.size())
    {
        _ballHandlersEnabled = boost::lexical_cast<int>(_params.at(0));
    }
}

