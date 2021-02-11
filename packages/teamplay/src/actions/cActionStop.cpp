// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionStop.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionStop.hpp"
#include "cDiagnostics.hpp"


cActionStop::cActionStop()
{
    _intention.action = actionTypeEnum::STOP;
}

cActionStop::~cActionStop()
{

}

behTreeReturnEnum cActionStop::execute(const std::map<std::string, std::string> &parameters)
{
    stopRobot();
    sendIntention();
    return behTreeReturnEnum::PASSED;
}

void cActionStop::stopRobot()
{
    try
    {
        stop();
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionStop::execute Linked to: ") + e.what());
    }
}
