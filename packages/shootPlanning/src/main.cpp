// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Thomas Kerkhof
 */

#include <iostream>


#include "int/cShootPlanner.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

/* Globals */
cShootPlanner shootPlanner;

cRTDBInputAdapter _rtdbInputAdapter;
cRTDBOutputAdapter _rtdbOutputAdapter;

using std::exception;
using std::cerr;
using std::endl;
using std::cout;



int main(int argc, char **argv)
{

    try
    {
        INIT_TRACE;

        // RTDB adapters
        _rtdbInputAdapter = cRTDBInputAdapter(&shootPlanner);
        _rtdbOutputAdapter = cRTDBOutputAdapter();

        // spin
        _rtdbInputAdapter.waitForShootSetpoint();
    } 
    catch (exception &e)
    {
        cerr << "Error occurred:" << e.what() << endl;
        TRACE_ERROR("Error occurred: %s", e.what());
    }
}
