// Copyright 2020-2021 Jan Feitsma (Falcons)
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

using std::exception;
using std::cerr;
using std::endl;
using std::cout;



int main(int argc, char **argv)
{

    try
    {
        INIT_TRACE("shootPlanning");

        tprintf("setup cShootPlanner");
        cShootPlanner shootPlanner;

        tprintf("setup RTDB input adapter");
        cRTDBInputAdapter rtdbInputAdapter(&shootPlanner);

        tprintf("spin ...");
        rtdbInputAdapter.waitForShootSetpoint();
    } 
    catch (exception &e)
    {
        cerr << "Error occurred:" << e.what() << endl;
        TRACE_ERROR("Error occurred: %s", e.what());
    }
}
