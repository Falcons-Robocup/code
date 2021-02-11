// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Oct 20, 2018
 *      Author: Jan Feitsma
 *
 * Main program for refbox relay.
 * Takes care of sending refbox commands to robots. 
 * Also sends back team state for MSL logging.
 * All of this via RTDB.
 */

#include "int/cRefboxRelay.hpp"
#include "tracing.hpp"

int main(int argc, char ** argv)
{
    INIT_TRACE_HOT_FLUSH;
    // no CLI
    cRefboxRelay r;
    r.run();
    return 0;
}

