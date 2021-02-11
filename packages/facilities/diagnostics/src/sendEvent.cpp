// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * sendEvent.cpp
 *
 *  Created on: April, 2019 
 *      Author: Jan Feitsma
 */

#include "ext/cDiagnostics.hpp"


void usage()
{
    fprintf(stdout, "\n");
    fprintf(stdout, "usage: sendEvent type message\n");
    fprintf(stdout, "type must be one of {INFO, WARNING, ERROR}\n");
    fprintf(stdout, "\n");
}

int main(int argc, char **argv)
{
    // check argument count
    if (argc < 3)
    {
        fprintf(stderr, "ERROR: expected more arguments\n");
        usage();
        return 1;
    }
    
    // check event type
    eventSeverityEnum severity = eventSeverityEnum::INFO;
    if (std::string(argv[1]) == "INFO")
    {
        severity = eventSeverityEnum::INFO;
    }
    else if (std::string(argv[1]) == "WARNING")
    {
        severity = eventSeverityEnum::WARNING;
    }
    else if (std::string(argv[1]) == "ERROR")
    {
        severity = eventSeverityEnum::ERROR;
    }
    else
    {
        fprintf(stderr, "ERROR: invalid type (%s)\n", argv[1]);
        usage();
        return 1;
    }
    
    // dispatch event
    (diagnostics::EventHandler(severity,__FILE__,__LINE__,__FUNCTION__))(argv[2]);
    return 0;
}

