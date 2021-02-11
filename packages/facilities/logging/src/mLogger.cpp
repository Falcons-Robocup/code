// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mLogger.cpp
 *
 * Executable controlling logging of RtDB to file.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */


#include "ext/cLogger.hpp"
#include <boost/lexical_cast.hpp>

#include <signal.h>

#include "tracing.hpp"

cLogger *logger = NULL;

void sig_handler(int sig) 
{
    TRACE("signal=%d", sig);
    delete logger;
}

int main(int argc, char **argv)
{
    int frequency = 10;
    if (argc > 1)
    {
        frequency = boost::lexical_cast<int>(argv[1]);
    }
    sleep(1);
    // install signal handler
    signal(SIGINT, sig_handler);
    // run
    logger = new cLogger();
    logger->setFrequency(frequency);
    logger->writeToFile(); // no argument == automatic filename, with timestamp
    
    logger->monitor();
    
    return 0;
}

