// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mLogdump.cpp
 *
 * Dump log to stdout.
 *
 *  Created on: January 2019
 *      Author: Jan Feitsma
 */


#include "ext/cLogFileReader.hpp"
#include <iostream>
#include <fstream>
#include <set>


int main(int argc, char **argv)
{
    if (argc == 1)
    {
        std::cerr << "ERROR: no logfile given" << std::endl;
        return 1;
    }
    
    // TODO option parsing: select agent, timeframe, etc
    
    // prepare
    cLogFileReader logFile(argv[1]);
    tLogHeader header = logFile.getHeader();
    int frameCounter = 0;
    tLogFrame frame;
    
    // heading
    std::cout << " frame      age  size                  timestamp" << std::endl;
    
    // run
    while (logFile.getFrame(frame))
    {
        frameCounter++;
        std::cout << " " << std::setw(5) << frameCounter;
        std::cout << " " << std::setw(8) << std::fixed << std::setprecision(2) << frame.age;
        std::cout << " " << std::setw(5) << (int)frame.data.size();
        rtime t = header.creation + frame.age;
        std::cout << " " << t.toStr();
        // TODO dump all keys? require RTDB API change
        std::cout << std::endl;
    }

    return 0;
}

