// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogger.hpp
 *
 * Sample RTDB contents on some frequency and store it in a file,
 * so it can be used to playback & visualize later.
 *
 *  Created on: Aug 11, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGGER_HPP_
#define CLOGGER_HPP_


#include <string>
#include <vector>
#include <map>
#include <fstream>
#include "cLogFileWriter.hpp"
#include "FalconsRTDB.hpp"


class cLogger
{
public:
    cLogger();
    ~cLogger();

    void setFrequency(int frequency);
    std::string createFileName();
    void writeToFile(std::string filename = "auto");
    bool makeFrame(tLogFrame &frame, rtime const &t);
    void monitor(); // loop around tick()
    bool tickNow(); // poked from monitor
    bool tick(rtime const &t); // handy for stimulation

private:
    RtDB2* _rtdb = NULL;
    int _frequency;
    cLogFileWriter *_logFile = NULL;
    rtime _t0; // start
    rtime _te; // end
    tLogHeader _header;

};

#endif

