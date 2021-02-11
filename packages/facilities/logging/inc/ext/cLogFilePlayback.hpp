// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogFilePlayback.hpp
 *
 * Load a log file and playback it contents, writing into RTDB.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGFILEPLAYBACK_HPP_
#define CLOGFILEPLAYBACK_HPP_


#include <string>
#include <fstream>

#include "cLogPlayback.hpp"
#include "cLogFileReader.hpp"


class cLogFilePlayback : public cLogPlayback
{
public:
    cLogFilePlayback(std::string const &filename);
    ~cLogFilePlayback();

private:
    cLogFileReader *_logFile = NULL;
    void load();
    
};

#endif

