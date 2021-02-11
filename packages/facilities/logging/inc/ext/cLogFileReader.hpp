// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogFileReader.hpp
 *
 * Read from a log file.
 *
 *  Created on: Dec, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGFILEREADER_HPP_
#define CLOGFILEREADER_HPP_


#include <string>
#include <fstream>

#include "tLogHeader.hpp"
#include "tLogFrame.hpp"


class cLogFileReader
{
public:
    cLogFileReader(std::string const &filename);
    ~cLogFileReader();
    void close();
    tLogHeader getHeader();
    bool getFrame(tLogFrame &frame); // return false if unsuccessful

private:
    std::string _filename;
    std::ifstream _file;
    tLogHeader _header;
    void openFile();
    
};

#endif

