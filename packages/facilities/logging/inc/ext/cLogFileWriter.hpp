// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogFileWriter.hpp
 *
 * Write to log file.
 *
 *  Created on: Dec, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGFILEWRITER_HPP_
#define CLOGFILEWRITER_HPP_


#include <string>
#include <fstream>
#include "tLogHeader.hpp"
#include "tLogFrame.hpp"


class cLogFileWriter
{
public:
    cLogFileWriter(std::string const &filename = "auto");
    ~cLogFileWriter();

    void writeHeader(tLogHeader const &header);
    void writeFrame(tLogFrame const &frame);
    
private:
    std::string _filename;
    std::fstream _file;
    void openFile();
    void closeFile();

};

#endif

