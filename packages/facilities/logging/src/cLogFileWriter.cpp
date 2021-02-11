// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogFileWriter.cpp
 *
 *  Created on: Dec, 2018
 *      Author: Jan Feitsma
 */


#include "ext/cLogFileWriter.hpp"
#include <boost/filesystem.hpp>


cLogFileWriter::cLogFileWriter(std::string const &filename)
{
    _filename = filename;
    // prepare the file
    openFile();
}

cLogFileWriter::~cLogFileWriter()
{
    closeFile();
}

void cLogFileWriter::closeFile()
{
    if (_file.is_open())
    {
        _file.close();
    }
}

void cLogFileWriter::openFile()
{
    if (boost::filesystem::exists(_filename))
    {
        // open in r/w mode, for instance to fix header
        _file.open(_filename, std::ios::binary | std::ios::out | std::ios::in);
    }
    else
    {
        // new file
        _file.open(_filename, std::ios::binary | std::ios::out);
    }
    if (!_file.is_open())
    {
        throw std::runtime_error("Failed to open log file for writing: " + _filename);
    }
}

template <typename T>
void writeRaw(T const &data, std::fstream &file)
{
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, data);
    size_t sz = sbuf.size();
    file.write((const char*)(&sz), sizeof(size_t));
    file.write((const char*)(sbuf.data()), sz);
}

void cLogFileWriter::writeHeader(tLogHeader const &header)
{
    // pack and write
    _file.seekp(0);
    writeRaw(header, _file);
    // TODO: this function is meant to be called at start and end of logging; if desired to do more often, we would need to seek back to after last frame
}

void cLogFileWriter::writeFrame(tLogFrame const &frame)
{
    // pack and write
    writeRaw(frame, _file);
}

