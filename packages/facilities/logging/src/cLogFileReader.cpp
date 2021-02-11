// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogFileReader.cpp
 */

#include <boost/bind.hpp>
#include "ext/cLogFileReader.hpp"

#include "tracing.hpp"


cLogFileReader::cLogFileReader(std::string const &filename)
{
    _filename = filename;
    openFile();
}

tLogHeader cLogFileReader::getHeader()
{
    return _header;
}
    
cLogFileReader::~cLogFileReader()
{
    close();
}

void cLogFileReader::close()
{
    if (_file.is_open())
    {
        _file.close();
    }
}

template <typename T>
bool readRaw(T &result, std::ifstream &file)
{
    size_t sz = 0;
    file.read((char*)(&sz), sizeof(size_t));
    
    if (!file.good())
    {
        return false;
    }
    msgpack::sbuffer sbuf(sz);
    file.read((char*)(sbuf.data()), sz);

    // unpack
    try 
    {
        msgpack::object_handle msg = msgpack::unpack(sbuf.data(), sz);
        msgpack::object obj = msg.get();
        obj.convert(result);
    } 
    catch (...) 
    {
        throw std::runtime_error("Failed to read object from log file");
    }
    return true;
}

bool cLogFileReader::getFrame(tLogFrame &frame)
{
    if (!_file.good())
    {
        return false;
    }

    // read next
    return readRaw(frame, _file);
}

void cLogFileReader::openFile()
{
    TRACE_FUNCTION("");
    // close if already open (use case: reset)
    if (_file.is_open())
    {
        _file.close();
        _file.clear();
    }
    // open
    _file.open(_filename, std::ios::binary);
    if (!_file.is_open())
    {
        throw std::runtime_error("Failed to open log file " + _filename);
    }
    // read header
    if (!readRaw(_header, _file))
    {
        throw std::runtime_error("Failed to read header from log file " + _filename);
    }
}

