// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogFilePlayback.cpp
 *
 */

#include <boost/bind.hpp>
#include "ext/cLogFilePlayback.hpp"

#include "tracing.hpp"


cLogFilePlayback::cLogFilePlayback(std::string const &filename)
{
    _logFile = new cLogFileReader(filename);
    load(); // TODO: could take a while ... move this into separate thread?
}

cLogFilePlayback::~cLogFilePlayback()
{
}

void cLogFilePlayback::load()
{
    TRACE_FUNCTION("");
    _header = _logFile->getHeader();
    // set current timestamp
    _t = _header.creation;
    TRACE("creation=%s", _t.toStr().c_str());
    // load file into memory
    tLogFrame frame;
    while (_logFile->getFrame(frame))
    {
        rtime t = _header.creation + frame.age;
        _frameBuffer.insert(t, frame);
    }
    _frameBuffer.traceStats();
    // read first frame(s)
    _frameBuffer.getIndex(0, _currentFrame);
    _frameBuffer.getIndex(1, _nextFrame);
    _index = 0;
}

