// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogPlayback.cpp
 *
 */

#include <boost/bind.hpp>
#include "ext/cLogPlayback.hpp"

#include "tracing.hpp"

cLogPlayback::cLogPlayback()
{
    _index = 0;
    _rtdb = new RtDB2(0); // agent id is irrelevant when dealing with entire frames at once
}

tLogHeader cLogPlayback::getHeader()
{
    return _header;
}

float cLogPlayback::getDuration()
{
    tLogFrame lastFrame;
    if ( _frameBuffer.getIndex( (_frameBuffer.getSize()-1), lastFrame ) )
    {
        return lastFrame.age;
    }
    return -1.0;
}

bool cLogPlayback::step()
{
    TRACE_FUNCTION("");
    _currentFrame = _nextFrame;
    _index++;
    if (!_frameBuffer.getIndex(_index, _nextFrame))
    {
        // this actually stops one frame too early, which doesn't matter much
        return false;
    }
    putFrame(_currentFrame);
    return true;
}

bool cLogPlayback::stepBack()
{
    TRACE_FUNCTION("");
    _nextFrame = _currentFrame;
    _index--;
    if (!_frameBuffer.getIndex(_index, _currentFrame))
    {
        return false;
    }
    putFrame(_currentFrame);
    return true;
}

bool cLogPlayback::seek(rtime t)
{
    // return FALSE in case of improper timestamp, either seeking too early or beyond EOF
    std::ostringstream ss;
    ss << "t=" << t.toStr();
    TRACE_FUNCTION(ss.str().c_str());
    if (t < _header.creation)
    {
        return false;
    }
    // normally given timestamp t is just a bit larger than _t
    // if not, we are apparently scrolling, in which case a linear search is way too expensive
    if ((t < _t) || (double(t - _t) > 0.6))
    {
        // TODO clear current RTDB state
        // seek backwards using binary search
        if (_frameBuffer.timeToIndex(t, _index))
        {
            if (_frameBuffer.getIndex(_index + 1, _nextFrame))
            {
                _frameBuffer.getIndex(_index, _currentFrame);
                putFrame(_currentFrame);
                return true;
            }
        }
        return false;
    }
    // invariant: both _currentFrame and _nextFrame are valid
    // normal iteration: handle a bunch of frames
    rtime tNext = _header.creation + _nextFrame.age;
    while (tNext < t)
    {
        if (!step())
        {
            return false;
        }
        tNext = _header.creation + _nextFrame.age;
    }
    return true;
}

void cLogPlayback::putFrame(tLogFrame const &frame)
{
    TRACE_FUNCTION("");
    // write current frame into RTDB
    rtime t = _header.creation + frame.age;
    if (_rtdb == NULL)
    {
        throw std::runtime_error("There is no RTDB connection");
    }
    TRACE("putFrame t=%.3f ts=%s size=%d", (double)t, t.toStr().c_str(), (int)frame.data.size());
    _rtdb->putFrameString(frame.data, true);
    // update timestamp administration
    _t = t;
}

