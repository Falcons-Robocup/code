// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogPlayback.hpp
 *
 * Base class for playback. Intended to split in-memory use case (live visualization) from post-mortem file-based playback.
 *
 *  Created on: Sep 2, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGPLAYBACK_HPP_
#define CLOGPLAYBACK_HPP_


#include <utility>
#include "tLogHeader.hpp"
#include "tLogFrame.hpp"
#include "cFrameBuffer.hpp"
#include "cDbConnection.hpp"


class cLogPlayback
{
public:
    cLogPlayback(int agentId);
    virtual ~cLogPlayback() {};

    tLogHeader getHeader();
    float getDuration();
    bool step(); // process a single frame, return success
    bool stepBack();
    bool seek(rtime t); // advance/jump to given timestamp
    
protected:
    RtDB2* _rtdb = NULL;
    tLogHeader _header; // header of this log
    rtime _t; // current timestamp
    tLogFrame _currentFrame;
    tLogFrame _nextFrame;
    cFrameBuffer _frameBuffer; // fast random access, chronologically-ordered
    size_t _index;
    // is required for stepping through time, handling frames until nextFrame is too new
    void putFrame(tLogFrame const &frame);
};

#endif

