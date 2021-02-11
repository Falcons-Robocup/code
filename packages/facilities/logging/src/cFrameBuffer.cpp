// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cFrameBuffer.cpp
 *
 */


#include "ext/cFrameBuffer.hpp"

#include "tracing.hpp"

cFrameBuffer::cFrameBuffer()
{
    _bytes = 0;
}

cFrameBuffer::~cFrameBuffer()
{
}

void cFrameBuffer::insert(rtime const &t, tLogFrame const &frame)
{
    _indexHash[t] = _array.size();
    _array.push_back(frame);
    // extra, for tracing only ... might be a bit expensive:
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, frame);
    _bytes += sbuf.size();
}

bool cFrameBuffer::getIndex(size_t idx, tLogFrame &frame)
{
    if (idx < _array.size())
    {
        frame = _array[idx];
        return true;
    }
    return false;
}

size_t cFrameBuffer::getSize()
{
    return _array.size();
}

bool cFrameBuffer::timeToIndex(rtime const &t, size_t &idx)
{
    TRACE_FUNCTION("");
    auto it = _indexHash.upper_bound(t);
    if (it == _indexHash.end())
    {
        return false;
    }
    if (it != _indexHash.begin())
    {
        --it;
    }
    idx = it->second;
    TRACE("t=%s idx=%d", t.toStr().c_str(), idx);
    return true;
}

void cFrameBuffer::traceStats()
{
    float duration = _array.rbegin()->age;
    size_t numFrames = _array.size();
    TRACE("indexed %d frames, avg %.1f frames/sec, total size %.2fMB", numFrames, numFrames / duration, _bytes / 1024.0 / 1024.0);
}

