// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPlayback.cpp
 *
 */

#include <boost/bind.hpp>
#include "ext/cPlayback.hpp"


cPlayback::cPlayback(std::string const &filename)
{
    _filename = filename;
    openFile();
}

cPlayback::~cPlayback()
{
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

void cPlayback::run()
{
    rtime::loop(1.0 / _dt, boost::bind(&cPlayback::step, this));
}

bool cPlayback::step()
{
    if (!_file.good())
    {
        return false;
    }
    // a sequence of frames will be queued, of which the latest ones per agent will actually be put
    // (to prevent spamming agent databases consecutively with outdated data)
    clearFrameQueue();
    // invariant: nextFrame is valid and it timestamp is larger than _t
    rtime tNext = _header.creation + _nextFrame.age;
    rtime tLim = _t + _dt;
    while (tNext < tLim)
    {
        // queue frame
        queueFrame(_nextFrame);
        // read next
        bool ok = readRaw(_nextFrame, _file);
        //printf("ok=%d t=%.3f tNext=%.3f\n", ok, (double)_t, (double)tNext); fflush(stdout);
        if (!ok) break;
        tNext = _header.creation + _nextFrame.age;
    }
    // now nextFrame points to right after timesatmp, so we can store all queued frames
    // and advance timestamp
    _t += _dt;
    putFrameQueue(); // when oversampling, we might have empty gaps
    return true; // so always return true, leave it to _file.good()
}

void cPlayback::seek(rtime t)
{
    // TODO visualizer use case
    //selectFrame(t);
    //putFrame();
}

void cPlayback::setDt(float dt)
{
    _dt = dt;
}

void cPlayback::clearFrameQueue()
{
    _frameQueue.clear();
}

void cPlayback::queueFrame(tLogFrame const &frame)
{
    _frameQueue[std::pair<int, bool>(frame.agent, frame.shared)] = frame;
}

bool cPlayback::putFrameQueue()
{
    for (auto it = _frameQueue.begin(); it != _frameQueue.end(); ++it)
    {
        putFrame(it->second);
    }
    return (_frameQueue.size() > 0);
}

void cPlayback::putFrame(tLogFrame const &frame)
{
    // write current frame into RTDB
    rtime t = _header.creation + frame.age;
    int agentId = frame.agent;
    if (!_rtdb.count(agentId))
    {
        throw std::runtime_error("There is no RTDB connection to agent " + std::to_string(agentId));
    }
    bool is_compressed = _header.compression;
    int life = 0;
    printf("putbatch agent=%d t=%.3f ts=%s shared=%d size=%d\n", agentId, (float)t, t.toStr().c_str(), (int)frame.shared, (int)frame.data.size()); fflush(stdout);
    _rtdb[agentId]->put_batch(agentId, frame.data, life, frame.shared, is_compressed);
}

void cPlayback::openFile()
{
    _file.open(_filename, std::ios::binary);
    if (!_file.is_open())
    {
        throw std::runtime_error("Failed to open log file " + _filename);
    }
    // read header into buffer
    if (!readRaw(_header, _file))
    {
        throw std::runtime_error("Failed to read header from log file " + _filename);
    }
    // set current timestamp
    _t = _header.creation;
    _dt = 1.0 / _header.frequency;
    // read first frame
    readRaw(_nextFrame, _file);
}

