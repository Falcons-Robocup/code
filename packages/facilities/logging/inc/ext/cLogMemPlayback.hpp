// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPlayback.hpp
 *
 * Load a log file and playback it contents, writing into RTDB.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#ifndef CPLAYBACK_HPP_
#define CPLAYBACK_HPP_


#include <string>
#include <utility>
#include <fstream>
#include "tLogHeader.hpp"
#include "tLogFrame.hpp"
#include "ext/cDbConnection.hpp"


class cPlayback : public cDbConnection
{
public:
    cPlayback(std::string const &filename);
    ~cPlayback();
    
    // run entire log on standard speed
    void run();
    bool step(); // perform a step, return success
    
    // detailed control, meant for visualizer
    void seek(rtime t); // seek to given timestamp
    void setDt(float dt); // modify step speed
    
private:
    std::string _filename;
    tLogHeader _header;
    tLogFrame _nextFrame;
    std::map<std::pair<int, bool>, tLogFrame> _frameQueue;
    std::ifstream _file;
    float _dt = 0.1;
    rtime _t;
    void clearFrameQueue();
    void queueFrame(tLogFrame const &frame);
    bool putFrameQueue();
    void putFrame(tLogFrame const &frame);
    void openFile();
    
};

#endif

