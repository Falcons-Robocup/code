// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cFrameBuffer.hpp
 *
 * Store a log in memory.
 *
 *  Created on: Oct 27, 2018
 *      Author: Jan Feitsma
 */

#ifndef CFRAMEBUFFER_HPP_
#define CFRAMEBUFFER_HPP_

#include <map>
#include <vector>
#include "tLogFrame.hpp"


class cFrameBuffer
{
public:
    cFrameBuffer();
    ~cFrameBuffer();
    
    void insert(rtime const &t, tLogFrame const &frame);
    bool getIndex(size_t idx, tLogFrame &frame);
    size_t getSize();
    bool timeToIndex(rtime const &t, size_t &idx);

    void traceStats();

private:
    std::map<rtime, size_t> _indexHash; // a hash map from time to index
    size_t _bytes;
    std::vector<tLogFrame> _array;
    
};

#endif

