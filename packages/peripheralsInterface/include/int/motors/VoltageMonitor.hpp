// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VoltageMonitor.hpp
 *
 *  Created on: Apr 25, 2017
 *      Author: Jan Feitsma
 */

#ifndef VOLTAGEMONITOR_HPP_
#define VOLTAGEMONITOR_HPP_

#include <deque>

#define BUFFERSIZE 256
#define WARNING_THRESHOLD 23.0
#define ERROR_THRESHOLD 22.0
#define EVENT_TIMEOUT 30.0

class VoltageMonitor {

public:
    VoltageMonitor();
    ~VoltageMonitor();

    void feed(float voltage);
    float get();
    void check(); // better not call this while driving due to power dips
	
private:
    std::deque<float> _buffer;
    
};
#endif /* VOLTAGEMONITOR_HPP_ */
