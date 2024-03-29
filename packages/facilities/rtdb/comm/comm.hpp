// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * comm.hpp
 *
 *  Created on: 2018-12-08
 *      Author: Jan Feitsma
 */


#ifndef _INCLUDED_RTDB2_COMM_HPP_
#define _INCLUDED_RTDB2_COMM_HPP_

#include <boost/thread/thread.hpp>

#include "RtDB2Context.h"
#include "RtDB2Store.h"
#include "MulticastSocket.h"
#include "timer.hpp"
#include "statistics.hpp"
#include "frameheader.hpp"


#define COMM_BUFFER_SIZE 65536

class Comm
{
public:
    Comm(RtDB2Context const &context);
    ~Comm();
    
    boost::thread start() {
        return boost::thread(&Comm::run, this);
    }
    static void shutdown();

    // public properties, overridable until initialize() is called
    RtDB2Context          context_;
    int                   agentId;
    CommunicationSettings settings;
    
private:
    bool                  _initialized = false;
    MulticastSocket       _socket;
    Statistics            _statistics;
    RtDB2                *_rtdb = NULL;
    int                   _counter = 0;
    
    void run();

    // singular operations
    void receive();
    void transmit();
    
    // loops / threads
    void receiver();
    void transmitter();
    void diagnostics();
    
    // miscellaneous
    void initialize();
    static void sigHandler(int sig);
    void printSettings();
    void removeHeader(std::string &buffer, FrameHeader &header);
    void makeHeader(FrameHeader &header, int counter);
    void insertHeader(std::string &buffer, FrameHeader const &header);
    
};

#endif

