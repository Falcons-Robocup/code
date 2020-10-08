 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * comm.hpp
 *
 *  Created on: 2018-12-08
 *      Author: Jan Feitsma
 */


#ifndef _INCLUDED_RTDB2_COMM_HPP_
#define _INCLUDED_RTDB2_COMM_HPP_

#include <boost/thread/thread.hpp>

#include "../../rtdb2/RtDB2Store.h"
#include "MulticastSocket.h"
#include "timer.hpp"
#include "statistics.hpp"
#include "frameheader.hpp"


#define COMM_BUFFER_SIZE 65536

class Comm
{
public:
    Comm();
    ~Comm();
    
    void run();

    // public properties, overridable until initialize() is called
    int                   agentId;
    std::string           dbPath = RTDB2_DEFAULT_PATH;
    CommunicationSettings settings;
    
private:
    bool                  _initialized = false;
    boost::thread         _receiverThread;
    boost::thread         _transmitterThread;
    boost::thread         _diagnosticsThread;
    MulticastSocket       _socket;
    Statistics            _statistics;
    RtDB2                *_rtdb = NULL;
    int                   _counter = 0;
    
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

