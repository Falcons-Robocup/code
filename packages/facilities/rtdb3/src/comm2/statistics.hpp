 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * statistics.hpp
 *
 *  Created on: 2018-12-09
 *      Author: Jan Feitsma
 */


#ifndef _INCLUDED_RTDB2_COMM2_STATISTICS_HPP_
#define _INCLUDED_RTDB2_COMM2_STATISTICS_HPP_


#include "../utils/rtime.hpp"


// TODO: serialize these structs to also store in RTDB / .rdl?


class TransmitStatistics
{
private:
    int _counter = 0;
    int _totalBytes = 0;
    rtime _lastReportTime = rtime::now();
    int _lastReportBytes = 0;

public:
    void update(int numBytes);
    std::string report();
};


class ReceiveStatistics
{
private:
    int _counter = 0;
    int _totalBytes = 0;
    int _packetsLost = 0;
    rtime _lastReportTime = rtime::now();
    int _lastReceivedId = 0;
    int _lastReportBytes = 0;
    int _lastReportLoss = 0;

public:
    void update(int numBytes, int packetId);
    std::string report();
};


struct Statistics
{
    TransmitStatistics transmit;
    std::map<int, ReceiveStatistics> receive;
    
    void display();
};

#endif

