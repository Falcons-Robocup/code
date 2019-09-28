 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBAdapterLogging.hpp
 *
 *  Created on: Jun 14, 2019
 *      Author: Jan Feitsma
 */

#ifndef RTDBADAPTERLOGGING_HPP_
#define RTDBADAPTERLOGGING_HPP_

#include <string>
#include <vector>
#include "int/TCPIP_client.h" // TODO: need only Send function, so pass as callback?
#include "int/logger/cPacketRefboxLogger.hpp"
#include "cWorldModelClient.hpp"


class RTDBAdapterLogging
{
public:

    RTDBAdapterLogging();
    void update(CTCPIP_Client *);

private:

    cWorldModelClient _wmClient;

    // helper functions for update()
    void getActiveRobots(std::vector<uint8_t> &active_robots);
    void updateRobots(packetRefboxLogger::cPacketRefboxLogger &logPacket);
    void updateBall(packetRefboxLogger::cPacketRefboxLogger &logPacket);
    void updateObstacles(packetRefboxLogger::cPacketRefboxLogger &logPacket);

};

#endif

