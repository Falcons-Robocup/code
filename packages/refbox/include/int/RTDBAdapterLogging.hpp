// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

