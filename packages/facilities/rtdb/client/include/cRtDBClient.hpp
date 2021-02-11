// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRtDBClient.hpp
 *
 * RtDB client facility, intended for consumer interfaces offered by the data producers.
 * e.g., WorldModel offers the cWorldModelClient interface to all consumers that need WM data (balls, obstacles, etc).
 * cWorldModelClient inherits the class cRtDBClient.
 *
 *  Created on: Oct 20, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBCLIENT_HPP_
#define CRTDBCLIENT_HPP_

// system includes
#include <vector>
#include <map>

#include "falconsCommon.hpp" // getRobotNumber()
#include "RtDB2Store.h"

class cRtDBClient
{
public:

    int _myRobotId;
    std::vector<int> _agentIds;
    std::map<int, RtDB2*> _rtdb; // key: agent id

    cRtDBClient()
    {
        for (int i = 0; i <= MAX_ROBOTS; i++)
        {
            _agentIds.push_back(i);
        }
        _myRobotId = getRobotNumber();
        connectRTDB();
    }

    ~cRtDBClient()
    {
        disconnectRTDB();
    }

    void connectRTDB()
    {
        auto teamChar = getTeamChar();
        for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
        {
            _rtdb[*it] = RtDB2Store::getInstance().getRtDB2(*it, teamChar);
        }
    }

    void disconnectRTDB()
    {
        for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
        {
            // Do not close the RtDB connection
            //delete _rtdb[*it];
        }
    }
};

#endif //CRTDBCLIENT_HPP_

