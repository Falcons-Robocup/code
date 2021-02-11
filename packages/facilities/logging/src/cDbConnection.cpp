// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDbConnection.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#include "falconsCommon.hpp" // MAX_ROBOTS

#include "ext/cDbConnection.hpp"


cDbConnection::cDbConnection(std::string const &storage)
{
    for (int i = 0; i <= MAX_ROBOTS; i++)
    {
        _agentIds.push_back(i);
    }

    connectRTDB(storage);
}

cDbConnection::~cDbConnection()
{
    disconnectRTDB();
}

void cDbConnection::connectRTDB(std::string const &storage)
{
    for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
    {
        _rtdb[*it] = RtDB2Store::getInstance().getRtDB2(*it, storage);
    }
}

void cDbConnection::disconnectRTDB()
{
    for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
    {
        // Do not close the RtDB connection
        //delete _rtdb[*it];
    }
}

