// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDbConnection.hpp
 *
 * Interfacing with RTDB instances. Base class for cLogger and cPlayback.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#ifndef CDBCONNECTION_HPP_
#define CDBCONNECTION_HPP_


#include <vector>
#include <map>
#include "RtDB2Store.h"


#define RTDB2_STORAGE_PRODUCTION ("/tmp/rtdb2_storage")
#define RTDB2_STORAGE_PLAYBACK ("/tmp/rtdb2_playback")


class cDbConnection
{
  public:
    cDbConnection(std::string const &storage = RTDB2_STORAGE_PRODUCTION);
    ~cDbConnection();
    
  public:
    std::vector<int> _agentIds;
    std::map<int, RtDB2*> _rtdb; // key: agent id
    void connectRTDB(std::string const &storage);
    void disconnectRTDB();
    
};

#endif

