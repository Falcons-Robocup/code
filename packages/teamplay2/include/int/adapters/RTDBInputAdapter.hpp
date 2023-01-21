// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpRTDBInputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBINPUTADAPTER_HPP_
#define RTDBINPUTADAPTER_HPP_

#include <vector>

#include "FalconsRTDB.hpp"
#include "cWorldModelClient.hpp"
#include "MTPAdapter.hpp"

#include "int/types/Robot.hpp"
#include "int/types/Role.hpp"

class tpRTDBInputAdapter
{
public:
    static tpRTDBInputAdapter& getInstance()
    {
        static tpRTDBInputAdapter instance;
        return instance;
    }

    void waitForTPHeartbeat(void (*iterateTeamplayFuncPtr) (void)) const;
    void waitForRobotRoles(void (*iterateTeamplayFuncPtr) (void)) const;
    void getWorldModelData();
    void getWorldModelData(const int robotID);
    void setWorldModelRobotId(const int robotID);
    T_ACTION_RESULT getActionResult() const;
    T_INTENTION getIntention(const int robotID) const;
    std::map<teamplay::RobotNumber,teamplay::RoleEnum> getRobotRoles(const int leaderRobotID) const;
    void setMTP(MixedTeamProtocolAdapter* mtp);

private:
    tpRTDBInputAdapter();

    int _myRobotId;
    RtDB2 *_rtdb;
    MixedTeamProtocolAdapter* _mtpAdapter;
    cWorldModelClient _wmClient;
};

#endif

