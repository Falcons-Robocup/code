// Copyright 2018-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBINPUTADAPTER_HPP_
#define CRTDBINPUTADAPTER_HPP_

#include <vector>

#include "FalconsRTDB.hpp"
#include "cWorldModelClient.hpp"
#include "int/types/cDecisionTreeTypes.hpp"

class cRTDBInputAdapter
{
public:
    static cRTDBInputAdapter& getInstance()
    {
        static cRTDBInputAdapter instance;
        return instance;
    }

    void waitForRobotState(void (*iterateTeamplayFuncPtr) (void));
    void getWorldModelData();
    void getWorldModelData(const int robotID);
    std::string getRole(const int robotID);
    T_INTENTION getIntention(const int robotID);

private:
    cRTDBInputAdapter();
    int _myRobotId;
    RtDB2 *_rtdb;
    cWorldModelClient _wmClient;
};

#endif

