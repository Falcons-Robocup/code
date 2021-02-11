// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBOUTPUTADAPTER_HPP_
#define CRTDBOUTPUTADAPTER_HPP_

#include <vector>

#include "FalconsRtDB2.hpp"
#include "cMotionPlanningClient.hpp"

class cRTDBOutputAdapter
{
public:
    static cRTDBOutputAdapter& getInstance()
    {
        static cRTDBOutputAdapter instance;
        return instance;
    }

    cMotionPlanningClient& getMPClient();
    void setActionData(const T_ACTION& actionData);
    void clearForbiddenAreas();
    void setForbiddenAreas(const T_FORBIDDEN_AREAS& forbiddenAreas);
    void setRole(const T_ROBOT_ROLE& role);
    void setIntention(const T_INTENTION& intention);

private:
    cRTDBOutputAdapter();
    RtDB2 *_rtdb;
    int _myRobotId;
    cMotionPlanningClient _mpClient;
};

#endif

