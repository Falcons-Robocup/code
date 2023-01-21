// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpRTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBOUTPUTADAPTER_HPP_
#define RTDBOUTPUTADAPTER_HPP_

#include <vector>

#include "FalconsRTDB.hpp"

class tpRTDBOutputAdapter
{
public:
    static tpRTDBOutputAdapter& getInstance()
    {
        static tpRTDBOutputAdapter instance;
        return instance;
    }

    void setActionData(const T_ACTION& actionData) const;
    void clearForbiddenAreas() const;
    void setForbiddenAreas(const T_FORBIDDEN_AREAS& forbiddenAreas) const;
    void setRoles(const T_ROBOT_ROLES& roles) const;
    void setIntention(const T_INTENTION& intention) const;
    void setLastUsedHeightmap(const T_HEIGHTMAP& heightmap) const;

private:
    tpRTDBOutputAdapter();
    RtDB2 *_rtdb;
    int _myRobotId;
};

#endif

