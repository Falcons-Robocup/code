// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanningClient.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNINGCLIENT_HPP_
#define PATHPLANNINGCLIENT_HPP_


#include "actionResult.hpp" // sharedTypes


class PathPlanningClient
{
public:
    PathPlanningClient();
    ~PathPlanningClient();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    // * return status as actionResultTypeEnum
    actionResultTypeEnum iterate();

    // legacy spinner, which was used when PathPlanning had its own process,
    // before being integrated into motionPlanning as library
    void spin();

private:
};

#endif

