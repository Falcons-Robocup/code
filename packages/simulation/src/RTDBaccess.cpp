// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBaccess.cpp
 *
 *  Created on: Feb 13, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBaccess.hpp"

#include <map>
#include "falconsCommon.hpp"


// simulation context switching
#define RTDB2_SIM_TEAM_A_PATH       (RTDB_STORAGE_TEAMS + std::string("A"))
#define RTDB2_SIM_TEAM_B_PATH       (RTDB_STORAGE_TEAMS + std::string("B"))


static const std::map<RobotID, int> ROBOT_NUMBERS =
{
    {RobotID::r1, 1},
    {RobotID::r2, 2},
    {RobotID::r3, 3},
    {RobotID::r4, 4},
    {RobotID::r5, 5}
};

static const std::map<TeamID, std::string> PATHS =
{
        {TeamID::A, RTDB2_SIM_TEAM_A_PATH},
        {TeamID::B, RTDB2_SIM_TEAM_B_PATH}
};

FalconsRTDB* getRTDBConnection()
{
    return FalconsRTDBStore::getInstance().getFalconsRTDB(COACH_AGENTID);
}

FalconsRTDB* getRTDBConnection (const TeamID& teamID)
{
    auto path = PATHS.at(teamID);
    return FalconsRTDBStore::getInstance().getFalconsRTDB(COACH_AGENTID, path);
}

FalconsRTDB* getRTDBConnection (const TeamID& teamID, const RobotID& robotID)
{
    auto robotNumber = ROBOT_NUMBERS.at(robotID);
    auto path = PATHS.at(teamID);
    return FalconsRTDBStore::getInstance().getFalconsRTDB(robotNumber, path);
}

int getRobotNumber (const RobotID& robotID)
{
    return ROBOT_NUMBERS.at(robotID);
}
