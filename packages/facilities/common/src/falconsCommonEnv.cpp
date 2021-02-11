// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "ext/falconsCommonEnv.hpp"

#include <string>
#include <stdlib.h>

bool isSimulatedEnvironment()
{
    bool isSimulated = false;
    std::string env_simulated;
    try
    {
        char* env_sim = getenv("SIMULATED");
        env_simulated = std::string(env_sim);
    }
    catch (...)
    {
        //std::cout << "Environment variable SIMULATED not found." << std::endl; // suppress spam
        // this is an error state, suggesting something weird in scripting or stand-alone deployment
        // hence we SET the simulated flag, so at least traffic will remain local
        isSimulated = true;
    }

    if (env_simulated == "1")
    {
        isSimulated = true;
    }

    return isSimulated;
}

int getRobotNumber()
{
    int robotNumber = 0;

    if (getenv("TURTLE5K_ROBOTNUMBER") != NULL)
    {
        robotNumber = atoi(getenv("TURTLE5K_ROBOTNUMBER"));
    }

    return robotNumber;
}

bool isGoalKeeper()
{
    // TODO this should not depend on robotNumber; what if r2 decides to behave as goalKeeper?
    // definition is vague anyway --
    // are we interested in hardware aspects (do we have a frame, then let's ignore some fake obstacles?)
    // or in software / behavior (behave as goalKeeper?)
    return (getRobotNumber() == 1);
}

char getTeamChar() // A or B
{
    char teamChar = 'A';

    if (getenv("TURTLE5K_TEAMNAME") != NULL)
    {
        if (getenv("TURTLE5K_TEAMNAME") == std::string("teamB"))
        {
            teamChar = 'B';
        }
    }

    return teamChar;
}

