// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "ext/falconsCommonDirs.hpp"

#include <string>
#include <stdlib.h>
#include <iostream>

// internal helper function: get environment variable or error
std::string getEnvWrapper(std::string key);

std::string pathToFalconsRoot()
{
    // typically /home/robocup/falcons
    return getEnvWrapper("FALCONS_PATH");
}

std::string pathToCodeRepo()
{
    // typically /home/robocup/falcons/code
    return getEnvWrapper("FALCONS_CODE_PATH");
}

std::string pathToConfig()
{
    // typically /home/robocup/falcons/code/config
    return getEnvWrapper("FALCONS_CODE_PATH") + "/config";
}

std::string pathToScripts()
{
    // typically /home/robocup/falcons/code/scripts
    return getEnvWrapper("FALCONS_SCRIPTS_PATH");
}

std::string pathToTeamplayDataRepo()
{
    // typically /home/robocup/falcons/teamplayData
    return getEnvWrapper("FALCONS_TPDATA_PATH");
}

std::string pathToDataRepo()
{
    // typically /home/robocup/falcons/data
    return getEnvWrapper("FALCONS_DATA_PATH");
}


// internal helper function: get environment variable or error
std::string getEnvWrapper(std::string key)
{
    char *cp = NULL;
    if ((cp = getenv(key.c_str())) == NULL)
    {
        std::cerr << "ERROR: missing environment variable: " + key << std::endl;
        return "ERROR";
    }

    return cp;
}

