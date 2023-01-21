// Copyright 2020-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "ext/falconsCommonDirs.hpp"
#include "ext/falconsCommonEnv.hpp"
#include "ext/falconsCommonConfig.hpp"

#include <string>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

// key[Sim][Teamb][Rx].yaml
// Options:
//   PathPlanning.yaml
//   PathPlanningR1.yaml
//   PathPlanningTeamb.yaml
//   PathPlanningTeambR1.yaml
//   PathPlanningSim.yaml
//   PathPlanningSimR1.yaml
//   PathPlanningSimTeamb.yaml
//   PathPlanningSimTeambR1.yaml
std::string determineConfig(std::string key, std::string fileExtension)
{

    // determine config folder
    std::string cfgFolder = pathToConfig();

    // determine yaml file to load

    // PathPlanning.yaml
    std::string target = key;

    if (isSimulatedEnvironment()) // simulator override?
    {
        std::string candidate = key + "Sim";
        std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            // PathPlanningSim.yaml
            target = candidate;
        }
        // test robot-specific
        candidate = key + "SimR" + boost::lexical_cast<std::string>(getRobotNumber());
        candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            // PathPlanningSimR1.yaml
            target = candidate;
        }

        // TeamB overrules
        if (getTeamChar() == 'B')
        {
            std::string candidate = key + "SimTeamb";
            std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
            if (boost::filesystem::exists(candidateFull))
            {
                // PathPlanningSimTeamb.yaml
                target = candidate;
            }
            // test robot-specific
            candidate = key + "SimTeambR" + boost::lexical_cast<std::string>(getRobotNumber());
            candidateFull = cfgFolder + "/" + candidate + fileExtension;
            if (boost::filesystem::exists(candidateFull))
            {
                // PathPlanningSimTeambR1.yaml
                target = candidate;
            }
        }

    }
    else // not simulator
    {
        // test robot-specific
        std::string candidate = key + "R" + boost::lexical_cast<std::string>(getRobotNumber());
        std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            // PathPlanningR1.yaml
            target = candidate;
        }

        // TeamB overrules
        if (getTeamChar() == 'B')
        {
            std::string candidate = key + "Teamb";
            std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
            if (boost::filesystem::exists(candidateFull))
            {
                // PathPlanningTeamb.yaml
                target = candidate;
            }
            // test robot-specific
            candidate = key + "TeambR" + boost::lexical_cast<std::string>(getRobotNumber());
            candidateFull = cfgFolder + "/" + candidate + fileExtension;
            if (boost::filesystem::exists(candidateFull))
            {
                // PathPlanningTeambR1.yaml
                target = candidate;
            }
        }
    }
    // final check and return
    std::string result = cfgFolder + "/" + target + fileExtension;
    if (!boost::filesystem::exists(result))
    {
        printf("ERROR: could not resolve yaml file for %s\n", key.c_str());
    }
    else
    {
        printf("resolved yaml file %s -> %s\n", key.c_str(), result.c_str());
    }
    return result;
}

