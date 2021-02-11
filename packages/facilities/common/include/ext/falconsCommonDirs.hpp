// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_FALCONSCOMMONDIRS_HPP_
#define _INCLUDED_FALCONSCOMMONDIRS_HPP_

#include <string>

// these functions basically make the associated environment variables available to code clients

std::string pathToFalconsRoot();      // typically /home/robocup/falcons
std::string pathToCodeRepo();         // typically /home/robocup/falcons/code
std::string pathToConfig();           // typically /home/robocup/falcons/code/config
std::string pathToScripts();          // typically /home/robocup/falcons/code/scripts
std::string pathToTeamplayDataRepo(); // typically /home/robocup/falcons/teamplayData
std::string pathToDataRepo();         // typically /home/robocup/falcons/data

#endif

