// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * pathPlanningTestDefaults.hpp
 *
 *  Created on: November 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_TESTDEFAULTS_HPP_
#define PATHPLANNING_TESTDEFAULTS_HPP_

#include "int/PathPlanning.hpp"


// load config from a yaml file
ConfigPathPlanning loadYAML(std::string const &yamlfilename);

// common setup and some config values (we normally don't want to be sensitive to production yamls)
PathPlanning pathPlanningSetup(ppCFI *ppci, exCFI *exci, OutputInterface *output = NULL);
PathPlanning defaultPathPlanningSetup();

// but for some tests we do want to check yaml values
PathPlanning yamlPathPlanningSetup(std::string const &yamlfilename, OutputInterface *output = NULL);
PathPlanning ConfigPathPlanningSetup(ConfigPathPlanning const &config, OutputInterface *output = NULL);


#endif
