// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_FALCONSCOMMONENV_HPP_
#define _INCLUDED_FALCONSCOMMONENV_HPP_

// Main facilities for robot numbers
bool isSimulatedEnvironment();
int getRobotNumber(); // 0=coach, 1+ agent id
bool isGoalKeeper();
char getTeamChar(); // A or B

#endif

