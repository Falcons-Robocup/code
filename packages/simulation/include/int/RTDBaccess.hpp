// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBaccess.hpp
 *
 *  Created on: Feb 13, 2019
 *      Author: Coen Tempelaars
 */

#ifndef RTDBACCESS_HPP_
#define RTDBACCESS_HPP_

#include <map>

#include "FalconsRTDB.hpp"
#include "robot.hpp"
#include "teamID.hpp"


FalconsRTDB* getRTDBConnection();
FalconsRTDB* getRTDBConnection (const TeamID&);
FalconsRTDB* getRTDBConnection (const TeamID&, const RobotID&);
int getRobotNumber (const RobotID&);


#endif /* RTDBACCESS_HPP_ */
