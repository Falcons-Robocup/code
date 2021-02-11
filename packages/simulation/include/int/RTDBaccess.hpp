// Copyright 2019 Coen Tempelaars (Falcons)
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

#include "RtDB2Store.h"
#include "robot.hpp"
#include "teamID.hpp"


RtDB2* getRTDBConnection();
RtDB2* getRTDBConnection (const TeamID&);
RtDB2* getRTDBConnection (const TeamID&, const RobotID&);
int getRobotNumber (const RobotID&);


#endif /* RTDBACCESS_HPP_ */
