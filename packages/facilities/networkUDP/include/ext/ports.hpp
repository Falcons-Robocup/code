// Copyright 2015-2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 *  Created on: Dec 21, 2015
 *      Author: Jan Feitsma
 */

#ifndef NETWORK_PORTS_HPP_
#define NETWORK_PORTS_HPP_

#include <string>

namespace Facilities
{
namespace Network
{

static const int PORT_OFFSET_TEAM_A    = 10000;
static const int PORT_OFFSET_TEAM_B    = 20000;
static const int PORT_OFFSET_COACH     =     0;
static const int PORT_OFFSET_PER_ROBOT =  1000;
static const int PORT_OFFSET_SIMULATOR =   500;
// DANGER: do NOT change these values unless you also change udpInterface.py


// each robot has 1000 consecutive ports available
// which are used according to following enum
enum portEnum
{ 
    PORT_INVALID = 0,
    PORT_ROBOTCONTROL = 1,   // DANGER: do NOT change this particular value unless you also change udpInterface.py
    PORT_REFBOXRELAY,
    PORT_WORLDMODELSYNC,
    PORT_WORLDMODELSYNC_STD,
    PORT_TEAMPLAY_INTENTION,
    // insert new ports here
    PORT_DIAG_BASE
    // all ports below PORT_DIAG_BASE are reserved for diagnostics!
    // so do NOT add new ports here!
};


// wrapper to apply team- and robot offsets
// and return a unique port
int getPort(portEnum key, int robotNumber);



} /* namespace Network */
} /* namespace Facilities */

#endif /* NETWORK_PORTS_HPP_ */
