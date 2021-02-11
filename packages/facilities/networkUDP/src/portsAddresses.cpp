// Copyright 2016-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * portsAddresses.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: Jan Feitsma
 */

#include "ext/ports.hpp"
#include "ext/addresses.hpp"
#include "falconsCommon.hpp"



namespace Facilities
{
namespace Network
{

std::string getMulticastAddress()
{
    return "224.16.32.74";
}


int getPort(portEnum key, int robotNumber)
{
    int result = 0;
    if (getTeamChar() == 'A')
    {
        result += PORT_OFFSET_TEAM_A;
    }
    else
    {
        result += PORT_OFFSET_TEAM_B;
    }
    result += PORT_OFFSET_PER_ROBOT * robotNumber;
    result += key;
    if (isSimulatedEnvironment())
    {
        result += PORT_OFFSET_SIMULATOR;
    }
    return result;
}

}

}

