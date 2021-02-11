// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * teamID.cpp
 *
 *  Created on: Apr 29, 2019
 *      Author: Coen Tempelaars
 */

#include "int/teamID.hpp"

TeamID otherTeam (const TeamID teamID)
{
    if (teamID == TeamID::A)
    {
        return TeamID::B;
    }
    else
    {
        return TeamID::A;
    }
}
