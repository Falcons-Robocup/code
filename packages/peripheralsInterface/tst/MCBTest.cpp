// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * MCBTest.cpp
 *
 *  Created on: Mar 30, 2019
 *      Author: Erik Kouters
 */

#include <thread>
#include <iostream>

#include "int/MCB.hpp"

int main()
{
    MCB mcb;
    while (true)
    {
        MCB_feedback_data data = mcb.readPublishedData();
        std::cout << "robotVel: (" << data.robotVel.x << ", " << data.robotVel.y << ", " << data.robotVel.Rz << ")" << std::endl;
        std::cout << "robotDisp: (" << data.robotDisp.x << ", " << data.robotDisp.y << ", " << data.robotDisp.Rz << ")" << std::endl;
    }

    return 0;
}
