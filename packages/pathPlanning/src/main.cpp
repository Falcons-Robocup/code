// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */


#include "ext/PathPlanningClient.hpp"
#include "tracing.hpp"

int main(int argc, char **argv)
{
    INIT_TRACE("pathPlanning");

    PathPlanningClient client;
    client.spin();

    return 0;
}
