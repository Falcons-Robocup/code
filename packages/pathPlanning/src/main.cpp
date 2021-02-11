// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */


#include "ext/PathPlanningClient.hpp"

int main(int argc, char **argv)
{

    PathPlanningClient client;
    client.spin();

    return 0;
}

