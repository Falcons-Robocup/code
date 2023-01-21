// Copyright 2020-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Oct 3, 2020
 *      Author: Erik Kouters
 */


#include "ext/VelocityControlClient.hpp"
#include "tracing.hpp"

int main(int argc, char **argv)
{
    INIT_TRACE("velocityControl");

    VelocityControlClient client;
    client.spin();

    return 0;
}

