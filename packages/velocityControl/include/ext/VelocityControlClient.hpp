// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocityControlClient.hpp
 *
 *  Created on: Oct, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROLCLIENT_HPP_
#define VELOCITYCONTROLCLIENT_HPP_


class VelocityControlClient
{
public:
    VelocityControlClient();
    ~VelocityControlClient();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    void iterate();

    void spin();

private:
};

#endif

