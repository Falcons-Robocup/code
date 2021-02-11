// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGPERIPHERALSINTERFACEKICKER_HPP_
#define CONFIGPERIPHERALSINTERFACEKICKER_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigPeripheralsInterfaceKicker
{

    int leverMaxHeight = 205;
    int leverSpeed = 50;

    SERIALIZE_DATA(leverMaxHeight, leverSpeed);
};

#endif

