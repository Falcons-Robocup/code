// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * refboxOverride.hpp
 *
 *  Created on: Feb 2020
 *      Author: Jan Feitsma
 */

#ifndef REFBOXOVERRIDE_HPP_
#define REFBOXOVERRIDE_HPP_

#include "RtDB2.h" // required for serialization

struct refboxOverride
{
    std::string command;
    // ROADMAP: option to temporily block normal refbox, so we can take over from coach?

    SERIALIZE_DATA(command);
};

#endif

