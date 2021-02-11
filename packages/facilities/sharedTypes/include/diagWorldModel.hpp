// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagWorldModel.hpp
 *
 */

#ifndef DIAGWORLDMODEL_HPP_
#define DIAGWORLDMODEL_HPP_

#include "diagWorldModelShared.hpp"
#include "diagWorldModelLocal.hpp"

struct diagWorldModel
{
    diagWorldModelLocal  local;
    diagWorldModelShared shared;
    // no serialization -- items are stored and handled separately in RTDB
};

#endif

