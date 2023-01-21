// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpOverrideState.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#ifndef TPOVERRIDESTATE_HPP_
#define TPOVERRIDESTATE_HPP_

#include "RtDB2.h"

struct tpOverrideState
{
    bool                     active;
    std::string              overrideStr;
    
    SERIALIZE_DATA(active, overrideStr);
};


#endif

