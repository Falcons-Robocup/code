// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpOverrideResult.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#ifndef TPOVERRIDERESULT_HPP_
#define TPOVERRIDERESULT_HPP_

#include "RtDB2.h"

#include "behTreeReturnEnum.hpp"

struct tpOverrideResult
{
    behTreeReturnEnum        status;
    
    SERIALIZE_DATA(status);
};


#endif

