// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * actionResult.hpp
 *
 *  Created on: Dec 01, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef ACTIONRESULT_HPP_
#define ACTIONRESULT_HPP_

#include "RtDB2.h" // required for serialization

enum class actionResultTypeEnum
{
    INVALID,
    PASSED,
    FAILED, 
    RUNNING
};

SERIALIZE_ENUM(actionResultTypeEnum);

struct actionResult
{
    actionResultTypeEnum result;
    
    SERIALIZE_DATA_FIXED(result);
};

#endif

