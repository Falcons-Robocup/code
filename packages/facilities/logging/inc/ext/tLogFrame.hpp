// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tLogFrame.hpp
 */

#ifndef TLOGFRAME_HPP_
#define TLOGFRAME_HPP_

#include "RtDB2.h" // for serialization

struct tLogFrame
{
    float       age; // in seconds, since start of log
    std::string data; // serialized (possibly compressed) RTDBFrame
    
    SERIALIZE_DATA_FIXED(age, data);
};

#endif

