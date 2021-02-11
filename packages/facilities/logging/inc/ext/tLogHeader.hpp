// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tLogHeader.hpp
 */

#ifndef TLOGHEADER_HPP_
#define TLOGHEADER_HPP_

#include "RtDB2.h" // for serialization

struct tLogHeader
{
    std::string hostname;
    rtime       creation;
    bool        compression;
    float       duration; // Deprecated. Do not use.
    std::string filename;
    int         frequency;
    
    SERIALIZE_DATA_FIXED(hostname, creation, compression, duration, filename, frequency);
};

#endif

