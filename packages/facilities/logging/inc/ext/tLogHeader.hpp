// Copyright 2018-2022 Jan Feitsma (Falcons)
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
    std::string commit_code;
    std::string commit_teamplay_data;

    
    SERIALIZE_DATA_FIXED(hostname, creation, compression, duration, filename, frequency, commit_code, commit_teamplay_data);
};

#endif

