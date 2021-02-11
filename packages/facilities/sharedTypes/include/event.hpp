// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * event.hpp
 *
 *  Created on: Dec 3, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef EVENT_HPP_
#define EVENT_HPP_

#include "RtDB2.h" // required for serialization


enum class eventSeverityEnum
{
    INFO,
    WARNING,
    ERROR
};

SERIALIZE_ENUM(eventSeverityEnum);


struct event
{
    eventSeverityEnum  severity;
    std::string        fileName;
    std::string        funcName;
    int                lineNumber;
    rtime              timeStamp;
    std::string        message;
    
    SERIALIZE_DATA(severity, fileName, funcName, lineNumber, timeStamp, message);
    // sparse data, OK to not use SERIALIZE_DATA_FIXED
};

#endif

