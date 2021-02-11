// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * LogEvent.h
 *
 *  Created on: January 4th, 2017
 *      Author: Jan Feitsma
 */

#ifndef LOGEVENT_H
#define LOGEVENT_H

#include "int/types/LogLevel.h"
#include <string>


struct LogEvent
{
    int              robotId;
    std::string      fileName;
    std::string      funcName;
    int              lineNumber;
    double           timeStamp;
    LogLevel         type;
    std::string      message;
    LogEvent(std::string msg = "")
    {
        message = msg;
        type = INFO;
        robotId = 0;
        lineNumber = 0;
        timeStamp = 0;
    }
};

#endif // LOGEVENT_H

