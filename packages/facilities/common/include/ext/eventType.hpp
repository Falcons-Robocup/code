// Copyright 2016 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * File: eventType.hpp
 * Author: Jan Feitsma
 * Structure which holds relevant attributes of an event.
 * Used for tracing and diagnostics.
 */

#ifndef _INCLUDED_EVENTTYPE_HPP_
#define _INCLUDED_EVENTTYPE_HPP_


#include <string>


namespace falcons
{
    struct eventType
    {
        std::string fileName;
        std::string funcName;
        int         lineNumber;
        double      timeStamp; // filled in by constructor
        std::string message; // needs to be set by client
        // constructor
        eventType(std::string const &file, int line, std::string const &func);
    };
}

#endif // _INCLUDED_EVENTTYPE_HPP_


