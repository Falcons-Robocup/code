// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDiagnostics.cpp
 *
 *    Created on: Dec 3, 2018
 *        Author: Jan Feitsma
 */

#include "ext/cDiagnostics.hpp"
#include <cstdarg>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include "falconsCommon.hpp"
#include "tracing.hpp"

using namespace diagnostics;

EventHandler::EventHandler(eventSeverityEnum severity, const char* fileName, const int lineNr, const char* functionName)
{
    _event.timeStamp = ftime::now();
    _event.severity = severity;
    _event.fileName = fileName;
    _event.lineNumber = lineNr;
    _event.funcName = functionName;
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, getTeamChar());
}

EventHandler::~EventHandler()
{
}

EventHandler& EventHandler::operator()(const char* format, ...)
{
    // Read variadic arguments and parse with vsnprintf into buffer
    const int buffer_size = 4092;
    char buffer[buffer_size];
    va_list argptr;
    va_start(argptr, format);
    if (vsnprintf(buffer, buffer_size-1, format, argptr) < 0)
    {
        buffer[buffer_size-1] = 0;
    }
    va_end(argptr);

    _event.message = std::string(buffer);
    dispatch();

    return *this;
}

void EventHandler::dispatch()
{
    // trace
    std::string severityString = "INFO";
    if (_event.severity == eventSeverityEnum::WARNING)
    {
        severityString = "WARNING";
    }
    if (_event.severity == eventSeverityEnum::ERROR)
    {
        severityString = "ERROR";
    }
    TRACE("timestamp=%s severity=%s file=%s line=%d function=%s event=%s", _event.timeStamp.toStr().c_str(), severityString.c_str(), _event.fileName.c_str(), _event.lineNumber, _event.funcName.c_str(), _event.message.c_str());
    tprintf("EVENT (%s): '%s' - file=%s line=%d", severityString.c_str(), _event.message.c_str(), _event.fileName.c_str(), _event.lineNumber);
    // this function does get/modify/put on a vector of events
    // and it can be called from several processes
    // so we need an interprocess mutex
    boost::interprocess::named_mutex mtx(boost::interprocess::open_or_create, "diagnostics_events_named_mutex");
    {
        TRACE_FUNCTION("ipc lock");
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mtx);
        // fetch current list of events
        T_EVENT_LIST events;
        int r = 0;
        if (_rtdb != NULL)
        {
            r = _rtdb->get(EVENT_LIST, &events);
            (void)r; // OK to ignore r: even when there is nothing yet, below code will put the first event in place
            // cleanup old ones
            cleanupOldEvents(events);
            // resize, to avoid fast growth when a client is spamming -- this might cause MDB_MAP_FULL errors
            int MAX_EVENTS = 100;
            if ((int)events.size() > MAX_EVENTS)
            {
                events.resize(MAX_EVENTS);
            }
            // append new one
            events.push_back(_event);
            // put
            _rtdb->put(EVENT_LIST, &events);
        }
        // TODO: spawn a cleanup thread which after a while calls cleanup, to prevent the last event to linger around forever
        // this thread can either run forever on a slow pace, or start once from here
    }
}

void EventHandler::cleanupOldEvents(T_EVENT_LIST &events)
{
    rtime tNow = ftime::now();
    const float timeout = 1.0;
    for (auto it = events.begin(); it != events.end(); )
    {
        if (double(tNow - it->timeStamp) > timeout)
        {
            it = events.erase(it);
        }
        else
        {
            it++;
        }
    }
}

