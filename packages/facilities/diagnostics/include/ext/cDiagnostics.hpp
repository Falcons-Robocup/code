// Copyright 2016-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDiagnostics.hpp
 *
 *    Created on: Jan 05, 2016
 *        Author: Jan Feitsma
 * Fully revised: Dec 3, 2018
 */

#ifndef CDIAGNOSTICS_HPP_
#define CDIAGNOSTICS_HPP_

#include "FalconsRTDB.hpp"
#include "ftime.hpp"

namespace diagnostics
{

    // This class is highly duplicate with the TraceClass in package tracing.
    // TODO: Can we perhaps factor out a base class?
    class EventHandler
    {
    public:
        EventHandler(eventSeverityEnum severity, const char* fileName, const int lineNr, const char* functionName);
        ~EventHandler();

        EventHandler& operator()(const char* format = "", ...);

    private:
        event _event;
        FalconsRTDB *_rtdb;
        int _myRobotId;
        
        void dispatch();
        void cleanupOldEvents(T_EVENT_LIST &events);
    };


} // end of namespace diagnostics

// 2019-04-09: from now on, we ALWAYS use default 1sec timeout, to prevent eventlog spam.
#define TRACE_INFO(...)     (TRACE_INFO_TIMEOUT(1.0, __VA_ARGS__))
#define TRACE_WARNING(...)  (TRACE_WARNING_TIMEOUT(1.0, __VA_ARGS__))
#define TRACE_ERROR(...)    (TRACE_ERROR_TIMEOUT(1.0, __VA_ARGS__))

// TODO: instantiate below 3 similar somehow?
#define TRACE_ERROR_TIMEOUT(timeout, ...) \
{ \
    static double tlast = 0; \
    double tcurr = ftime::now(); \
    if (tcurr - tlast > timeout) { \
        (diagnostics::EventHandler(eventSeverityEnum::ERROR,__FILE__,__LINE__,__FUNCTION__))( __VA_ARGS__ ) ; \
        tlast = tcurr; \
    } \
}
#define TRACE_INFO_TIMEOUT(timeout, ...) \
{ \
    static double tlast = 0; \
    double tcurr = ftime::now(); \
    if (tcurr - tlast > timeout) { \
        (diagnostics::EventHandler(eventSeverityEnum::INFO,__FILE__,__LINE__,__FUNCTION__))( __VA_ARGS__ ) ; \
        tlast = tcurr; \
    } \
}
#define TRACE_WARNING_TIMEOUT(timeout, ...) \
{ \
    static double tlast = 0; \
    double tcurr = ftime::now(); \
    if (tcurr - tlast > timeout) { \
        (diagnostics::EventHandler(eventSeverityEnum::WARNING,__FILE__,__LINE__,__FUNCTION__))( __VA_ARGS__ ) ; \
        tlast = tcurr; \
    } \
}

#endif

