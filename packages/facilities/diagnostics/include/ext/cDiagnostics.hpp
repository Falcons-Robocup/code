 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnostics.hpp
 *
 *    Created on: Jan 05, 2016
 *        Author: Jan Feitsma
 * Fully revised: Dec 3, 2018
 */

#ifndef CDIAGNOSTICS_HPP_
#define CDIAGNOSTICS_HPP_

#include "FalconsRtDB2.hpp"
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
        RtDB2 *_rtdb;
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

