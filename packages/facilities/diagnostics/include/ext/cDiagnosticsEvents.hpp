 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsEvents.hpp
 *
 * Provide the macros TRACE_ERROR, TRACE_INFO and TRACE_WARNING.
 * Each of these generates an event, publishes it on a ROS topic so that robotControl can relay it to coach.
 *
 *  Created on: Dec 25, 2016
 *      Author: Jan Feitsma
 */

#ifndef CDIAGNOSTICSEVENTS_HPP_
#define CDIAGNOSTICSEVENTS_HPP_

#include "eventType.hpp"
#include "timeConvert.hpp"
#include "macroRedirect.hpp"

#define TRACE_ERROR   (macroRedirect(&diagnostics::diag_error,__FILE__,__LINE__,__FUNCTION__))
#define TRACE_INFO    (macroRedirect(&diagnostics::diag_info,__FILE__,__LINE__,__FUNCTION__))
#define TRACE_WARNING (macroRedirect(&diagnostics::diag_warning,__FILE__,__LINE__,__FUNCTION__))

#define TRACE_ERROR_TIMEOUT(timeout, ...) \
{ \
    static double tlast = 0; \
    double tcurr = getTimeNow(); \
    if (tcurr - tlast > timeout) { \
        TRACE_ERROR( __VA_ARGS__ ) ; \
        tlast = tcurr; \
    } \
}
#define TRACE_INFO_TIMEOUT(timeout, ...) \
{ \
    static double tlast = 0; \
    double tcurr = getTimeNow(); \
    if (tcurr - tlast > timeout) { \
        TRACE_INFO( __VA_ARGS__ ) ; \
        tlast = tcurr; \
    } \
}
#define TRACE_WARNING_TIMEOUT(timeout, ...) \
{ \
    static double tlast = 0; \
    double tcurr = getTimeNow(); \
    if (tcurr - tlast > timeout) { \
        TRACE_WARNING( __VA_ARGS__ ) ; \
        tlast = tcurr; \
    } \
}

namespace diagnostics
{

    void diag_error(falcons::eventType const &event);
    void diag_warning(falcons::eventType const &event);
    void diag_info(falcons::eventType const &event);

} // end of namespace diagnostics



#endif /* CDIAGNOSTICSEVENTS_HPP_ */

