 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsDutyCycle.cpp
 *
 *  Created on: Mar 07, 2017
 *      Author: Jan Feitsma
 */


#include "ext/cDiagnosticsDutyCycle.hpp"
#include "ext/cDiagnosticsEvents.hpp"
#include "FalconsCommon.h"
#include "timeConvert.hpp"

using namespace diagnostics;

cDiagnosticsDutyCycle::cDiagnosticsDutyCycle(std::string name, int freq, float warningThreshold)
{
    _name = name;
    _frequency = freq;
    _warningThreshold = warningThreshold;
    _active = false;
}

cDiagnosticsDutyCycle::~cDiagnosticsDutyCycle()
{
}

void cDiagnosticsDutyCycle::pokeStart()
{
    // must not be active
    if (_active)
    {
        TRACE_ERROR("unexpected pokeStart from process %s", _name.c_str());
        return;
    }
    // store timestamp and raise flag
    _lastStart = getTimeNow();
    _active = true;
}

void cDiagnosticsDutyCycle::pokeEnd()
{
    // must be active
    if (!_active)
    {
        TRACE_ERROR("unexpected pokeEnd from process %s", _name.c_str());
        return;
    }
    // downcast and handle timestamp, lower flag
    handleElapsed((float)(getTimeNow() - _lastStart));
    _active = false;
}

void cDiagnosticsDutyCycle::handleElapsed(float elapsed)
{
    // disable in simulation
    if (isSimulatedEnvironment())
    {
        return;
    }
    float dutyCycle = elapsed * _frequency;
    TRACE("dutyCycle for process %s: %5.1f%%", _name.c_str(), 100.0 * dutyCycle);
    if (dutyCycle > _warningThreshold)
    {
        TRACE_WARNING("duty cycle for process %s is %.1f%%, which exceeds threshold (%.1f%%)", _name.c_str(), 100.0 * dutyCycle, 100.0 * _warningThreshold);
        // TODO filter dupes?
    }
}


