 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * stoppingController.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Coen Tempelaars
 */

#include "int/stoppingController.hpp"

#include "tracing.hpp"

const static float maximumWaitingTime = 5.0; // seconds


boost::signals2::connection StoppingController::stoppedSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _stoppedSignal.connect(subscriber);
}

void StoppingController::declareGameStopped()
{
    if (!_signalSent)
    {
        // raise the 'stopped' signal
        _stoppedSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void StoppingController::control (const ArbiterGameData& gamedata)
{
    control(gamedata, 0.0);
}

void StoppingController::control (const ArbiterGameData& gamedata,
                                  const float secondsSinceLastTransition)
{
    TRACE_FUNCTION("");
    _signalSent = false;

    if (!gamedata.anyRobotIsMoving())
    {
        declareGameStopped();
    }

    if (secondsSinceLastTransition > maximumWaitingTime)
    {
        declareGameStopped();
    }
}
