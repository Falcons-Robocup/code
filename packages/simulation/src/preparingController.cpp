 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * preparingController.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: Coen Tempelaars
 */

#include "int/preparingController.hpp"

#include "tracing.hpp"

const static float minimumWaitingTime =  1.0; //seconds
const static float maximumWaitingTime = 10.0; // seconds


boost::signals2::connection PreparingController::preparedSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _preparedSignal.connect(subscriber);
}

boost::signals2::connection PreparingController::stoppingSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _stoppingSignal.connect(subscriber);
}

void PreparingController::declareGamePrepared()
{
    if (!_signalSent)
    {
        // raise the 'prepared' signal
        _preparedSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void PreparingController::declareGameStopping()
{
    if (!_signalSent)
    {
        // raise the 'stopping' signal
        _stoppingSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void PreparingController::control (const ArbiterGameData& gamedata)
{
    control(gamedata, 0.0);
}

void PreparingController::control (const ArbiterGameData& gamedata,
                                   const float secondsSinceLastTransition)
{
    TRACE_FUNCTION("");
    _signalSent = false;

    if (gamedata.ball.isMoving())
    {
        declareGameStopping();
    }

    if (secondsSinceLastTransition > minimumWaitingTime)
    {
        if (!gamedata.anyRobotIsMoving())
        {
            declareGamePrepared();
        }
    }

    if (secondsSinceLastTransition > maximumWaitingTime)
    {
        declareGamePrepared();
    }
}
