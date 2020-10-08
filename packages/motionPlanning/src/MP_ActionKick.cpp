 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionKick.cpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#include "../include/int/MP_ActionKick.hpp"

#include "cDiagnostics.hpp"


MP_ActionKick::MP_ActionKick()
{
    _actuatedKickerHeight = false;
    _actuatedBhDisable = false;
    // consecutive timers
    _heightTimer.setDuration(1.0); // TODO make reconfigurable
    // a bit long, but we have no feedback nor any guarantee how fast the height lever will move ... 
    // since this action is mostly intended for calibration purposes, better safe than sorry!
    _bhTimer.setDuration(getConfig().shootAtTargetConfig.disableBhDelay); // shared with pass- and shoot action
}

void MP_ActionKick::unpackParameters()
{
    _kickPower = boost::lexical_cast<float>(_params.at(0));
    _kickHeight = boost::lexical_cast<float>(_params.at(1));
}

actionResultTypeEnum MP_ActionKick::execute()
{
    unpackParameters();
    
    // kick is a special action for testing purposes
    // from robotCLI, it often happens that wm is not active or that we do not have the ball
    // so unlike in the 'production' shoot- and pass actions, here we do not check on wm->isActive.
    // We do check on wm->hasBall.
    // Exception: When the kick power is below *30*, we do allow a shot without ball.

    float powerThreshold = 30.0;
    if (!_wm->hasBall() && _kickPower > powerThreshold)
    {
        TRACE_WARNING("cannot kick without having the ball, unless power is below %.1f", powerThreshold);
        return actionResultTypeEnum::FAILED;
    }

    // done?
    if (_actuatedBhDisable && _bhTimer.expired())
    {
        // finish: shoot
        _rtdbOutput->setKickerPower(_kickPower);
        // TODO: wait until ball gone, then reset kicker height to zero and enable ballhandlers
        // maybe we need to instantiate a finite state machine to reduce code complexity, find common part for kick/pass/shoot
        return actionResultTypeEnum::PASSED;
    }
    // timed sequence
    if (!_actuatedKickerHeight)
    {
        // set kicker height once, only at the very start
        _rtdbOutput->setKickerHeight(_kickHeight);
        _actuatedKickerHeight = true;
    }
    // disable ballHandlers
    if (_heightTimer.expired())
    {
        if (!_actuatedBhDisable)
        {
            _bhTimer.reset();
            //_rtdbOutput->setBallHandlersSetpoint(false); // TODO: disabled in shoot so also here ... should merge implementations
            _actuatedBhDisable = true;
        }
    }
    return actionResultTypeEnum::RUNNING;
}

