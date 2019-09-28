 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionTurnAwayFromOpponent.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: Erik Kouters
 */

#include "int/cActionTurnAwayFromOpponent.hpp"
#include "int/stores/configurationStore.hpp"

#include "FalconsCommon.h"
#include <stdexcept>
#include "cDiagnostics.hpp"


cActionTurnAwayFromOpponent::cActionTurnAwayFromOpponent()
{
    _result = actionResultTypeEnum::RUNNING;
    getConfig();
}

actionResultTypeEnum cActionTurnAwayFromOpponent::execute()
{
    TRACE_FUNCTION("");
    // check if finished already
    if (_result != actionResultTypeEnum::RUNNING)
    {
        return _result;
    }
    
    // check basics conditions to return FAILED (robot not inplay)
    if (!_wm->isActive())
    {
        traceResult(false, "robot inactive");
        return actionResultTypeEnum::FAILED;
    }

    // unpack parameters
    unpackParameters();

    // get current position and calculate deltas
    calculate();
    
    // check if done and set _result
    if (!checkDone())
    {
        _rtdbOutput->setMotionSetpoint(actionTypeEnum::MOVE, _targetPos, _slow);
    }
    else
    {
        // make sure to end with zero setpoint, to prevent drifting based on last nonzero 
        // setpoint in combination with peripheralsInterface watchdog
        _rtdbOutput->setMotionSetpoint(actionTypeEnum::STOP, Position2D(), false);
    }

    // wrap up
    return _result;
}

void cActionTurnAwayFromOpponent::traceResult(bool success, const char* details)
{
    if (success)
    {
        TRACE_INFO("turnAwayFromOpponent PASSED (%s)", details);
    }
    else
    {
        TRACE_INFO("turnAwayFromOpponent FAILED (%s)", details);
    }
}

void cActionTurnAwayFromOpponent::unpackParameters()
{
    _opponentPos.x = boost::lexical_cast<float>(_params.at(0));
    _opponentPos.y = boost::lexical_cast<float>(_params.at(1));
    _slow = boost::lexical_cast<int>(_params.at(2));
}

void cActionTurnAwayFromOpponent::calculate()
{
    _currentPos = _wm->getPosition();
    _targetPos = _currentPos;

    _targetPhi = angle_between_two_points_0_2pi(_currentPos.x, _currentPos.y, _opponentPos.x, _opponentPos.y);
    _targetPhi = project_angle_0_2pi(_targetPhi + M_PI);

    _deltaPhi = project_angle_mpi_pi(_currentPos.phi - _targetPhi);
    _phiOk = (fabs(_deltaPhi) < _phiThreshold);

    _targetPos.phi = _targetPhi;
}

void cActionTurnAwayFromOpponent::getConfig()
{
    motionPlanning::configuration cfg = motionPlanning::configurationStore::getConfiguration();
    _phiThreshold    =  cfg.getTurnAwayFromOpponent_PhiThreshold();
}

bool cActionTurnAwayFromOpponent::checkDone()
{
    if (_phiOk)
    {
        _result = actionResultTypeEnum::PASSED;
        return true;
    }
    return false;
}

