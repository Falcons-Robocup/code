 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include "../include/int/MP_ActionTurnAwayFromOpponent.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"


MP_ActionTurnAwayFromOpponent::MP_ActionTurnAwayFromOpponent()
{
}

actionResultTypeEnum MP_ActionTurnAwayFromOpponent::execute()
{
    TRACE_FUNCTION("");
    
    // check basics conditions to return FAILED (robot not inplay)
    if (!_wm->isActive())
    {
        return actionResultTypeEnum::FAILED;
    }

    // unpack parameters
    unpackParameters();

    // get current position and calculate deltas
    calculate();

    // move by writing setpoint to RTDB and calling pathPlanning iteration
    return setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _targetPos, _slow);
}

void MP_ActionTurnAwayFromOpponent::unpackParameters()
{
    _opponentPos.x = boost::lexical_cast<float>(_params.at(0));
    _opponentPos.y = boost::lexical_cast<float>(_params.at(1));
    _slow = boost::lexical_cast<int>(_params.at(2));
}

void MP_ActionTurnAwayFromOpponent::calculate()
{
    _currentPos = _wm->getPosition();
    _targetPos = _currentPos;

    float targetPhi = angle_between_two_points_0_2pi(_currentPos.x, _currentPos.y, _opponentPos.x, _opponentPos.y);
    targetPhi = project_angle_0_2pi(targetPhi + M_PI);

    _targetPos.phi = targetPhi;
}

