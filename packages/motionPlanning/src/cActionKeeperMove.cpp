 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionKeeperMove.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: Erik Kouters
 */

#include "int/cActionKeeperMove.hpp"
#include "int/stores/configurationStore.hpp"

#include "FalconsCommon.h"
#include <stdexcept>
#include "cDiagnostics.hpp"
#include "cEnvironmentField.hpp"


cActionKeeperMove::cActionKeeperMove()
{
    _result = actionResultTypeEnum::RUNNING;
    _disabledObstacleAvoidance = false;
    getConfig();
}

actionResultTypeEnum cActionKeeperMove::execute()
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
    
    // clip the position such that the goalKeeper stays between the posts
    // also move slightly forward w.r.t. the line
    clipTarget();

    // get current position and calculate deltas
    calculate();
    
    // when inside goal area, the keeper must not respond anymore to obstacles
    // so once the keeper is close to goal, we disable obstacle avoidance by reconfiguring pathPlanning
    checkDisableObstacleAvoidance();

    // determine if we need to extend keeperFrame
    checkExtendKeeperFrame();

    // check if done and set _result
    if (!checkDone())
    {
        _rtdbOutput->setMotionSetpoint(actionTypeEnum::MOVE, _targetPos, false);
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

void cActionKeeperMove::clipTarget()
{
    poiInfo goalPostLeft = poiInfo();
    cEnvironmentField::getInstance().getFieldPOI(P_OWN_GOALPOST_LEFT, goalPostLeft);

    poiInfo goalPostRight = poiInfo();
    cEnvironmentField::getInstance().getFieldPOI(P_OWN_GOALPOST_RIGHT, goalPostRight);

    _targetPos.x = fmax(_targetPos.x, goalPostLeft.x + _GOALPOST_OFFSET_KEEPER);
    _targetPos.x = fmin(_targetPos.x, goalPostRight.x - _GOALPOST_OFFSET_KEEPER);

    // Use a fixed Y coordinate relative to the goal
    _targetPos.y = goalPostLeft.y + _Y_MAX_OFFSET_KEEPER;

    // Always face away from goal
    _targetPos.phi = M_PI_2;
}

void cActionKeeperMove::traceResult(bool success, const char* details)
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

void cActionKeeperMove::unpackParameters()
{
    _targetPos.x = boost::lexical_cast<float>(_params.at(0));
}

void cActionKeeperMove::calculate()
{
    _currentPos = _wm->getPosition();

    _deltaDistance = (Vector2D(_targetPos.x, _targetPos.y) - _currentPos.xy()).size();
    _xyOk = (_deltaDistance < _xyThreshold);
}

void cActionKeeperMove::getConfig()
{
    motionPlanning::configuration cfg = motionPlanning::configurationStore::getConfiguration();
    _xyThreshold = cfg.getKeeperMove_XYThreshold();
    _Y_MAX_OFFSET_KEEPER = cfg.getKeeperMove_YMaxOffset();
    _GOALPOST_OFFSET_KEEPER = cfg.getKeeperMove_XGoalpostOffset();
    _haveKeeperFrame = (getRobotNumber() == 1);
    // TODO: do we need some facility which knows about these hardware capabilities?
}

bool cActionKeeperMove::checkDone()
{
    if (_xyOk)
    {
        _result = actionResultTypeEnum::PASSED;
        return true;
    }
    return false;
}

void cActionKeeperMove::checkExtendKeeperFrame()
{
    // if no ball seen, do nothing
    if (_wm->noBall())
    {
        return;
    }
    
    // if we do not have a keeperFrame, then do nothing
    if (!_haveKeeperFrame)
    {
        return;
    }
    
    // get ball data
    Vector3D ballPos = _wm->ballPosition();
    Vector3D ballVel = _wm->ballVelocity();

    // don't respond to balls on far half, for instance being passed around
    if (ballPos.y > 0)
    {
        return; // do nothing - ball is too far away
    }

    // check if it likely to be a pass
    if (fabs(ballVel.x) > fabs(ballVel.y))
    {
        return; // do nothing - ball seems to be moving along X axis
    }
    
    // check ball speed towards goal
    if (ballVel.y > -_FRAME_EXTENSION_SPEED_THRESHOLD)
    {
        return; // do nothing - ball is moving too slow or not towards goalie, assume we can deal with it via motion only
    }

    // determine if we need to extend the frame
    // left/right: based on current distance on where we are and where we need to be 
    // if significantly different, keeper needs to perform a dash, so it might as well extend the frame
    // NOTE: we do not worry about rapid/erratic setpoints, since we expect these are filtered by peripheralsInterface and/or firmware
    if (_currentPos.x + _FRAME_SIDE_EXTENSION_TOLERANCE < _targetPos.x)
    {
        _rtdbOutput->setKeeperFrameSetpoint(keeperFrameSetpointEnum::RIGHT);
    }
    else if (_currentPos.x - _FRAME_SIDE_EXTENSION_TOLERANCE > _targetPos.x)
    {
        _rtdbOutput->setKeeperFrameSetpoint(keeperFrameSetpointEnum::LEFT);
    }
    // otherwise: is ball in the air?
    else if (ballPos.z > _FRAME_TOP_EXTENSION_TOLERANCE)
    {
        _rtdbOutput->setKeeperFrameSetpoint(keeperFrameSetpointEnum::UP);
    }
}

void cActionKeeperMove::checkDisableObstacleAvoidance()
{
    float y_delta_goal = fabs(_currentPos.y - _targetPos.y);
    // one time only
    if ((y_delta_goal < 1.0) && !_disabledObstacleAvoidance)
    {
        _rtdbOutput->disableObstacleAvoidance();
        _disabledObstacleAvoidance = true;
        // TODO: re-enable obstacle avoidance when action finishes (e.g. moving to side for substitution of PARK)
    }
}

