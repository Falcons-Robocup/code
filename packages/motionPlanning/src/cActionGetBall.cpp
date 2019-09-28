 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionGetBall.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Jan Feitsma
 */

#include "int/cActionGetBall.hpp"
#include "int/stores/configurationStore.hpp"

using namespace std;

/* Get the ball.
 * This motionPlanning action combines legacy teamplay actions getBall and getBallOnVector.
 * 
 * Exit criteria:
 *  PASSED: robot has the ball
 *  FAILED: other teammember got the ball, or there is no ball, or robot is inactive
 *
 * Options:
 *   bool slow : move slowly towards the ball (typically used during setpiece)
 *
 * Method:
 *  * when chasing a closeby ball, project a target using the ball speed vector (legacy getBallOnVector)
 *  * TODO when moving towards a far-away ball, do a sprint, in case no obstacles closeby and if needed
 *
 * Interfacing with teamplay:
 *  * teamplay should assign 1 robot to get the ball (based on ball proximity, vector and/or intention)
 *    and that robot should be the only one calling this getBall action
 *    (if more robots are assigned, they soon end up fighting over the ball, which looks silly and is inefficient)
 *  * teamplay is responsible for outside-field boundaries, motionPlanning will just try to get the ball, regardless...
 *
 * Future: 
 *  * adjust target to take ball bouncing into account
 * 
 */

void cActionGetBall::unpackParameters()
{
    _slow = bool(boost::lexical_cast<int>(_params.at(0)));
}

void cActionGetBall::getConfig()
{
    motionPlanning::configuration cfg = motionPlanning::configurationStore::getConfiguration();
    _obstacleThreshold  = cfg.getGetBall_ObstacleThreshold();
    _ballSpeedThreshold = cfg.getGetBall_BallSpeedThreshold();
    _ballSpeedScaling   = cfg.getGetBall_BallSpeedScaling();
}

void cActionGetBall::initialize()
{

    getConfig();
    _rtdbOutput->setBallHandlersSetpoint(true);
    
    _R  = _wm->getPosition();
    _Vr = _wm->getVelocity();
    _B  = _wm->ballPosition();
    _Vb = _wm->ballVelocity();
    // face ball
    _target = _R;
    faceBall();
    _motionProfile = cMotionProfileType::NORMAL;
}

void cActionGetBall::analyzeGeometry()
{
    // analyze ball movement
    Velocity2D ballVelocity(_wm->ballVelocity().x, _wm->ballVelocity().y, 0);
    _ballMovingFastEnough = (vectorsize(Vector2D(ballVelocity.x, ballVelocity.y)) > _ballSpeedThreshold);
    
    // TODO: determine if a sprint is needed
    _needToSprint = false;

    // roadmap: detect if ball is bouncing, if so, adjust depth D on the ball line
}

void cActionGetBall::determineMotionProfile()
{
    _motionProfile = cMotionProfileType::NORMAL;
    if (_slow)
    {
        _motionProfile = cMotionProfileType::SLOW;
    }
    else
    {
        bool obstacleCloseBy = false;
        if ((_wm->numObstacles() > 0) && (_wm->closestObstacleDistance() < _obstacleThreshold))
        {
            obstacleCloseBy = true;
        }
        bool canSprint = !obstacleCloseBy;
        if (canSprint && _needToSprint)
        {
            _motionProfile = cMotionProfileType::FAST;
        }
        // TODO: finetune further for accuracy: if closeby enough and ball does not move too fast, then do not sprint
    }
}

void cActionGetBall::faceBall()
{
    _target.phi = angle_between_two_points_0_2pi(_R.x, _R.y, _B.x, _B.y);
}

actionResultTypeEnum cActionGetBall::execute()
{
    TRACE_FUNCTION("");

    // check active
    if (!_wm->isActive())
    {
        return actionResultTypeEnum::FAILED;
    }
    // check ball existence
    if (_wm->noBall())
    {
        return actionResultTypeEnum::FAILED;
    }
    // check ball possession
    if (_wm->hasBall())
    {
        return actionResultTypeEnum::PASSED;
    }
    if (_wm->teamHasBall())
    {
        return actionResultTypeEnum::FAILED;
    }

    unpackParameters();

    initialize();
    if (_wm->opponentHasBall())
    {
        // TODO - compute if we need to place a subtarget in front of the ball before grabbing it.
        _target.x = _B.x;
        _target.y = _B.y;
        TRACE("harass (%.2f, %.2f)", _target.x, _target.y);
    }
    else
    {
        // ball is free on the field
    
        // analyze the situation
        analyzeGeometry();
        
        // determine motion profile normal/slow/sprint
        determineMotionProfile();
        
        // choose what kind of move to do: getBall or getBallOnVector
        if (_ballMovingFastEnough)
        {
            // getBallOnVector
            Vector2D tmp = Vector2D(_B.x, _B.y) + Vector2D(_wm->ballVelocity().x, _wm->ballVelocity().y) * _ballSpeedScaling;
            _target.x = tmp.x;
            _target.y = tmp.y;
            TRACE("getBallOnVector (%.2f, %.2f)", _target.x, _target.y);
        }
        else
        {
            // getBall
            _target.x = _B.x;
            _target.y = _B.y;
            TRACE("getBall (%.2f, %.2f)", _target.x, _target.y);
        }
    }
    // execute the move
    faceBall();
    TRACE("target (%.2f, %.2f, %.2f)", _target.x, _target.y, _target.phi);
    _rtdbOutput->setMotionSetpoint(actionTypeEnum::MOVE, _target, _slow);
    return actionResultTypeEnum::RUNNING;
}
