 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionShootAtTarget.cpp
 *
 *  Created on: Nov 18, 2017
 *      Author: Jan Feitsma
 *    
 *   The shootAtTarget action tries to decouple motion tuning from shooting accuracy by 
 *   using a settling phase and other tricks.
 *
 *   Action shootAtTarget consists of 3 phases.
 *   * phase 1: ROTATE
 *     * rotate as fast as possible towards aiming direction
 *       * note: pathPlanning turnWithBall velocity&acceleration should be tuned high
 *     * at the start of the phase:
 *       * send a prepare command to shootPlanning, to set the kicker height
 *       * enable ballHandlers (in case ballRelaxation was started)
 *     * this phase ends when robot is aiming towards target within given accuracy
 *       * but since the robot may still be moving (especially when tuned aggressively with overshoot), we need to SETTLE
 *       * shootAccuracy is measured in meters, not radians (typically 5cm)
 *         * radians follow from distance to shoot target (5cm accuracy over 10m is 0.005rad)
 *   * phase 2: SETTLE
 *     * stabilizing aim towards target
 *     * somewhere midway: disable ballHandlers and start ballRelaxation timer, anticipating the shoot timestamp
 *     * phase finishes with success if settleTimer expires (typically 3 heartbeats = 0.1s)
 *     * go back to ROTATE phase in case robot rotates out of the aim direction (or noise)
 *     * to prevent getting stuck between rotate and settle, there is an aimTimeout (typically 2 or 3 seconds)
 *     * future: shaking / moving to reduce inaccuracy due to ballHandler pressure might be added in settle phase, or even in rotate phase
 *   * phase 3: SHOOT
 *     * wait for the ballRelaxation timer to expire, to guarantee a stable relaxation state
 *     * shoot the ball
 *     * it may take a few heartbeats for the ball to leave the robot
 *     * return PASSED once the ball has left
 *     * return FAILED in case the ball does not leave after shootTimeout
 *       * e.g. in case of shooter failure, or scrum 
 *
 */

#include "../include/int/MP_ActionShootAtTarget.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"


MP_ActionShootAtTarget::MP_ActionShootAtTarget()
{
    TRACE("init start");
    initialize();
    TRACE("init end");
}

MP_ActionShootAtTarget::~MP_ActionShootAtTarget()
{
    TRACE("finalize start");
    finalize();
    TRACE("finalize start");
}

actionResultTypeEnum MP_ActionShootAtTarget::execute()
{
    TRACE_FUNCTION("");
    _timestamp = ftime::now();

    // check if finished already
    if (_result != actionResultTypeEnum::RUNNING)
    {
        return _result;
    }
    
    // unpack parameters
    unpackParameters();
    
    // calculate target and delta angle
    calculateAngles();

    // determine which phase we are in and handle phase transitions
    // TODO: consider using a state machine? 
    determineCurrentPhase();

    // check basics conditions to return FAILED (lost ball, robot not inplay)
    if (!checkOk())
    {
        finalize();
        return actionResultTypeEnum::FAILED;
    }

    // performance tracing
    traceData();

    // determine what to do based on current phase and send out commands
    executeCurrentPhase();

    // wrap up
    _previousPhase = _currentPhase;
    double elapsedThisTick = ftime::now() - _timestamp;
    if (elapsedThisTick > 0.008)
    {
        TRACE("elapsedThisTick=%.3fs", elapsedThisTick);
    }
    return _result;
}

void MP_ActionShootAtTarget::traceData()
{
    // elapsedTime phase deltaPhi tolerance didShoot hasBall bhEnabled settleTimer aimTimer
    tprintf("ptrace %5s %8.3f %9s %8.4f %8.4f %d %d %d %d %8.3f %8.3f", _actionName.c_str(), elapsed(), phaseString().c_str(), _deltaPhi, _moveAngleThreshold, _didShoot, _wm->hasBall(), !_disabledBh, _settledOk, _settleTimer.elapsed(), _aimTimer.elapsed());
}

void MP_ActionShootAtTarget::traceResult(bool success, const char* details)
{
    if (success)
    {
        TRACE_INFO("shootAtTarget PASSED (%s)", details);
    }
    else
    {
        TRACE_INFO("shootAtTarget FAILED (%s)", details);
    }
}

void MP_ActionShootAtTarget::unpackParameters()
{
    _shootTargetPos.x = boost::lexical_cast<float>(_params.at(0));
    _shootTargetPos.y = boost::lexical_cast<float>(_params.at(1));
    _shootTargetPos.z = boost::lexical_cast<float>(_params.at(2));
    if (_params.at(3) == "SHOOT")
    {
        _shootType = actionTypeEnum::SHOOT;
    }
    else
    {
        _shootType = actionTypeEnum::LOB;
    }
}

void MP_ActionShootAtTarget::calculateAngles()
{
    _currentPos = _wm->getPosition();
    float targetAngle = angle_between_two_points_0_2pi(_currentPos.x, _currentPos.y, _shootTargetPos.x, _shootTargetPos.y);
    _targetPos = Position2D(_currentPos.x, _currentPos.y, targetAngle);
    _distance = (Vector2D(_shootTargetPos.x, _shootTargetPos.y) - _currentPos.xy()).size();
    _moveAngleThreshold = 0.1; // default (closeby targets only): quite coarse
    if (_distance > 0.5) // avoid division by zero
    {
        _moveAngleThreshold = fabs(sin(_accuracy / _distance));
    }
    TRACE("_accuracy=%.3f _moveAngleThreshold=%.3f", _accuracy, _moveAngleThreshold);
    _deltaPhi = project_angle_mpi_pi(_currentPos.phi - _targetPos.phi);
    _aimWithinTolerance = (fabs(_deltaPhi) < _moveAngleThreshold);
}

bool MP_ActionShootAtTarget::checkOk()
{
    if (!_wm->isActive())
    {
        traceResult(false, "robot inactive");
        return false;
    }
    if (!_wm->hasBall() && ((_currentPhase == phaseEnum::ROTATE) || (_currentPhase == phaseEnum::SETTLE)))
    {
        // lost ball during ROTATE or SETTLE, probably due to bumping by opponent, or poor ballHandling control?
        traceResult(false, "ball lost");
        return false;
    }
    return true; // all OK
}

void MP_ActionShootAtTarget::getSpecificConfig()
{
    _accuracy = getConfig().shootAtTargetConfig.accuracy;
    _timeout = getConfig().shootAtTargetConfig.timeout;
}

void MP_ActionShootAtTarget::getCommonConfig()
{
    _aimSettleTime   =  getConfig().shootAtTargetConfig.aimSettleTime;
    _coarseAngle     =  getConfig().shootAtTargetConfig.coarseAngle; // timeout starts when within this tolerance
    _disableBhDelay  =  getConfig().shootAtTargetConfig.disableBhDelay;
    _sleepAfterShoot =  getConfig().shootAtTargetConfig.sleepAfterShoot; // don't worry, it is not a blocking sleep
}

void MP_ActionShootAtTarget::initialize()
{
    _previousPhase = phaseEnum::UNDEFINED;
    _currentPhase = phaseEnum::UNDEFINED;
    _result = actionResultTypeEnum::RUNNING;
    _didShoot = false;
    _settledOk = false;
    _actionName = "SHOOT";
    getCommonConfig();
    getSpecificConfig();
    // setup timers
    _actionTimer.reset();
    _aimTimer.reset();
    _aimTimer.setDuration(_timeout);
    _settleTimer.setDuration(_aimSettleTime);
    _shootTimer.setDuration(_sleepAfterShoot);
    _bhTimer.setDuration(_disableBhDelay);
}

void MP_ActionShootAtTarget::finalize()
{
    TRACE(">");
    if (_isConnected)
    {
        _rtdbOutput->setBallHandlersSetpoint(true);

        // reset kicker height to zero
        _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::PASS, Position2D(5.0, 0, 0));

        // This was introduced in December 2017. During field testing we observed that during a shootAtTarget,
        // the ballhandlers would turn on/off - causing us to lose the ball. Apparently, during a shootAtTarget,
        // other actions might be started, causing this finalize method to be called. However, we do not see
        // any apparent reason for stopping the robot and/or stopping the ballhandlers at this point.
//        stopMoving();
    }
    TRACE("<");
}

void MP_ActionShootAtTarget::checkDisableBh()
{
    // disable ball handlers a fixed period before shooting
    if (!_disabledBh && (_settleTimer.elapsed() + _disableBhDelay > _aimSettleTime))
    {
        bool doDisable = true;
        // also require that it is not one-off settling, because it will cause timing issues 
        // with sequential enable/disable which severely reduce accuracy
        if ((_currentPhase == phaseEnum::SETTLE) && (_previousPhase != phaseEnum::SETTLE))
        {
            doDisable = false;
        }
        if (doDisable)
        {
            // EKPC HACK Portugal 2019
            // Remove relax ballhandlers from the shooting sequence
            // When aiming at goal, the ballhandlers were disabled, but the robot was still repositioning/aiming for the goal.
            // This resulted in the robot losing control of the ball when shooting
            //_rtdbOutput->setBallHandlersSetpoint(false);

            _bhTimer.reset();
            _disabledBh = true;
        }
    }
}

void MP_ActionShootAtTarget::executeShot()
{
    TRACE("Executing shoot");
    _rtdbOutput->setShootSetpoint(shootPhaseEnum::SHOOT, shootTypeEnum::SHOOT, Position2D());
}

void MP_ActionShootAtTarget::shoot()
{
    // make sure to shoot only once
    // also, we require a fixed duration of ballHandler relaxation, to release the inwards force applied from ballHandlers on ball 
    if (!_didShoot && _bhTimer.expired())
    {
        TRACE("Shooting: !_didShoot && _bhTimer.expired()");
        executeShot();
        _shootTimer.reset();
        _didShoot = true;
    }
}

void MP_ActionShootAtTarget::moveToTarget()
{
    // ignore pathPlanning return value, it is determined in this function based on shooting phase
    (void)setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _targetPos, false, false);
}

void MP_ActionShootAtTarget::enterPhaseRotate()
{
    _currentPhase = phaseEnum::ROTATE;
    _disabledBh = false;
    _rtdbOutput->setBallHandlersSetpoint(true);
}

void MP_ActionShootAtTarget::enterPhaseSettle()
{
    _currentPhase = phaseEnum::SETTLE;
    _settledOk = false;
    _settleTimer.reset();
}

void MP_ActionShootAtTarget::enterPhaseShoot()
{
    // stopMoving(); // Erik advises to keep moving
    _currentPhase = phaseEnum::SHOOT;
    _shootTimer.reset();
    // performance tracing event    x     y     z  dist lob
    // TODO make ptrace specific: SHOOTEVENT versus PASSEVENT, for better visualization
    //PTRACE("SHOOTEVENT %6.2f %6.2f %6.2f %6.2f %d", _shootTargetPos.x, _shootTargetPos.y, _shootTargetPos.z, _distance, (int)_doLob);
}

void MP_ActionShootAtTarget::enterPhaseFinished()
{
    stopMoving();
    _currentPhase = phaseEnum::FINISHED;
}

std::string MP_ActionShootAtTarget::phaseString()
{
    switch (_currentPhase)
    {
        case phaseEnum::UNDEFINED:
            return "UNDEFINED";
        case phaseEnum::ROTATE:
            return "ROTATE";
        case phaseEnum::SETTLE:
            return "SETTLE";
        case phaseEnum::SHOOT:
            return "SHOOT";
        case phaseEnum::FINISHED:
            if (_result == actionResultTypeEnum::PASSED)
            {
                return "SUCCESS";
            }
            return "FAILED";
    }
    return "ERROR";
}

void MP_ActionShootAtTarget::determineCurrentPhase()
{
    TRACE_FUNCTION("");
    switch (_previousPhase)
    {
        case phaseEnum::UNDEFINED:
        {
            TRACE_SCOPE("UNDEFINED", "");
            enterPhaseRotate();
            _initialDeltaPhi = _deltaPhi;

            if (_shootType == actionTypeEnum::LOB)
            {
                _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::LOB, Position2D(_distance, 0, _shootTargetPos.z));
            }
            else if (_shootType == actionTypeEnum::SHOOT)
            {
                _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::SHOOT, Position2D(_distance, 0, _shootTargetPos.z));
            }
            else
            {
                _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::PASS, Position2D(_distance, 0, _shootTargetPos.z));
            }
            break;
        }
        case phaseEnum::ROTATE:
        {
            TRACE_SCOPE("ROTATE", "");
            if (_aimTimer.expired())
            {
                TRACE("Entering SHOOT phase: _aimTimer expired during ROTATE phase");
                enterPhaseShoot();
            }
            else if (_aimWithinTolerance)
            {
                TRACE("Entering SETTLE phase: _aimWithinTolerance is TRUE during ROTATE phase");
                enterPhaseSettle();
            }
            break;
        }
        case phaseEnum::SETTLE:
        {
            TRACE_SCOPE("SETTLE", "");
            if (_settleTimer.expired() || _aimTimer.expired())
            {
                _settledOk = true;
                if (_settleTimer.expired())
                {
                    TRACE("Entering SHOOT phase: _settleTimer expired during SETTLE phase");
                }
                else
                {
                    TRACE("Entering SHOOT phase: _aimTimer expired during SETTLE phase");
                }
                enterPhaseShoot();
            }
            else if (!_aimWithinTolerance)
            {
                TRACE("Entering ROTATE phase: _aimWithinTolerance is FALSE during SETTLE phase");
                enterPhaseRotate();
            }
            break;
        }
        case phaseEnum::SHOOT:
        {
            TRACE_SCOPE("SHOOT", "");
            if (!_wm->hasBall())
            {
                if (_shootTimer.expired())
                {
                    TRACE("Shooting: _shootTimer expired during SHOOT phase");
                    enterPhaseFinished();
                    // require final aim also to be OK, since robot might drift during shooting phase
                    if (_settledOk && _aimWithinTolerance)
                    {
                        TRACE("_settledOK && _aimWithinTolerance");
                        char details[250] = {0};
                        sprintf(details, "elapsed=%.3f, initialDeltaPhi=%.4f", elapsed(), _initialDeltaPhi);
                        _result = actionResultTypeEnum::PASSED;
                        traceResult(true, details);
                    }
                    else
                    {
                        if (!_settledOk)
                        {
                            TRACE("!_settledOK");
                            char details[250] = {0};
                            sprintf(details, "did not settle, initialDeltaPhi=%.4f, deltaPhi=%.4f, threshold=%.4f", _initialDeltaPhi, _deltaPhi, _moveAngleThreshold);
                            _result = actionResultTypeEnum::FAILED;
                            traceResult(false, details);
                        }
                        else
                        {
                            TRACE("_settledOK && !_aimWithinTolerance");
                            char details[250] = {0};
                            sprintf(details, "drifted after settle, initialDeltaPhi=%.4f, deltaPhi=%.4f, threshold=%.4f", _initialDeltaPhi, _deltaPhi, _moveAngleThreshold);
                            _result = actionResultTypeEnum::FAILED;
                            traceResult(false, details);
                        }
                    }
                }
                // else: wait for shootTimer to expire
            }
            else if (_shootTimer.expired() && _didShoot)
            {
                TRACE("Shot fired, ball did not leave. _shootTimer.expired() && _didShoot");
                enterPhaseFinished();
                _result = actionResultTypeEnum::FAILED;
                traceResult(false, "ball did not leave");
            }
            break;
        }
        case phaseEnum::FINISHED:
        {
            TRACE_SCOPE("FINISHED", "");
            TRACE("we finished already ... stop poking me");
            break;
        }
        default:
            throw std::runtime_error(std::string("invalid phase"));
    }
}

void MP_ActionShootAtTarget::executeCurrentPhase()
{
    TRACE_FUNCTION("");
    switch (_currentPhase)
    {
        case phaseEnum::UNDEFINED:
        {
            TRACE_SCOPE("UNDEFINED", "");
            throw std::runtime_error(std::string("impossible to currently be in undefined phase"));
            break;
        }
        case phaseEnum::ROTATE:
        {
            TRACE_SCOPE("ROTATE", "");
            
            if (_shootType == actionTypeEnum::LOB)
            {
                _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::LOB, Position2D(_distance, 0, _shootTargetPos.z));
            }
            else if (_shootType == actionTypeEnum::SHOOT)
            {
                _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::SHOOT, Position2D(_distance, 0, _shootTargetPos.z));
            }
            else
            {
                _rtdbOutput->setShootSetpoint(shootPhaseEnum::PREPARE, shootTypeEnum::PASS, Position2D(_distance, 0, _shootTargetPos.z));
            }
            
            // reset aim timer
            if (fabs(_deltaPhi) > _coarseAngle)
            {
                _aimTimer.reset();
            }
            moveToTarget();
            break;
        }
        case phaseEnum::SETTLE:
        {
            TRACE_SCOPE("SETTLE", "");
            moveToTarget();
            // due to overshoot, we lost the ball too often in testmatch VDL 2018-01-25
            // so we decided to disable this micro-optimization: only disable bh at start of SHOOT phase
            //checkDisableBh(); 
            break;
        }
        case phaseEnum::SHOOT:
        {
            TRACE_SCOPE("SHOOT", "");
            checkDisableBh();
            moveToTarget(); // Erik advises to keep moving
            shoot();
            break;
        }
        case phaseEnum::FINISHED:
        {
            TRACE_SCOPE("FINISHED", "");
            finalize();
            break;
        }
        default:
            throw std::runtime_error(std::string("invalid phase"));
    }
}


