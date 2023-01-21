// Copyright 2019-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

#include <stdexcept>
#include <thread>

#include "../include/int/MP_ActionShootAtTarget.hpp"

#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"


MP_ActionShootAtTarget::MP_ActionShootAtTarget()
{
    TRACE("init start");
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
    tprintf("ptrace %5s %8.3f %9s %8.4f %8.4f %d %d %d %8.3f %8.3f", _actionName.c_str(), elapsed(), phaseString().c_str(), _deltaPhi, _moveAngleThreshold, _didShoot, _wm->hasBall(), _settledOk, _settleTimer.elapsed(), _aimTimer.elapsed());
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
    _sleepAfterShoot =  getConfig().shootAtTargetConfig.sleepAfterShoot; // don't worry, it is not a blocking sleep
}

void MP_ActionShootAtTarget::initialize()
{
    _previousPhase = phaseEnum::UNDEFINED;
    _currentPhase = phaseEnum::UNDEFINED;
    TRACE_CONTEXT_START(phaseString().c_str(), this);
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

    // Trace the config
    std::ostringstream oss;
    oss << "_timeout=" << _timeout << "; _aimSettleTime=" << _aimSettleTime << "; _sleepAfterShoot=" << _sleepAfterShoot;
    TRACE_FUNCTION(oss.str().c_str());
}

void MP_ActionShootAtTarget::finalize()
{
    TRACE_FUNCTION("");
    TRACE_CONTEXT_FINISH(phaseString().c_str(), this);
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
}

void MP_ActionShootAtTarget::executeShot()
{
    TRACE("Executing shoot");
    _rtdbOutput->setShootSetpoint(shootPhaseEnum::SHOOT, shootTypeEnum::SHOOT, Position2D());
}

void MP_ActionShootAtTarget::shoot()
{
    // make sure to shoot only once
    if (!_didShoot)
    {
        TRACE("Shooting (_didShoot == false)");
        executeShot();
        _shootTimer.reset();
        _didShoot = true;

        // UGLY HACK / TODO / FIXME
        // Sleep after shooting.
        // This hack prevents the robot from immediately chasing the ball after shooting
        // This is because the Teamplay logic is that the 'closest robot to the ball' always grabs the ball.
        // The better solution is to improve the Teamplay logic.
        // But until that time, this should work
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(_sleepAfterShoot * 1000.0)));
    }
}

void MP_ActionShootAtTarget::moveToTarget()
{
    // ignore pathPlanning return value, it is determined in this function based on shooting phase
    (void)setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _targetPos, motionTypeEnum::NORMAL, false);
}

void MP_ActionShootAtTarget::enterPhaseRotate()
{
    TRACE_CONTEXT_FINISH(phaseString().c_str(), this);
    _currentPhase = phaseEnum::ROTATE;
    TRACE_CONTEXT_START(phaseString().c_str(), this);
    _rtdbOutput->setBallHandlersSetpoint(true);
}

void MP_ActionShootAtTarget::enterPhaseSettle()
{
    TRACE_CONTEXT_FINISH(phaseString().c_str(), this);
    _currentPhase = phaseEnum::SETTLE;
    TRACE_CONTEXT_START(phaseString().c_str(), this);
    _settledOk = false;
    _settleTimer.reset();
}

void MP_ActionShootAtTarget::enterPhaseShoot()
{
    TRACE_CONTEXT_FINISH(phaseString().c_str(), this);
    _currentPhase = phaseEnum::SHOOT;
    TRACE_CONTEXT_START(phaseString().c_str(), this);
    _shootTimer.reset();
    // performance tracing event    x     y     z  dist lob
    // TODO make ptrace specific: SHOOTEVENT versus PASSEVENT, for better visualization
    //PTRACE("SHOOTEVENT %6.2f %6.2f %6.2f %6.2f %d", _shootTargetPos.x, _shootTargetPos.y, _shootTargetPos.z, _distance, (int)_doLob);
}

void MP_ActionShootAtTarget::enterPhaseFinished()
{
    stopMoving();
    TRACE_CONTEXT_FINISH(phaseString().c_str(), this);
    _currentPhase = phaseEnum::FINISHED;
    TRACE_CONTEXT_START(phaseString().c_str(), this);
}

std::string MP_ActionShootAtTarget::phaseString()
{
    switch (_currentPhase)
    {
        case phaseEnum::UNDEFINED:
            return "UNDEFINED_PHASE";
        case phaseEnum::ROTATE:
            return "ROTATE_PHASE";
        case phaseEnum::SETTLE:
            return "SETTLE_PHASE";
        case phaseEnum::SHOOT:
            return "SHOOT_PHASE";
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
                TRACE_WARNING("Wide angle timer expired while aiming for a shot during ROTATE phase");
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
                    TRACE_WARNING("Wide angle timer expired while aiming for a shot during SETTLE phase");
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
            
            // reset aim timer if outside of the wide angle
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
            break;
        }
        case phaseEnum::SHOOT:
        {
            TRACE_SCOPE("SHOOT", "");
            moveToTarget();
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


