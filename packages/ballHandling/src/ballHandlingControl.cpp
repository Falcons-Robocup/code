// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballHandlingControl.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#include <chrono>
#include <functional>

#include "int/ballHandlingControl.hpp"

#include "tracing.hpp" // obsolete but still used in commented out traceData function
#include "cDiagnostics.hpp"

using std::bind;
using namespace std::chrono;

const int _calibWarning = 100; // warning level, if delta between measured and calibrated down position is larger than this a "please calibrate me" warning is raised

ballHandlingControl::ballHandlingControl(cRTDBOutputAdapter& rtdbOutputAdapter, ConfigInterface<ConfigBallHandling> *cfi)
{
    _rtdbOutputAdapter = &rtdbOutputAdapter;
    _configInterface = cfi;

    _enabled = true;
    _ballPossession = false;
    _needCalibrationCheck = false;

    _angleLeftFraction = 0.0;
    _angleRightFraction = 0.0;

    start_time = high_resolution_clock::now();
}

ballHandlingControl::~ballHandlingControl()
{
}

void ballHandlingControl::setConfig(ConfigBallHandling const &config)
{
    _config = config;
    // select proper calibration values
    int myRobotId = getRobotNumber();
    bool found = false;
    for (auto cal: config.calibration)
    {
        // check if the values apply to this robot, otherwise ignore
        if (cal.robotId == myRobotId)
        {
            if (found)
            {
                TRACE_ERROR("duplicate calibration values detected for robot %d", myRobotId);
            }
            // store
            _calibration = cal;
            found = true;
        }
    }
    if (!found)
    {
        TRACE_ERROR("no calibration values detected for robot %d", myRobotId);
    }
}

void ballHandlingControl::checkForNewConfig()
{
    if (_configInterface != NULL)
    {
        ConfigBallHandling config;
        if (_configInterface->get(config))
        {
            setConfig(config);
            _needCalibrationCheck = true;
        }
    }
}

void ballHandlingControl::update_status(ballHandlersStatusType status)
{
    _status = status;
}

void ballHandlingControl::update_enabled(bool enabled)
{
    _enabled = enabled;
}

void ballHandlingControl::update_robot_velocity(Velocity2D robotVelocity)
{
    _robotVelocity = robotVelocity;
}

void ballHandlingControl::updateSetpoint()
{
    checkForNewConfig();
    calculateSetpoints();
    //traceData(); // prevent writing diagnostics twice per heartBeat

    _rtdbOutputAdapter->setBallHandlersMotorSetpoint(_enabled, _setpoints);
}

void ballHandlingControl::updateFeedback()
{
    checkForNewConfig();
    calculateAngleFractions();
    if (_needCalibrationCheck)
    {
        // ticket #74: robot does not have the ball so we expect the angle to be close to the configured downangle, if not raise a warning

        // prevent this from triggering too early:
        // * peripheralsInterface can still be initializing
        // * configuration may not have loaded yet
        // * configuration can have been reloaded
        // * robot might hold the ball currently
        bool statusSanityCheck = (_status.angleLeft > 0 && _status.angleLeft < 10000 && _status.angleRight > 0 && _status.angleRight < 10000);
        if (statusSanityCheck == true && _ballPossession == false)
        {
            // OK to perform the check
            if ((abs(_status.angleLeft - _calibration.leftArm.down) > _calibWarning) or (abs(_status.angleRight - _calibration.rightArm.down) > _calibWarning))
            {
                TRACE_WARNING("Offset between calibrated and initial measured down position larger than %d, recalibrate ballHandlers", _calibWarning);
                TRACE_INFO("_calibration.leftArm.down=%d _calibration.rightArm.down=%d _status.angleLeft=%d _status.angleRight=%d");
            }
            _needCalibrationCheck = false;
        }
    }
    determineRobotHasBall();
    traceData();

    _rtdbOutputAdapter->setBallHandlersBallPossession(_ballPossession);

    WRITE_TRACE;
}

void ballHandlingControl::calculateAngleFractions()
{
    // Calculate for each arm what the fraction of the arm is (based on the minimum and maximum angle).
    _angleLeftFraction = calculateAngleFraction(_status.angleLeft, _calibration.leftArm.down, _calibration.leftArm.up);
    _angleRightFraction = calculateAngleFraction(_status.angleRight, _calibration.rightArm.down, _calibration.rightArm.up);
}

double ballHandlingControl::calculateAngleFraction(int angle, int downAngle, int upAngle)
{
    return (angle - downAngle) / ((double) (upAngle - downAngle));
}

int ballHandlingControl::calculateAngle(double angleFraction, int downAngle, int upAngle)
{
    return ((int) (angleFraction * (upAngle - downAngle))) + downAngle;
}

void ballHandlingControl::calculateSetpoints()
{
    _setpoints.angleLeft = calculateAngle(_config.angleSetpoint, _calibration.leftArm.down, _calibration.leftArm.up);
    _setpoints.angleRight= calculateAngle(_config.angleSetpoint, _calibration.rightArm.down, _calibration.rightArm.up);
    _setpoints.velocityLeft = 0;
    _setpoints.velocityRight = 0;

    // Check if ballHandlers are disabled, which is typically done by teamplay via motionPlanning in game state STOP or DROPBALL.
    if (_enabled)
    {
        // Calculate setpoints only if ballhandlers are enabled.

        // arm lifted booleans
        _armLeftLifted = _angleLeftFraction > _config.armLiftedAngleThreshold;
        _armRightLifted = _angleRightFraction > _config.armLiftedAngleThreshold;

        // extra pull option
        if ((_ballPossession && _config.extraPull.enabledWithBall)
            || (!_ballPossession && _config.extraPull.enabledWithoutBall))
        {
            addExtraPullForceWhenOneArmLifted();
        }

        // feed forward option
        if ((_ballPossession && _config.feedForward.enabledWithBall)
            || (!_ballPossession && _config.feedForward.enabledWithoutBall))
        {
            addVelocityFeedForward();
        }
    }
}

void ballHandlingControl::addExtraPullForceWhenOneArmLifted() {
    if (_armLeftLifted && !_armRightLifted) {
        _setpoints.velocityLeft += _config.extraPull.setpointVelocity;
    } else if (!_armLeftLifted && _armRightLifted) {
        _setpoints.velocityRight += _config.extraPull.setpointVelocity;
    }
}

void ballHandlingControl::determineRobotHasBall() {
    // Always release ball is none of the ball handlers have the ball
    if ((_angleLeftFraction < _config.ballPossession.angleThresholdOff) && (_angleRightFraction < _config.ballPossession.angleThresholdOff))
    {
        _ballPossession = false;
    }

    if ((_angleLeftFraction < _config.ballPossession.angleThresholdOff) || (_angleRightFraction < _config.ballPossession.angleThresholdOff))
    {
        start_time = high_resolution_clock::now();
    }

    // if ballPoss == False
    //      and
    //      angles are UP
    //      and
    //      timer inactive
    //  then
    //      start timer
    //
    // if ballPoss == False
    //      and
    //      angles are UP
    //      and
    //      timer expired
    //  then
    //      ballPoss = True
    //
    // if one angle is down
    //  then
    //      set timer inactive
    //

    // Always ball if both ball handlers have the ball
    if ((_angleLeftFraction > _config.ballPossession.angleThresholdOn) && (_angleRightFraction > _config.ballPossession.angleThresholdOn))
    {
        if ((high_resolution_clock::now() - start_time) > milliseconds(int(1e3 * _config.ballPossession.minimumTimeUp)))
        {
            _ballPossession = true;
        }
    }
}

void ballHandlingControl::addVelocityFeedForward()
{
    if (_armLeftLifted && _armRightLifted)
    {
        _setpoints.velocityLeft += _robotVelocity.x * -_config.feedForward.factorX +
                _robotVelocity.y * _config.feedForward.factorY +
                _robotVelocity.phi * _config.feedForward.factorRz;
        _setpoints.velocityRight += _robotVelocity.x * _config.feedForward.factorX +
                _robotVelocity.y * _config.feedForward.factorY +
                _robotVelocity.phi * -_config.feedForward.factorRz;
    }
}

DiagBallHandling ballHandlingControl::makeDiagnostics()
{
    DiagBallHandling result;
    result.left.enabled = _enabled;
    result.left.lifted = _armLeftLifted;
    result.left.angleFraction = _angleLeftFraction;
    result.left.velocitySetpoint = _setpoints.velocityLeft;
    result.right.enabled = _enabled;
    result.right.lifted = _armRightLifted;
    result.right.angleFraction = _angleRightFraction;
    result.right.velocitySetpoint = _setpoints.velocityRight;
    result.ballPossession = _ballPossession;
    return result;
}

void ballHandlingControl::traceData()
{
    _rtdbOutputAdapter->setDiagnostics(makeDiagnostics());
}

//void Ballhandlers::addAngleDiffToFeedForwardVelocity() { ### UNUSED
//
//    float leftAngle = _leftBallhandler.data.ballhandler.angle;
//    float rightAngle = _rightBallhandler.data.ballhandler.angle;
//
//    if (leftAngle > rightAngle) {
//        _rightBhVelocity += (leftAngle - rightAngle);
//    } else if (rightAngle > leftAngle) {
//        _leftBhVelocity += (rightAngle - leftAngle);
//    }
//}
//
//void Ballhandlers::calculateFeedForwardVelocity( ### UNUSED
//        double xVelocityEnc, double yVelocityEnc, double thetaVelocityEnc) {
//
//    float dist_ball2robot = 0.26;
//    float distn_ball2wheel_x = 0.06;
//    float distn_ball2wheel_y = 0.058;
//    float r_ball = 0.11;
//    float r_wheel = 0.028;
//    float angle_bh = 0.52;
//
//    float ball_dx = -xVelocityEnc + dist_ball2robot * thetaVelocityEnc;
//    float ball_dy = yVelocityEnc;
//    float ball_M = std::sqrt(pow(ball_dx, 2.0) + pow(ball_dy, 2.0));
//    //# dy can be zero ! divide by zero. -> atan2( , )
//    float ball_theta = std::atan2(ball_dx, ball_dy);
//    float ball2wheel_M = std::sqrt(
//        	pow(distn_ball2wheel_x, 2.0) + pow(distn_ball2wheel_y, 2.0));
//    float ball2wheel_theta = std::atan(distn_ball2wheel_x / distn_ball2wheel_y);
//
//    // Velocity wheel Left:
//    float Wheel_vel_left = (ball_M / (r_ball * r_wheel))
//        	* std::sqrt(pow(r_ball, 2.0) - pow(ball2wheel_M, 2.0) * pow(std::sin(ball2wheel_theta - ball_theta), 2.0))
//        	* std::cos(angle_bh - ball_theta);
//    _leftBhVelocity = -(Wheel_vel_left * r_wheel);
//    //# Feedforward BH speed [m/s]
//    float Wheel_vel_right = (ball_M / (r_ball * r_wheel))
//        	* std::sqrt(pow(r_ball, 2.0) - pow(ball2wheel_M, 2) * pow(std::cos(M_PI_2 - (ball2wheel_theta - ball_theta)), 2.0))
//        			* std::cos(angle_bh + ball_theta);
//    _rightBhVelocity = -(Wheel_vel_right * r_wheel);
//}
//
//    // STEP to override the bh velocity in case when any one of the handle is engaged with the ballls
//    // do not enable ki parameter for angle since the angle error will pile up over time until BH has balls!
//    // if ki of BH angle is enabled the following if else conditions will not work properly
//
//    if (_leftArmLifted && !_rightArmLifted) {
//        _leftBhVelocity = 300;
//    } else if (!_leftArmLifted && _rightArmLifted) {
//        _rightBhVelocity = 300;
//    }
//
