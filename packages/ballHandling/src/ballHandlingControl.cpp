 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

ballHandlingControl::ballHandlingControl(cRTDBOutputAdapter& rtdbOutputAdapter)
{
    _rtdbOutputAdapter = &rtdbOutputAdapter;

    _enabled = true;
    _ballPossession = false;
    _firstTime = true;

    _angleLeftFraction = 0.0;
    _angleRightFraction = 0.0;
    
    start_time = high_resolution_clock::now();
}

ballHandlingControl::~ballHandlingControl()
{
}

void ballHandlingControl::set_settings(ballHandlingSettingsType settings)
{
    _settings = settings;
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
    calculateSetpoints();
    traceData();

    _rtdbOutputAdapter->setBallHandlersMotorSetpoint(_enabled, _setpoints);
}

void ballHandlingControl::updateFeedback()
{
    calculateAngleFractions();
    if (_firstTime)
    {
        // ticket #74: robot does not have the ball so we expect the angle to be close to the configured downangle, if not raise a warning
        if ((abs(_status.angleLeft - _settings.leftArmCalibrationDown) > _calibWarning) or (abs(_status.angleRight - _settings.rightArmCalibrationDown) > _calibWarning))
            {
            TRACE_WARNING("Offset between calibrated and measured down position larger than %d, recalibrate ballHandlers", _calibWarning);
            }
        _firstTime = false;
    }
    determineRobotHasBall();
    traceData();

    _rtdbOutputAdapter->setBallHandlersBallPossession(_ballPossession);

    WRITE_TRACE;
}

void ballHandlingControl::calculateAngleFractions()
{
    // Calculate for each arm what the fraction of the arm is (based on the minimum and maximum angle).
    _angleLeftFraction = calculateAngleFraction(_status.angleLeft, _settings.leftArmCalibrationDown, _settings.leftArmCalibrationUp);
    _angleRightFraction = calculateAngleFraction(_status.angleRight, _settings.rightArmCalibrationDown, _settings.rightArmCalibrationUp);
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
    _setpoints.angleLeft = calculateAngle(_settings.angleSetpoint, _settings.leftArmCalibrationDown, _settings.leftArmCalibrationUp);
    _setpoints.angleRight= calculateAngle(_settings.angleSetpoint, _settings.rightArmCalibrationDown, _settings.rightArmCalibrationUp);
    _setpoints.velocityLeft = 0;
    _setpoints.velocityRight = 0;

    // Check if ballHandlers are disabled, which is typically done by teamplay via motionPlanning in game state STOP or DROPBALL.
    if (_enabled)
    {
        // Calculate setpoints only if ballhandlers are enabled.

        if (_settings.enableExtraPullForceWhenSingleArmUp)
        {
            addExtraPullForceWhenOneArmLifted();
        }

        if (_settings.enableRobotVelocityFeedForward)
        {
            addVelocityFeedForward();
        }
    }
}

void ballHandlingControl::addExtraPullForceWhenOneArmLifted() {
    bool _armLeftLifted = _angleLeftFraction > _settings.armLiftedAngleThreshold;
    bool _armRightLifted = _angleRightFraction > _settings.armLiftedAngleThreshold;

    if (_armLeftLifted && !_armRightLifted) {
        _setpoints.velocityLeft += _settings.extraPullForceVelocity;
    } else if (!_armLeftLifted && _armRightLifted) {
        _setpoints.velocityRight += _settings.extraPullForceVelocity;
    }
}

void ballHandlingControl::determineRobotHasBall() {
    // Always release ball is none of the ball handlers have the ball
    if ((_angleLeftFraction < _settings.ballPossessionAngleThresholdOff) && (_angleRightFraction < _settings.ballPossessionAngleThresholdOff))
    {
        _ballPossession = false;
    }

    if ((_angleLeftFraction < _settings.ballPossessionAngleThresholdOff) || (_angleRightFraction < _settings.ballPossessionAngleThresholdOff))
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
    if ((_angleLeftFraction > _settings.ballPossessionAngleThresholdOn) && (_angleRightFraction > _settings.ballPossessionAngleThresholdOn))
    {
        if ((high_resolution_clock::now() - start_time) > milliseconds(_settings.ballPossessionTimeInterval))
        {
            _ballPossession = true;
        }
    }
}

void ballHandlingControl::addVelocityFeedForward()
{
    bool _armLeftLifted = _angleLeftFraction > _settings.armLiftedAngleThreshold;
    bool _armRightLifted = _angleRightFraction > _settings.armLiftedAngleThreshold;

    if ((_armLeftLifted && _armRightLifted) || !_settings.disableRobotVelocityFeedForwardWhenArmsNotUp)
    {
        _setpoints.velocityLeft += _robotVelocity.x * -_settings.xFeedForwardVelocityFactor +
                _robotVelocity.y * _settings.yFeedForwardVelocityFactor +
                _robotVelocity.phi * _settings.thetaFeedForwardVelocityFactor;
        _setpoints.velocityRight += _robotVelocity.x * _settings.xFeedForwardVelocityFactor +
                _robotVelocity.y * _settings.yFeedForwardVelocityFactor +
                _robotVelocity.phi * -_settings.thetaFeedForwardVelocityFactor;
    }
}

void ballHandlingControl::traceData()
{
    // PTRACE("%1.6f %1.6f %d %d %d %d %d %d",
    //         _angleLeftFraction, _angleRightFraction, _enabled, _ballPossession,
    //    	_angleLeftFraction < _settings.ballPossessionAngleThresholdOff,
    //    	_angleRightFraction < _settings.ballPossessionAngleThresholdOff,
    //    	_angleLeftFraction > _settings.ballPossessionAngleThresholdOn,
    //    	_angleRightFraction > _settings.ballPossessionAngleThresholdOn);
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
