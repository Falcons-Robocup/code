 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PeripheralsInterfaceBallHandlers.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: Prabhu Mani
 */

#define _USE_MATH_DEFINES

#include <chrono>
#include <cmath>

#include "cDiagnosticsEvents.hpp"

#include "int/Ballhandlers.hpp"

#include "int/DeviceManager.hpp"

// TODO:calculate correct value
static const float velocityToEncoderValue = 50;
static const float hasBallTimeout = 0; // JFEI/EKPC what was the idea behind 500ms here? this spoils responsiveness when getting ball; we already have camera check; so we disable.

//TODO: REMOVE REMOVE REMOVE: Somehow the encoder to velocity value for the motion motors is used here :S?
static const float velocityToMotionEncoderValue = 155.053664073;

Ballhandlers::Ballhandlers(
		PeripheralsInterfaceData &piData, BallhandlerBoard& leftBallhandlerBoard, BallhandlerBoard& rightBallhandlerBoard) :
				_piData(piData), _leftBallhandlerBoard(leftBallhandlerBoard), _rightBallhandlerBoard(rightBallhandlerBoard) {
	bothBallhandlersDownTime = std::chrono::steady_clock::now();

	// Set online status of ballhandlers initially to true. An error will be logged when these are not available.
	_leftBallhandler.online = true;
	_rightBallhandler.online = true;
}

Ballhandlers::~Ballhandlers() {

}

void Ballhandlers::start() {
	_updateThread = thread(&Ballhandlers::update, this);
}

void Ballhandlers::update() {

	while (true) {
		// Calculate the time when the next iteration should be started.
		chrono::system_clock::time_point end_time = chrono::system_clock::now() + chrono::milliseconds(100);

		updateBoardData();

		determineRobotHasBall();

		setVelocityInput();

		updateBoardSetpoints();

		updateBoardStatus();

		this_thread::sleep_until(end_time);
	}
}

void Ballhandlers::updateBoardStatus() {
	_piData.getLeftBallhandlerBoard().setOnline(_leftBallhandlerBoard.isConnected());
	_piData.getRightBallhandlerBoard().setOnline(_rightBallhandlerBoard.isConnected());
}

void Ballhandlers::updateBoardData() {

	if (_piData.getLeftBallhandlerBoard().isSettingsChanged()) {
		_leftBallhandlerBoard.setSettings(_piData.getLeftBallhandlerBoard().getSettings());
	}
	if (_piData.getRightBallhandlerBoard().isSettingsChanged()) {
		_rightBallhandlerBoard.setSettings(_piData.getRightBallhandlerBoard().getSettings());
	}

	if (_piData.isBallhandlerSettingsChanged()) {
		_leftBallhandlerBoard.setControlMode(_piData.getBallhandlerSettings().controlMode);
		_rightBallhandlerBoard.setControlMode(_piData.getBallhandlerSettings().controlMode);
	}

	_leftBallhandler.data = _leftBallhandlerBoard.getBoardData();
	_rightBallhandler.data = _rightBallhandlerBoard.getBoardData();

	_piData.getLeftBallhandlerBoard().setDataOutput(_leftBallhandler.data);
	_piData.getRightBallhandlerBoard().setDataOutput(_rightBallhandler.data);
}

void Ballhandlers::updateBoardSetpoints() {

	float angle = _piData.getBallhandlerAngle();
	_leftBallhandlerBoard.setSetpoint(angle, _leftBallhandler.setpoint);
	_rightBallhandlerBoard.setSetpoint(angle, _rightBallhandler.setpoint);
}

/* TODO: CRUFT starts here, cleanup is needed and new feed-forward algorithm. */
void Ballhandlers::calculatePassBallVelocity(
		double &leftBhVelocity, double &rightBhVelocity,
		passBall passBallParams) {
	//spins the BH motors such that the ball is pushed away with a given speed

	//for a push pass
	//passBallParams speed is * -1 as the ball has to spin in reverse direction to push
	leftBhVelocity = leftBhVelocity + (-1 * passBallParams.bhLeftSpeed);
	rightBhVelocity = rightBhVelocity + (-1 * passBallParams.bhRightSpeed);

	//for left spinned pass. passBallParams.mode == LEFT_SPIN_PASS?
	//leftBhVelocity = 0;
	//rightBhVelocity = rightBhVelocity + passBallParams.bhSpeedRight;

}

void Ballhandlers::setVelocityInput() {
//	piVelAcc velocity = _piData.getVelocityInput();
	piVelAcc velocity = _piData.getVelocityOutput();

	double leftBhVelocity = 0;
	double rightBhVelocity = 0;

	double xVelocity = velocity.vel.x;
	double yVelocity = velocity.vel.y;
	double thetaVelocity = velocity.vel.phi;

	//STEP to calculate feedforward velocity
//	calculateFeedForwardVelocity(xVelocity, yVelocity, thetaVelocity,
//			leftBhVelocity, rightBhVelocity);

	BallhandlerBoardSettings ballhandlerBoardSettingsLeft = _piData.getLeftBallhandlerBoard().getSettings();
	BallhandlerBoardSettings ballhandlerBoardSettingsRight = _piData.getRightBallhandlerBoard().getSettings();

	bool leftArmLifted = calculateArmLifted(_leftBallhandler.data, ballhandlerBoardSettingsLeft);
	bool rightArmLifted = calculateArmLifted(_rightBallhandler.data, ballhandlerBoardSettingsRight);
	TRACE("leftArmLifted=%d rightArmLifted=%d", leftArmLifted, rightArmLifted);

	// STEP to override the bh velocity in case when any one of the handle is engaged with the ballls
	// do not enable ki parameter for angle since the angle error will pile up over time until BH has balls!
	// if ki of BH angle is enabled the following if else conditions will not work properly

	if (leftArmLifted && !rightArmLifted) {
		leftBhVelocity = 300;
	} else if (!leftArmLifted && rightArmLifted) {
		rightBhVelocity = 300;
	}

	_leftBallhandler.setpoint = leftBhVelocity * velocityToEncoderValue;
	_rightBallhandler.setpoint = rightBhVelocity * velocityToEncoderValue;
}

bool Ballhandlers::calculateArmLifted(const BallhandlerBoardDataOutput &data,
		const BallhandlerBoardSettings &settings) {
	bool armLifted;

	int ballPossAngleVal = data.ballhandler.angleZero +
			(((settings.maxAngle - data.ballhandler.angleZero) / 100) * settings.ballPossessionTreshold);

	armLifted = (data.ballhandler.angle > ballPossAngleVal) ? true : false;

	return armLifted;
}

void Ballhandlers::addAngleDiffToFeedForwardVelocity(
		double &leftBhVelocity, double &rightBhVelocity) {

	float leftAngle = _leftBallhandler.data.ballhandler.angle;
	float rightAngle = _rightBallhandler.data.ballhandler.angle;

	if (leftAngle > rightAngle) {
		rightBhVelocity += (leftAngle - rightAngle);
	} else if (rightAngle > leftAngle) {
		leftBhVelocity += (rightAngle - leftAngle);
	}
}

void Ballhandlers::determineRobotHasBall() {
	// Determine if the robot has the ball

	BallhandlerBoardSettings ballhandlerBoardSettingsLeft = _piData.getLeftBallhandlerBoard().getSettings();
	BallhandlerBoardSettings ballhandlerBoardSettingsRight = _piData.getRightBallhandlerBoard().getSettings();

	bool leftArmLifted = calculateArmLifted(_leftBallhandler.data, ballhandlerBoardSettingsLeft);
	bool rightArmLifted = calculateArmLifted(_rightBallhandler.data, ballhandlerBoardSettingsRight);

	// Time the instance that both ballhandlers are up. Only assume we have a good
	// grip on the ball if the ballhandlers stay up for > 1 second.
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

	// Always release ball is none of the ball handlers have the ball
	if ((!leftArmLifted) && (!rightArmLifted)) {
		bothBallhandlersDownTime = now;
		_piData.setHasBall(false);
	}
	// It happens occasionally that only one ballhandler has the ball
	// in which case robot may decide to shoot...
	// Discussed with Stan Mertens -> we currently cannot shoot accurately if not engaged by both
	// ballhandlers. For now, we take this as a requirement.
//	if (!(leftArmLifted && rightArmLifted))
//	{
//		bothBallhandlersDownTime = now;
//		_piData.setHasBall(false);
//	}

	// Always ball if both ball handlers have the ball
	if (leftArmLifted && rightArmLifted) {
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - bothBallhandlersDownTime).count() > hasBallTimeout) {
		    TRACE("we have the ball");
			_piData.setHasBall(true);
		}
	}
}
//
void Ballhandlers::calculateFeedForwardVelocity(
		double xVelocityEnc, double yVelocityEnc, double thetaVelocityEnc,
		double &leftBhVelocity, double &rightBhVelocity) {

	float dist_ball2robot = 0.26;
	float distn_ball2wheel_x = 0.06;
	float distn_ball2wheel_y = 0.058;
	float r_ball = 0.11;
	float r_wheel = 0.028;
	float angle_bh = 0.52;

	float ball_dx = -xVelocityEnc + dist_ball2robot * thetaVelocityEnc;
	float ball_dy = yVelocityEnc;
	float ball_M = std::sqrt(pow(ball_dx, 2.0) + pow(ball_dy, 2.0));
	//# dy can be zero ! divide by zero. -> atan2( , )
	float ball_theta = std::atan2(ball_dx, ball_dy);
	float ball2wheel_M = std::sqrt(
			pow(distn_ball2wheel_x, 2.0) + pow(distn_ball2wheel_y, 2.0));
	float ball2wheel_theta = std::atan(distn_ball2wheel_x / distn_ball2wheel_y);

	// Velocity wheel Left:
	float Wheel_vel_left = (ball_M / (r_ball * r_wheel))
			* std::sqrt(pow(r_ball, 2.0) - pow(ball2wheel_M, 2.0) * pow(std::sin(ball2wheel_theta - ball_theta), 2.0))
			* std::cos(angle_bh - ball_theta);
	leftBhVelocity = -(Wheel_vel_left * r_wheel);
	//# Feedforward BH speed [m/s]
	float Wheel_vel_right = (ball_M / (r_ball * r_wheel))
			* std::sqrt(pow(r_ball, 2.0) - pow(ball2wheel_M, 2) * pow(std::cos(M_PI_2 - (ball2wheel_theta - ball_theta)), 2.0))
					* std::cos(angle_bh + ball_theta);
	rightBhVelocity = -(Wheel_vel_right * r_wheel);
}
