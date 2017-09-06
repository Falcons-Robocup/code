 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Motion.cpp
 *
 *  Created on: Jun 30, 2016
 *      Author: Robocup
 */

#include <chrono>
#include <thread>

#include "FalconsCommon.h"

#include "cDiagnosticsEvents.hpp"

#include "int/DeviceManager.hpp"
#include "int/motors/MotionBoard.hpp"
#include "int/Motion.hpp"

using namespace std;

Motion::Motion(
		PeripheralsInterfaceData &piData, MotionBoard& leftMotionBoard, MotionBoard& rightMotionBoard, MotionBoard& rearMotionBoard) :
				_piData(piData), _leftMotionBoard(leftMotionBoard), _rightMotionBoard(rightMotionBoard), _rearMotionBoard(rearMotionBoard) {
	_watchDogCycles = 100;
	_totalDisplX = 0;
	_totalDisplY = 0;
	_totalDisplPhi = 0;

	_leftMotor.name = "Left";
	_leftMotor.online = true;
	_rightMotor.name = "Right";
	_rightMotor.online = true;
	_rearMotor.name = "Rear";
	_rearMotor.online = true;
}

Motion::~Motion() {

}

void Motion::start() {
	_updateThread = thread(&Motion::update, this);
}

void Motion::update() {

	while (true) {
		// Calculate the time when the next iteration should be started.
		chrono::system_clock::time_point end_time = chrono::system_clock::now() + chrono::milliseconds(10);

		watchDog();

		if (_piData.isMotionSettingsChanged())
		{
			_motionSettings = _piData.getMotionSettings();
		}

		updateBoardData();

		calculateRobotVelocityAndDisplacementOutput();

		calculateEncodersInput();

		updateBoardSetpoints();

		updateStatus();

		this_thread::sleep_until(end_time);
	}
}

void Motion::updateStatus() {

	_piData.getLeftMotionBoard().setOnline(_leftMotionBoard.isConnected());
	_piData.getRightMotionBoard().setOnline(_rightMotionBoard.isConnected());
	_piData.getRearMotionBoard().setOnline(_rearMotionBoard.isConnected());
}

void Motion::watchDog() {
	if (_piData.isVelocityInputChanged()) {
		_watchDogCycles = 100;
	}

	if (_watchDogCycles > 0) {
		_watchDogCycles--;
	}
	else {
		_piData.setVelocityInput(piVelAcc());
	}
}

void Motion::updateBoardData() {

	if (_piData.getLeftMotionBoard().isSettingsChanged()) {
		_leftMotionBoard.setSettings(_piData.getLeftMotionBoard().getSettings());
	}
	if (_piData.getRightMotionBoard().isSettingsChanged()) {
		_rightMotionBoard.setSettings(_piData.getRightMotionBoard().getSettings());
	}
	if (_piData.getRearMotionBoard().isSettingsChanged()) {
		_rearMotionBoard.setSettings(_piData.getRearMotionBoard().getSettings());
	}

	_leftMotor.data = _leftMotionBoard.getBoardData();
	_rightMotor.data = _rightMotionBoard.getBoardData();
	_rearMotor.data = _rearMotionBoard.getBoardData();

	// Update output data
	_leftMotor.data.motorController.setpoint = _leftMotor.setpoint / velocityToEncoderValue;
	_rightMotor.data.motorController.setpoint = _rightMotor.setpoint / velocityToEncoderValue;
	_rearMotor.data.motorController.setpoint = _rearMotor.setpoint / velocityToEncoderValue;

	_piData.getLeftMotionBoard().setDataOutput(_leftMotor.data);
	_piData.getRightMotionBoard().setDataOutput(_rightMotor.data);
	_piData.getRearMotionBoard().setDataOutput(_rearMotor.data);
}

void Motion::updateBoardSetpoints() {

	_leftMotionBoard.setSetpoint(_leftMotor.setpoint);
	_rightMotionBoard.setSetpoint(_rightMotor.setpoint);
	_rearMotionBoard.setSetpoint(_rearMotor.setpoint);
}

void Motion::calculateEncodersInput() {
	// Compute new encoder values based on velocity
	piVelAcc vel = _piData.getVelocityInput();

	boost::numeric::ublas::vector<double> v(3);
	v(0) = vel.vel.x;
	v(1) = vel.vel.y;
	v(2) = vel.vel.phi;

	boost::numeric::ublas::vector<double> m = prod(_motionSettings.matrix, v);

	_leftMotor.setpoint = m(0) * velocityToEncoderValue;
	_rightMotor.setpoint = m(1) * velocityToEncoderValue;
	_rearMotor.setpoint = m(2) * velocityToEncoderValue;
}

void Motion::calculateRobotVelocityAndDisplacementOutput() {

	piVelAcc vel;
	piDisplacement displacement;

	double leftMotorVelocity = _leftMotor.data.motion.velocity * newEncoderToVelocityFactor;
	double leftMotorTicks = _leftMotor.data.motion.displacementEncTicks;
	double leftMotorDisplacement = (leftMotorTicks - _leftMotor.prevTicks ) * oneWheelTick;
	_leftMotor.prevTicks = leftMotorTicks;

	double rightMotorVelocity = _rightMotor.data.motion.velocity * newEncoderToVelocityFactor;
	double rightMotorTicks = _rightMotor.data.motion.displacementEncTicks;
	double rightMotorDisplacement = (rightMotorTicks - _rightMotor.prevTicks) * oneWheelTick;
	_rightMotor.prevTicks = rightMotorTicks;

	double rearMotorVelocity = _rearMotor.data.motion.velocity * newEncoderToVelocityFactor;
	double rearMotorTicks = _rearMotor.data.motion.displacementEncTicks;
	double rearMotorDisplacement = (rearMotorTicks - _rearMotor.prevTicks) * oneWheelTick;
	_rearMotor.prevTicks = rearMotorTicks;

	boost::numeric::ublas::vector<double> mVelocity(3);
	mVelocity(0) = leftMotorVelocity;
	mVelocity(1) = rightMotorVelocity;
	mVelocity(2) = rearMotorVelocity;

	boost::numeric::ublas::vector<double> vVelocity = prod(_motionSettings.inverseMatrix, mVelocity);

	boost::numeric::ublas::vector<double> mDisplacement(3);
	mDisplacement(0) = leftMotorDisplacement;
	mDisplacement(1) = rightMotorDisplacement;
	mDisplacement(2) = rearMotorDisplacement;

	boost::numeric::ublas::vector<double> vDisplacement = prod(_motionSettings.inverseMatrix, mDisplacement);

	vel.vel.x = vVelocity(0);
	vel.vel.y = vVelocity(1);
	vel.vel.phi = vVelocity(2);
	displacement.pos = geometry::Pose2D(vDisplacement(0), vDisplacement(1), vDisplacement(2));

	_totalDisplX += displacement.pos.getX();
	_totalDisplY += displacement.pos.getY();
	_totalDisplPhi += displacement.pos.getPhi();

//	TRACE(
//			"PerDeb LeftMotVel: %5.5f RightMotVel: %5.5f RearMotVel: %5.5f LeftMotDispl: %5.5f RightMotDispl: %5.5f RearMotDispl: %5.5f xVel: %5.5f yVel: %5.5f phiVel: %5.5f xDispl: %5.5f yDispl: %5.5f phiDispl: %5.5f xTotDispl: %5.5f yTotDispl: %5.5f phiTotDispl: %5.5f LeftMotTicks: %5.5f RightMotTicks: %5.5f RearMotTicks: %5.5f",
//			leftMotorVelocity, rightMotorVelocity, rearMotorVelocity,
//			leftMotorDisplacement, rightMotorDisplacement,
//			rearMotorDisplacement, vel.vel.x, vel.vel.y, vel.vel.phi,
//			displacement.pos.getX(), displacement.pos.getY(),
//			displacement.pos.getPhi(), _totalDisplX, _totalDisplY,
//			_totalDisplPhi, leftMotorTicks, rightMotorTicks, rearMotorTicks);

	_piData.setVelocityOutput(vel);
	_piData.setDisplacementOutput(displacement);
}
