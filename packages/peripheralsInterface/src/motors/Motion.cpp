 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"

#include "int/motors/DeviceManager.hpp"
#include "int/motors/MotionBoard.hpp"
#include "int/motors/Motion.hpp"

using namespace std;

Motion::Motion(
        PeripheralsInterfaceData &piData, MotionBoard& leftMotionBoard, MotionBoard& rightMotionBoard, MotionBoard& rearMotionBoard) :
                _piData(piData), _leftMotionBoard(leftMotionBoard), _rightMotionBoard(rightMotionBoard), _rearMotionBoard(rearMotionBoard) {
    _watchDogCycles = 100;

    _leftMotor.name = "Left";
    _leftMotor.online = true;
    _rightMotor.name = "Right";
    _rightMotor.online = true;
    _rearMotor.name = "Rear";
    _rearMotor.online = true;
}

Motion::~Motion() {

}

void Motion::update() {
    watchDog();

    updateBoardSetpoints();

    updateBoardData();

    calculateVelocityAndDisplacementOutput();

    calculateEncodersInput();

    updateStatus();

    traceData();
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

/*
 * This function does the following:
 * 1) If motion board settings changed, send new settings
 * 2) Get the latest motion board data
 */
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
    _leftMotor.data.motorController.setpoint = _leftMotor.setpoint / newVelocityToEncoderFactor;
    _rightMotor.data.motorController.setpoint = _rightMotor.setpoint / newVelocityToEncoderFactor;
    _rearMotor.data.motorController.setpoint = _rearMotor.setpoint / newVelocityToEncoderFactor;

    _piData.getLeftMotionBoard().setDataOutput(_leftMotor.data);
    _piData.getRightMotionBoard().setDataOutput(_rightMotor.data);
    _piData.getRearMotionBoard().setDataOutput(_rearMotor.data);
}

/*
 * This function does the following:
 * 1) Set the motor velocity setpoint to the motion board
 */
void Motion::updateBoardSetpoints() {

    _leftMotionBoard.setSetpoint(_leftMotor.setpoint);
    _rightMotionBoard.setSetpoint(_rightMotor.setpoint);
    _rearMotionBoard.setSetpoint(_rearMotor.setpoint);
}

/*
 * This function does the following:
 * 1) Grab the latest motor velocity setpoints from _piData
 * 2) Set motor velocity setpoints to the motors
 */
void Motion::calculateEncodersInput() {
    // Compute new encoder values based on velocity
    piVelAcc vel = _piData.getVelocityInput();

    _leftMotor.setpoint = vel.m1_vel * newVelocityToEncoderFactor;
    _rightMotor.setpoint = vel.m2_vel * newVelocityToEncoderFactor;
    _rearMotor.setpoint = vel.m3_vel * newVelocityToEncoderFactor;
}

/*
 * This function does the following:
 * 1) Use the motion board data to compute the motor's velocity and displacement (based on encoder ticks)
 */
void Motion::calculateVelocityAndDisplacementOutput() {

    piVelAcc vel;
    piDisplacement displacement;

    double leftMotorVelocity = _leftMotor.data.motion.velocity * newEncoderToVelocityFactor;
    double leftMotorTicks = _leftMotor.data.motion.displacementEncTicks;
    double leftMotorDisplacement = leftMotorTicks * oneWheelTick;

    double rightMotorVelocity = _rightMotor.data.motion.velocity * newEncoderToVelocityFactor;
    double rightMotorTicks = _rightMotor.data.motion.displacementEncTicks;
    double rightMotorDisplacement = rightMotorTicks * oneWheelTick;

    double rearMotorVelocity = _rearMotor.data.motion.velocity * newEncoderToVelocityFactor;
    double rearMotorTicks = _rearMotor.data.motion.displacementEncTicks;
    double rearMotorDisplacement = rearMotorTicks * oneWheelTick;

    vel.m1_vel = leftMotorVelocity;
    vel.m2_vel = rightMotorVelocity;
    vel.m3_vel = rearMotorVelocity;

    displacement.m1_pos = leftMotorDisplacement;
    displacement.m2_pos = rightMotorDisplacement;
    displacement.m3_pos = rearMotorDisplacement;

    _piData.setVelocityOutput(vel);
    _piData.setDisplacementOutput(displacement);
}

void Motion::traceData()
{
    // tprintf would produce quite some data to stdout
    // instead we should use .rdl logging and related diagnostics tools

    //tprintf("KSTM %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
    //    _leftMotor.setpoint, _leftMotor.data.motion.velocity, _leftMotor.data.motion.pidOutput, _leftMotor.data.motion.error, _leftMotor.data.motion.integral, _leftMotor.data.motorController.pwm,
    //    _rightMotor.setpoint, _rightMotor.data.motion.velocity, _rightMotor.data.motion.pidOutput, _rightMotor.data.motion.error, _rightMotor.data.motion.integral, _rightMotor.data.motorController.pwm,
    //    _rearMotor.setpoint, _rearMotor.data.motion.velocity, _rearMotor.data.motion.pidOutput, _rearMotor.data.motion.error, _rearMotor.data.motion.integral, _rearMotor.data.motorController.pwm);
}
