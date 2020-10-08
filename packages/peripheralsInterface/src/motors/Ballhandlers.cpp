 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PeripheralsInterfaceBallHandlers.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: Robocup
 */

#define _USE_MATH_DEFINES

#include <cmath>

#include "cDiagnostics.hpp"
#include "tracing.hpp"

#include "int/motors/Ballhandlers.hpp"

#include "int/motors/DeviceManager.hpp"

// TODO:calculate correct value
static const float velocityToEncoderValue = 50;

Ballhandlers::Ballhandlers(
        PeripheralsInterfaceData &piData, BallhandlerBoard& leftBallhandlerBoard, BallhandlerBoard& rightBallhandlerBoard) :
        		_piData(piData), _leftBallhandlerBoard(leftBallhandlerBoard), _rightBallhandlerBoard(rightBallhandlerBoard) {

    // Set online status of ballhandlers initially to true. An error will be logged when these are not available.
    _leftBallhandler.online = true;
    _rightBallhandler.online = true;
    _leftEnabled = false;
    _rightEnabled = false;
}

Ballhandlers::~Ballhandlers() {

}

void Ballhandlers::update() {
    updateBoardSettings();

    updateControlMode();

    updateBoardSetpoints();

    updateBoardData();

    updateBoardStatus();

    traceData();
}

void Ballhandlers::updateBoardSettings() {
    if (_piData.getLeftBallhandlerBoard().isSettingsChanged()) {
        _leftBallhandlerBoard.setSettings(_piData.getLeftBallhandlerBoard().getSettings());
    }
    if (_piData.getRightBallhandlerBoard().isSettingsChanged()) {
        _rightBallhandlerBoard.setSettings(_piData.getRightBallhandlerBoard().getSettings());
    }
}

void Ballhandlers::updateControlMode() {
    // update disabled/enabled state of both BallhandlerBoard objects
    BallhandlerBoardControlMode controlModeSetpoint = _piData.getBallhandlerSettings().controlMode;
    BallhandlerBoardControlMode controlModeCurrentLeft = _leftBallhandlerBoard.getControlMode();
    BallhandlerBoardControlMode controlModeCurrentRight = _rightBallhandlerBoard.getControlMode();

    // TODO: either make sure this does not cause one ballHandler to disable before the other, or schedule parallel
    if (controlModeCurrentLeft != controlModeSetpoint)
    {
        TRACE("left set: current=%d setpoint=%d", (int)controlModeCurrentLeft, (int)controlModeSetpoint);
        _leftBallhandlerBoard.setControlMode(controlModeSetpoint);
    }
    _leftEnabled = (controlModeCurrentLeft == BALLHANDLER_CONTROL_MODE_ON);

    if (controlModeCurrentRight != controlModeSetpoint)
    {
        TRACE("right set: current=%d setpoint=%d", (int)controlModeCurrentRight, (int)controlModeSetpoint);
        _rightBallhandlerBoard.setControlMode(controlModeSetpoint);
    }
    _rightEnabled = (controlModeCurrentRight == BALLHANDLER_CONTROL_MODE_ON);
}

void Ballhandlers::updateBoardData() {
    _leftBallhandler.data = _leftBallhandlerBoard.getBoardData();
    _rightBallhandler.data = _rightBallhandlerBoard.getBoardData();

    _piData.getLeftBallhandlerBoard().setDataOutput(_leftBallhandler.data);
    _piData.getRightBallhandlerBoard().setDataOutput(_rightBallhandler.data);

    BallhandlerFeedback feedback;
    feedback.angleLeft = _leftBallhandler.data.ballhandler.angle;
    feedback.angleRight = _rightBallhandler.data.ballhandler.angle;
    feedback.velocityLeft = _leftBallhandler.data.ballhandler.tacho / velocityToEncoderValue;
    feedback.velocityRight = _rightBallhandler.data.ballhandler.tacho / velocityToEncoderValue;
    _piData.setBallhandlerFeedback(feedback);
}

void Ballhandlers::updateBoardSetpoints() {
    BallhandlerSetpoints setpoints = _piData.getBallhandlerSetpoints();
    _leftBallhandler.setpoint = setpoints.velocityLeft * velocityToEncoderValue;
    _rightBallhandler.setpoint = setpoints.velocityRight * velocityToEncoderValue;

    _leftBallhandlerBoard.setSetpoint(setpoints.angleLeft, _leftBallhandler.setpoint);
    _rightBallhandlerBoard.setSetpoint(setpoints.angleRight, _rightBallhandler.setpoint);
}

void Ballhandlers::updateBoardStatus() {
    _piData.getLeftBallhandlerBoard().setOnline(_leftBallhandlerBoard.isConnected());
    _piData.getRightBallhandlerBoard().setOnline(_rightBallhandlerBoard.isConnected());
}

void Ballhandlers::traceData()
{
    // tprintf would produce quite some data to stdout
    // instead we should use .rdl logging and related diagnostics tools

    //tprintf("KSTB %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f",
    //    	_leftBallhandler.setpoint, _rightBallhandler.setpoint,
    //    	_leftBallhandler.data.ballhandler.angle, _rightBallhandler.data.ballhandler.angle,
    //    	_leftBallhandler.data.ballhandler.tacho, _rightBallhandler.data.ballhandler.tacho,
    //    	_leftBallhandler.data.motorController.error, _rightBallhandler.data.motorController.error,
    //    	_leftBallhandler.data.motorController.integral, _rightBallhandler.data.motorController.integral,
    //    	_leftBallhandler.data.motorController.pidOutput, _rightBallhandler.data.motorController.pidOutput,
    //    	_leftBallhandler.data.motorController.setpoint, _rightBallhandler.data.motorController.setpoint,
    //    	_leftBallhandler.data.motorController.pwm, _rightBallhandler.data.motorController.pwm);
}
