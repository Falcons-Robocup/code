// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Diagnostics.cpp
 *
 *  Created on: Nov 10, 2016
 *      Author: Edwin Schreuder
 */

#include "int/motors/Diagnostics.hpp"

#include <iostream>
#include <thread>

#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace std;

Diagnostics::Diagnostics(PeripheralsInterfaceData& piData, bool ballhandlersAvailable)
    : piData(piData)
{
	TRACE(">");

	started = false;
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(getRobotNumber());


	if (ballhandlersAvailable) {
		desiredNumberOfDevices = 5;
	}
	else {
		desiredNumberOfDevices = 3;
	}

	previousNumberOfDevices = desiredNumberOfDevices;

	numberOfOnlineBoards = desiredNumberOfDevices;
	previousLeftMotionBoardOnline = true;
	previousRightMotionBoardOnline = true;
	previousRearMotionBoardOnline = true;
	previousLeftBallhandlerBoard = true;
	previousRightBallhandlerBoard = true;

	TRACE("<");
}

void Diagnostics::start() {
	TRACE(">");

	started = true;
	diagnosticsThread = thread(&Diagnostics::timer, this);

	TRACE("<");
}

void Diagnostics::stop() {
	TRACE(">");

	started = true;
	diagnosticsThread = thread(&Diagnostics::timer, this);

	TRACE("<");
}

void Diagnostics::timer() {
	TRACE(">");

	cout << "INFO    : Starting diagnostics." << endl;

	while (started) {
		addSensorData();
		addStatus();

		this_thread::sleep_for(chrono::milliseconds(100));
	}

	TRACE("<");
}

void Diagnostics::addSensorData() {
	T_DIAG_PERIPHERALSINTERFACE msg;

	/////// DRIVE MOTORS

	piVelAcc v = piData.getVelocityInput();
	msg.velocity_setpoint[0] = v.m1_vel;
	msg.velocity_setpoint[1] = v.m2_vel;
	msg.velocity_setpoint[2] = v.m3_vel;

	v = piData.getVelocityOutput();
	msg.velocity_feedback[0] = v.m1_vel;
	msg.velocity_feedback[1] = v.m2_vel;
	msg.velocity_feedback[2] = v.m3_vel;

	MotionBoardDataOutput rearMotionBoardData = piData.getRearMotionBoard().getDataOutput();
	MotionBoardDataOutput rightMotionBoardData = piData.getRightMotionBoard().getDataOutput();
	MotionBoardDataOutput leftMotionBoardData = piData.getLeftMotionBoard().getDataOutput();

	msg.motion_temperature[0] = rightMotionBoardData.motion.motorTemperature;
	msg.motion_temperature[1] = rearMotionBoardData.motion.motorTemperature;
	msg.motion_temperature[2] = leftMotionBoardData.motion.motorTemperature;

	// voltage 
	voltageMonitor.feed(rearMotionBoardData.motorController.voltage);
	voltageMonitor.feed(leftMotionBoardData.motorController.voltage);
	voltageMonitor.feed(rightMotionBoardData.motorController.voltage);
	msg.voltage = voltageMonitor.get();

	// check if driving
	bool isDriving = false;
	if (fabs(v.m1_vel) > 0.1) isDriving = true;
	if (fabs(v.m2_vel) > 0.1) isDriving = true;
	if (fabs(v.m3_vel) > 0.1) isDriving = true;
	
	// check voltage, but do not generate warnings while driving
	if (!isDriving)
	{
		voltageMonitor.check();
	}

    // motor PID
    msg.motor_pid_output[0] = rightMotionBoardData.motion.pidOutput;
    msg.motor_pid_output[1] = rearMotionBoardData.motion.pidOutput;
    msg.motor_pid_output[2] = leftMotionBoardData.motion.pidOutput;

    // motor Error (PID)
    msg.motor_error[0] = rightMotionBoardData.motion.error;
    msg.motor_error[1] = rearMotionBoardData.motion.error;
    msg.motor_error[2] = leftMotionBoardData.motion.error;

    // motor Integral (PID)
    msg.motor_integral[0] = rightMotionBoardData.motion.integral;
    msg.motor_integral[1] = rearMotionBoardData.motion.integral;
    msg.motor_integral[2] = leftMotionBoardData.motion.integral;

    // motor Derivative (PID)
    msg.motor_derivative[0] = rightMotionBoardData.motion.derivative;
    msg.motor_derivative[1] = rearMotionBoardData.motion.derivative;
    msg.motor_derivative[2] = leftMotionBoardData.motion.derivative;



	/////// BALLHANDLERS

	BallhandlerBoardDataOutput bh_left_data = piData.getLeftBallhandlerBoard().getDataOutput();
	BallhandlerBoardDataOutput bh_right_data = piData.getRightBallhandlerBoard().getDataOutput();

	msg.bh_angle[0] = bh_left_data.ballhandler.angle;
	msg.bh_angle[1] = bh_right_data.ballhandler.angle;

	msg.bh_tacho_zero[0] = bh_left_data.ballhandler.tachoZero;
	msg.bh_tacho_zero[1] = bh_right_data.ballhandler.tachoZero;

	msg.bh_tacho[0] = bh_left_data.ballhandler.tacho;
	msg.bh_tacho[1] = bh_right_data.ballhandler.tacho;

	msg.bh_pid_output[0] = bh_left_data.motorController.pidOutput / 10000.0;
	msg.bh_pid_output[1] = bh_right_data.motorController.pidOutput / 10000.0;

	msg.bh_error[0] = bh_left_data.motorController.error;
	msg.bh_error[1] = bh_right_data.motorController.error;

	msg.bh_integral[0] = bh_left_data.motorController.integral;
	msg.bh_integral[1] = bh_right_data.motorController.integral;

	msg.bh_pwm[0] = bh_left_data.motorController.pwm;
	msg.bh_pwm[1] = bh_right_data.motorController.pwm;

    if (_rtdb != NULL)
    {
        _rtdb->put(DIAG_PERIPHERALSINTERFACE, &msg);
    }
}

void Diagnostics::addStatus() {

	size_t numberOfDevices = piData.getNumberOfConnectedDevices();

	if (numberOfDevices != previousNumberOfDevices) {
		logDeviceStatus(numberOfDevices);
		previousNumberOfDevices = numberOfDevices;
	}

	if (numberOfDevices > 0) {
		addBoardStatus();
	}
}

void Diagnostics::logDeviceStatus(size_t numberOfDevices) {

	if (numberOfDevices == 0) {
			// No USB converters connected.
			cerr << "ERROR   : No USB to Serial converters connected." << endl;
			TRACE_ERROR("No USB to Serial converters connected; USB problem?");
	}
	else if (numberOfDevices < desiredNumberOfDevices) {
		if ((previousNumberOfDevices >= numberOfDevices) || (previousNumberOfDevices == 0)) {
			cerr << "ERROR   : Not all (" << numberOfDevices << "/" << desiredNumberOfDevices << ") USB to Serial converters connected." << endl;
			TRACE_ERROR("Not all (%d/%d) USB to Serial converters connected; USB cable loose?",
					numberOfDevices, desiredNumberOfDevices);
		}
	}
	else {
		if (numberOfDevices >= desiredNumberOfDevices) {
			cout << "ERROR   : All USB to Serial converters connected." << endl;
			TRACE_INFO("All USB to Serial converters connected.");
		}
	}
}

void Diagnostics::addBoardStatus() {
	size_t previousNumberOfOnlineBoards = numberOfOnlineBoards;
	numberOfOnlineBoards = 0;

	bool leftMotionBoardOnline = updateBoardStatus(piData.getLeftMotionBoard().isOnline());
	bool rightMotionBoardOnline = updateBoardStatus(piData.getRightMotionBoard().isOnline());
	bool rearMotionBoardOnline = updateBoardStatus(piData.getRearMotionBoard().isOnline());

	bool leftBallhandlerBoardOnline = updateBoardStatus(piData.getLeftBallhandlerBoard().isOnline());;
	bool rightBallhandlerBoardOnline = updateBoardStatus(piData.getRightBallhandlerBoard().isOnline());;

	if (numberOfOnlineBoards > 0) {
		if ((previousNumberOfOnlineBoards == 0) && (numberOfOnlineBoards >= desiredNumberOfDevices)) {
			cout << "INFO    : EMO unpressed." << endl;
			TRACE_INFO("EMO unpressed.");
		}
		else {
			logBoardStatus(previousLeftMotionBoardOnline, leftMotionBoardOnline, "Left motion");
			logBoardStatus(previousRightMotionBoardOnline, rightMotionBoardOnline, "Right motion");
			logBoardStatus(previousRearMotionBoardOnline, rearMotionBoardOnline, "Rear motion");
			logBoardStatus(previousLeftBallhandlerBoard, leftBallhandlerBoardOnline, "Left ballhandler");
			logBoardStatus(previousRightBallhandlerBoard, rightBallhandlerBoardOnline, "Right ballhandler");
		}
	}
	else {
		if (previousNumberOfOnlineBoards > 0) {
			cerr << "ERROR   : EMO pressed." << endl;
			TRACE_ERROR("EMO pressed.");
		}
	}

	previousLeftMotionBoardOnline = leftMotionBoardOnline;
	previousRightMotionBoardOnline = rightMotionBoardOnline;
	previousRearMotionBoardOnline = rearMotionBoardOnline;
	previousLeftBallhandlerBoard = leftBallhandlerBoardOnline;
	previousRightBallhandlerBoard = rightBallhandlerBoardOnline;
}

bool Diagnostics::updateBoardStatus(bool boardOnline) {
	if (boardOnline) {
		numberOfOnlineBoards++;
	}

	return boardOnline;
}

void Diagnostics::logBoardStatus(bool previousBoardOnline, bool boardOnline, string name) {
	if (boardOnline && !previousBoardOnline) {
		cout << "INFO    : " << name << " board is online again!" << endl;
		TRACE_INFO("%s board is online again!", name.c_str());
	}
	if (!boardOnline && previousBoardOnline) {
		cerr << "ERROR   : " << name << " board is not available." << endl;
		TRACE_ERROR("%s board is not available.", name.c_str());
	}
}
