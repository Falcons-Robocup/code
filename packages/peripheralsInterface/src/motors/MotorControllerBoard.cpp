 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * MotorControllerBoard.cpp
 *
 *  Created on: May 3, 2016
 *      Author: Edwin Schreuder
 */

// TODO: Check if values from board are correct.

#include <algorithm>
#include <chrono>
#include <map>
#include <iostream>

#include "int/motors/MotorControllerBoard.hpp"

#include "int/CommunicationPackage.hpp"
#include "int/PeripheralsInterfaceExceptions.hpp"

using namespace std;

typedef void (MotorControllerBoard::*handleFunction)(ReceivePackage &package);

typedef struct {
	size_t payloadSize;
	handleFunction handleCommand;
} ResponseHandlerInfo;

MotorControllerBoard::MotorControllerBoard(ApplicationId id, DeviceManager &deviceManager):
		applicationId(id),
		boardName(getApplicationIdString(id)),
		deviceManager(deviceManager)
		{
	updateCounter = 0;
	configured = false;
}

MotorControllerBoard::~MotorControllerBoard() {

}

bool MotorControllerBoard::isConnected() {
	bool connected;

	try {
		shared_ptr<Communication> channel = deviceManager.getChannel(applicationId);
		connected = true;
	}
	catch (exception &e) {
		// Could not find the motorcontrollerboard, so not connected.
		connected = false;
	}

	return connected;
}

bool MotorControllerBoard::isConfigured() {
	return configured;
}

void MotorControllerBoard::update() {
	/*	feedForwardFactor = 0.0;
	angleCalibrated = false;
	 *
	 * The responsibility of the update method is to read any available
	 * package from the channel for this controller and update the internal
	 * administration and data. Furthermore, it should keep track of time and
	 * update the connected status based on whether default packages are still
	 * being updated and all initialization commands have been correctly sent.
	 */

	try {
		shared_ptr<Communication> channel = deviceManager.getChannel(applicationId);

		while (channel->receivePackageAvailable()) {
			ReceivePackage package = channel->receivePackage();

			handleResponse(package);
		}

		chrono::steady_clock::time_point currentTime = chrono::steady_clock::now();

		// If new packages are received and no responses are pending, we can update the current clock.
		unsigned int duration = chrono::duration_cast<chrono::milliseconds>(currentTime - lastUpdateTime).count();
		if (duration > 500) {
			cout << "INFO    : [" << boardName << "] Configuration started." << endl;
			lastUpdateTime = currentTime;
			configured = false;
			expectedResponses.clear();
			configure();
		}
		else {
			if (expectedResponses.size() == 0) {
				if (!configured && isConfigurationDone()) {
					cout << "INFO    : [" << boardName << "] Configuration complete [" << duration <<" milliseconds]." << endl;
					configured = true;
				}
				lastUpdateTime = currentTime;
			}
		}
	}
	catch (exception &e) {
		// Could not find the motorcontrollerboard, so disconnect the motor.
		configured = false;
	}
}

void MotorControllerBoard::forceConfiguration() {
	lastUpdateTime = chrono::steady_clock::now();
	configured = false;
}

void MotorControllerBoard::configure() {
	// Force a new configuration
	forceConfiguration();

	// Clear any errors
	clearAllErrors();

	// Apply the motor timeout to (200 * 2.5 =) 500 milliseconds;
	changeMotorTimeout(200);
	requestCommand(CMD_GET_MOTOR_TIMEOUT);
	expectedResponses.push_back(RESP_MOTOR_TIMEOUT);

	// Set the Drv8301 settings.
	changeDrv8301();
	requestCommand(CMD_GET_DRV8301);
	expectedResponses.push_back(RESP_DRV8301);
}

bool MotorControllerBoard::isConfigurationDone() {
	return true;
}

void MotorControllerBoard::send(TransmitPackage package) {
	try {
		shared_ptr<Communication> channel = deviceManager.getChannel(applicationId);
		channel->transmitPackage(package);
	}
	catch (exception &e) {
		// Could not find the motorcontrollerboard, so disconnect the motor.
		configured = false;
	}
}

void MotorControllerBoard::handleResponse(ReceivePackage &package) {
	/*
	 * The responsibility of the handleResponse function is to read out
	 * the command of the incoming package and call the function that handles
	 * that specific command. Furthermore, it searches the list of currently
	 * expected commands and
	 */

	static map<ResponseList, ResponseHandlerInfo> reponseHandlerInfoMap = {
			{RESP_DEFAULT, {40, &MotorControllerBoard::handleDefaultResponse}},
			{RESP_ERROR, {14, &MotorControllerBoard::handleErrorResponse}},
			{RESP_GAIN, {1, &MotorControllerBoard::handleGainResponse}},
			{RESP_LED_GREEN, {1, &MotorControllerBoard::handleLedGreenResponse}},
			{RESP_LED_YELLOW, {1, &MotorControllerBoard::handleLedYellowResponse}},
			{RESP_LOOPBACK, {8, &MotorControllerBoard::handleLoopbackResponse}},
			{RESP_MOTOR_TIMEOUT, {2, &MotorControllerBoard::handleMotorTimeoutResponse}},
			{RESP_PID_PRIMARY_PROPERTIES, {12, &MotorControllerBoard::handlePidPrimaryPropertiesResponse}},
			{RESP_PWM_LIMIT, {2, &MotorControllerBoard::handlePwmLimitResponse}},
			{RESP_PRIMARY_SETPOINT, {2, &MotorControllerBoard::handlePrimarySetpointResponse}},
			{RESP_PWM_MANUAL, {2, &MotorControllerBoard::handlePwmManualResponse}},
			{RESP_DRV8301, {12, &MotorControllerBoard::handleDrv8301Response}},
			{RESP_PWM_DELTA, {4, &MotorControllerBoard::handlePwmDeltaResponse}},
			{RESP_MODE, {1, &MotorControllerBoard::handleModeResponse}},
			{RESP_ANGLE_TACHO_ZERO, {4, &MotorControllerBoard::handleAngleTachoZeroResponse}},
			{RESP_PID_ANGLE_PROPERTIES, {12, &MotorControllerBoard::handlePidAnglePropertiesResponse}},
			{RESP_ANGLE_SETPOINT, {2, &MotorControllerBoard::handleAngleSetpointResponse}},
			{RESP_ANGLE_DIRECTION, {1, &MotorControllerBoard::handleAngleDirectionResponse}}
	};

	vector<ResponseList>::iterator expectedResponse = find(expectedResponses.begin(),
			expectedResponses.end(), (ResponseList) package.getResponseType());
	if (expectedResponse != expectedResponses.end())
	{
		expectedResponses.erase(expectedResponse);
	}

	map<ResponseList, ResponseHandlerInfo>::iterator it = reponseHandlerInfoMap.find((ResponseList) package.getResponseType());
	if (it == reponseHandlerInfoMap.end()) {
		cerr << "ERROR   : [" << boardName << "] Received an unknown response: " << package.getResponseType() << "!" << endl;
	} else
	{
		if (package.getPayloadSize() != (*it).second.payloadSize) {
			cerr << "ERROR   : Received payload size " << package.getPayloadSize();
			cerr << ", but " << (*it).second.payloadSize << "was expected for command " << package.getResponseType() << "!" << endl;
		} else {
			updateCounter++;
			(this->*((*it).second.handleCommand))(package);
		}
	}

}

void MotorControllerBoard::handleDefaultResponse(ReceivePackage &package) {
	motorControllerData.voltage = adcVoltageToPowerSupply(adcToAdcVoltage(package.getData<uint16_t>(16)));
	motorControllerData.boardTemperature = ntcResistanceToBoardTemperature(adcVoltageToNtcResistance(adcToAdcVoltage(package.getData<uint16_t>(17))));

	motorControllerData.error = package.getData<int16_t>(3);
	motorControllerData.integral = package.getData<int32_t>(2);
	motorControllerData.pidOutput = package.getData<int32_t>(3);
	motorControllerData.pwm =  package.getData<int16_t>(8);
	motorControllerData.measureTime = package.getData<uint32_t>(5);
	motorControllerData.calculationTime = package.getData<uint32_t>(6);
	motorControllerData.currentChannelA = package.getData<uint16_t>(14) - 2222;
	motorControllerData.currentChannelB = package.getData<uint16_t>(15) - 2222;
}

void MotorControllerBoard::handleErrorResponse(ReceivePackage &package) {
	handleCommError(package.getData<uint16_t>(0));
	handleEncError(package.getData<uint16_t>(1));
	handlePidError(package.getData<uint16_t>(2));
	handlePwmError(package.getData<uint16_t>(3));
	handleSafetyError(package.getData<uint16_t>(4));
	handleSchedulerError(package.getData<uint16_t>(5));
	handleDrv8301Error(package.getData<uint16_t>(6));

	// Clear errors so these won't keep popping up.
	clearErrors(package);
}

void MotorControllerBoard::handleGainResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] gain value is " << package.getData<uint8_t>(0);
	cout << "(" << package.getData<uint8_t>(0) << ")" << endl;
	motorControllerData.controllerGain = package.getData<uint8_t>(0);
}

void MotorControllerBoard::handleLedGreenResponse(ReceivePackage &package) {
	if (package.getData<uint8_t>(0) == 1) {
		cout << "INFO    : [" << boardName << "] Green LED is on." << endl;
		motorControllerData.ledGreen = true;
	} else {
		cout << "INFO    : [" << boardName << "] Green LED is off." << endl;
		motorControllerData.ledGreen = false;
	}
}

void MotorControllerBoard::handleLedYellowResponse(ReceivePackage &package) {
	if (package.getData<uint8_t>(0) == 1) {
		cout << "INFO    : [" << boardName << "] Yellow LED is on." << endl;
		motorControllerData.ledYellow = true;
	} else {
		cout << "INFO    : [" << boardName << "] Yellow LED is off." << endl;
		motorControllerData.ledYellow = false;
	}
}

void MotorControllerBoard::handleLoopbackResponse(ReceivePackage &package) {
	bool invalid = false;
	cout << "INFO    : [" << boardName << "] Loopback:";
	for (size_t i = 0; i < package.getPayloadSize(); i++) {
		cout << " " << package.getData<uint8_t>(i);
		if (package.getData<uint8_t>(i)) {
			invalid = true;
		}
	}
	cout << endl;

	if (invalid) {
		cerr << "ERROR   : [" << boardName << "] Loopback sequence is incorrect." << endl;
	}
}

void MotorControllerBoard::handleMotorTimeoutResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] Motor will be disabled if no valid packet has been received within " << 2.5 * package.getData<uint16_t>(0) << " ms." << endl;
	motorControllerData.motorTimeout =  2.5 * package.getData<uint16_t>(0);
}

void MotorControllerBoard::handlePidPrimaryPropertiesResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] PID constants are set to: Kp " << package.getData<uint16_t>(0);
	cout << " Ki " << package.getData<uint16_t>(1);
	cout << " Kd " << package.getData<uint16_t>(2);
	cout << " iTh " << package.getData<uint16_t>(3);
    cout << " iMax " << package.getData<uint32_t>(2) << endl;
	motorControllerData.pid.p = package.getData<uint16_t>(0);
	motorControllerData.pid.i = package.getData<uint16_t>(1);
	motorControllerData.pid.d = package.getData<uint16_t>(2);
	motorControllerData.pid.iTh = package.getData<uint16_t>(3);
	motorControllerData.pid.iMax = package.getData<uint32_t>(2);
}

void MotorControllerBoard::handlePwmLimitResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] PWM will be limited to " << package.getData<uint16_t>(0) << endl;
}

void MotorControllerBoard::handlePrimarySetpointResponse(ReceivePackage &package) {

}

void MotorControllerBoard::handlePwmManualResponse(ReceivePackage &package) {
}

void MotorControllerBoard::handleDrv8301Response(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] DRV8301 settings set." << endl;
}

void MotorControllerBoard::handlePwmDeltaResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] PWM delta (maximal increase per 2.5ms) is limited to " << package.getData<uint16_t>(0) << endl;
	motorControllerData.maxPwmStepValue = package.getData<uint32_t>(0);
}

void MotorControllerBoard::handleModeResponse(ReceivePackage &package) {
	string mode;

	switch(package.getData<uint8_t>(0)) {
	case MODE_PID_ANGLE:
		cout << "INFO    : [" << boardName << "] Mode angle = first PID input is angle + setpoint, second PID input is first PID + tacho." << endl;
		break;
	case MODE_PID_ENCODER:
		cout << "INFO    : [" << boardName << "] Mode encoder = PID input is encoder + setpoint." << endl;
		break;
	case MODE_PID_TACHO:
		cout << "INFO    : [" << boardName << "] Mode tacho = PID input is tacho + setpoint." << endl;
		break;
	case MODE_PWM_ONLY:
		cout << "INFO    : [" << boardName << "] Mode PWM = no PID, PWM is directly controlled from setpoint." << endl;
		break;
	case MODE_COMMUNICATION_TIMEOUT:
		cout << "INFO    : [" << boardName << "] Mode timeout = motor disabled because communication timeout." << endl;
		break;
	case MODE_DISABLE_MOTOR_POWER:
		cout << "INFO    : [" << boardName << "] Mode disabled = motor disabled (after reset / or Linux software initialization)." << endl;
		break;
	case MODE_ZERO_CURRENT_CALIBRATION:
		cout << "INFO    : [" << boardName << "] Mode calibration = motor disabled for zero current calibration of DRV8301." << endl;
		break;
	default:
		cerr << "ERROR   : [" << boardName << "] No mode " << package.getData<uint8_t>(0) << "set, abort." << endl;
	}
	motorControllerData.mode = package.getData<uint8_t>(0);
}

void MotorControllerBoard::handleAngleTachoZeroResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] Angle zero " << package.getData<uint16_t>(0) <<" tacho zero "<< package.getData<uint16_t>(1) << endl;
}

void MotorControllerBoard::handlePidAnglePropertiesResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] PID angle value are set to: Kp "<< package.getData<uint16_t>(0);
	cout << ", Ki " << package.getData<uint16_t>(1);
	cout << ", Kd " << package.getData<uint16_t>(2);
	cout << ", iTh " << package.getData<uint16_t>(3);
	cout << ", iMax " << package.getData<uint32_t>(2) << endl;
}

void MotorControllerBoard::handleAngleSetpointResponse(ReceivePackage &package) {

}

void MotorControllerBoard::handleAngleDirectionResponse(ReceivePackage &package) {
	if (package.getData<uint8_t>(0) == 0) {
		cout << "INFO    : [" << boardName << "] Angle direction behavior NOT inverted" << endl;
	} else {
		cout << "INFO    : [" << boardName << "] Angle direction behavior inverted." << endl;
	}
}

void MotorControllerBoard::handleCommError(unsigned short commError) {
	if ((commError & COMMUNICATION_ERROR_FROM_BOARD_WRITE) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication from board write!" << endl;
	if ((commError & COMMUNICATION_ERROR_FROM_BOARD_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication from board out of range!" << endl;
	if ((commError & COMMUNICATION_ERROR_FROM_BOARD_END_CHECK) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication from board end check!" << endl;
	if ((commError & COMMUNICATION_ERROR_FROM_BOARD_SOF_OVERWRITTEN) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication from board SOF overwritten!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_SIZE_OVERFLOW) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board size overflow!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_OVERFLOW) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board overflow!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_CHECKSUM) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board checksum!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_END_CHECK) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board end check!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_SOF_ALIGNMENT) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board SOF alignment!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_ILLEGAL_COMMAND) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board illegal command!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board ACK ID out of sync!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board packet overwritten!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board buffer overflow!" << endl;
	if ((commError & COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE) != 0) cerr << "ERROR   : [" << boardName << "] Far end communication to board incorrect amount of bytes received!" << endl;
}

void MotorControllerBoard::handleEncError(unsigned short encError) {
	if ((encError & ENCODER_ERROR_INIT_NOT_PERFORMED) != 0) cerr << "ERROR   : [" << boardName << "] Encoder not initialized!" << endl;
	if ((encError & ENCODER_ERROR_VELOCITY_MAX_NEGATIVE) != 0) cerr << "ERROR   : [" << boardName << "] Encoder maximal negative motor speed!" << endl;
	if ((encError & ENCODER_ERROR_VELOCITY_MAX_POSITIVE) != 0) cerr << "ERROR   : [" << boardName << "] Encoder maximal positive motor speed!" << endl;
}

void MotorControllerBoard::handlePidError(unsigned short pidError) {
	if ((pidError & PID_ERROR_INIT_NOT_PERFORMED) != 0) cerr << "ERROR   : [" << boardName << "] PID not initialized!" << endl;
	if ((pidError & PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Primary PID error value out of range!" << endl;
	if ((pidError & PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Primary PID integral out of range!" << endl;
	if ((pidError & PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Primary PID derivative out of range!" << endl;
	if ((pidError & PID_ERROR_PRIMARY_RESULT_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Primary PID result out of range!" << endl;
	if ((pidError & PID_ERROR_ANGLE_CURRENT_ERROR_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Angle PID error value out of range!" << endl;
	if ((pidError & PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Angle PID integral out of range!" << endl;
	if ((pidError & PID_ERROR_ANGLE_DERIVATIVE_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Angle PID derivative out of range!" << endl;
	if ((pidError & PID_ERROR_ANGLE_RESULT_OUT_OF_RANGE) != 0) cerr << "ERROR   : [" << boardName << "] Angle PID result out of range!" << endl;
}

void MotorControllerBoard::handlePwmError(unsigned short pwmError) {
	if ((pwmError & PWM_ERROR_INIT_NOT_PERFORMED) != 0) cerr << "ERROR   : [" << boardName << "] PWM not initialized!" << endl;
	if ((pwmError & PWM_ERROR_POSITIVE_LIMIT) != 0) cerr << "ERROR   : [" << boardName << "] PWM error positive limit!" << endl;
	if ((pwmError & PWM_ERROR_NEGATIVE_LIMIT) != 0) cerr << "ERROR   : [" << boardName << "] PWM error negative limit!" << endl;
	if ((pwmError & PWM_ERROR_POSITIVE_DELTA) != 0) cerr << "ERROR   : [" << boardName << "] PWM error positive delta (increase to fast)!" << endl;
	if ((pwmError & PWM_ERROR_NEGATIVE_DELTA) != 0) cerr << "ERROR   : [" << boardName << "] PWM error negative delta (reverse to fast)!" << endl;
}

void MotorControllerBoard::handleSafetyError(unsigned short safetyError) {
	if ((safetyError & SAFETY_ERROR_INIT_NOT_PERFORMED) != 0) cerr << "ERROR   : [" << boardName << "] Safety not initialized!" << endl;
	if ((safetyError & SAFETY_COMMUNICATION_TIMEOUT) != 0) cerr << "ERROR   : [" << boardName << "] Safety says board did not receive new packet in time! (motorTimeout)" << endl;
	if ((safetyError & SAFETY_ANGLE_TACHO_UNINITIALZED) != 0) cerr << "ERROR   : [" << boardName << "] Safety says un-initialized angle and or tacho used!" << endl;
	// the next error might appear when noise on the hall sensor
	if ((safetyError & SAFETY_ANGLE_VALUE_TOO_HIGH) != 0) cerr << "ERROR   : [" << boardName << "] Safety says angle value too high!, probaly unconnected hall sensor" << endl;
	if ((safetyError & SAFETY_ANGLE_VALUE_TOO_LOW) != 0) cerr << "ERROR   : [" << boardName << "] Safety says angle value too low!, probably short on hall sensor" << endl;
	if ((safetyError & SAFETY_TACHO_VALUE_TOO_HIGH) != 0) cerr << "ERROR   : [" << boardName << "] Safety says tacho value too high!" << endl;
	if ((safetyError & SAFETY_TACHO_VALUE_TOO_LOW) != 0) cerr << "ERROR   : [" << boardName << "] Safety says tacho value too low!" << endl;

}

void MotorControllerBoard::handleSchedulerError(unsigned short schedulerError) {
	if ((schedulerError & SCHEDULER_ERROR_INIT_NOT_PERFORMED) != 0) cerr << "ERROR   : [" << boardName << "] Scheduler not initialized!" << endl;
	if ((schedulerError & SCHEDULER_ERROR_MEASURE_TIME_EXPIRED) != 0) cerr << "ERROR   : [" << boardName << "] Scheduler measurement cycle to long!" << endl;
	if ((schedulerError & SCHEDULER_ERROR_CALCULATE_TIME_EXPIRED) != 0) cerr << "ERROR   : [" << boardName << "] Scheduler calculation cycle to long!" << endl;
	if ((schedulerError & SCHEDULER_ERROR_TEST) != 0) cerr << "ERROR   : [" << boardName << "] Scheduler test" << endl;
}

void MotorControllerBoard::handleDrv8301Error(unsigned short drv8301Error) {
	if ((drv8301Error & DRV8301_ERROR_INIT_NOT_PERFORMED) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 not initialized!" << endl;
	if ((drv8301Error & DRV8301_ERROR_FETLC_OC) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error FET low c over current (FETLC_OC) !" << endl;
	if ((drv8301Error & DRV8301_ERROR_FETHC_OC) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error FET high c over current (FETHC_OC)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_FETLB_OC) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error FET low b over current (FETLB_OC)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_FETHB_OC) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error FET high b over current (FETHB_OC)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_FETLA_OC) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error FET low a over current (FETLA_OC)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_FETHA_OC) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error FET high a over current (FETHA_OC)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_OTW) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error junction overtemperature warning (OTW)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_OTSD) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error junction overtemperature shutdown (OTSD)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_PVDD_UV) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error power supply undervoltage protection (PVDD_UV)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_GVDD_UV) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error internal gate driver undervoltage protection (GVDD_UV)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_FAULT) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error shutdown occurred (FAULT)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_DEVICE_ID) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error wrong Device ID!" << endl;
	if ((drv8301Error & DRV8301_ERROR_GVDD_OV) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 error overvoltage protection limit (GVDD_OV)!" << endl;
	if ((drv8301Error & DRV8301_ERROR_CONTROL_REGISTER1) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 write to control register 1 unsuccessful!" << endl;
	if ((drv8301Error & DRV8301_ERROR_CONTROL_REGISTER2) != 0) cerr << "ERROR   : [" << boardName << "] DRV8301 write to control register 2 unsuccessful!" << endl;
}

void MotorControllerBoard::requestCommand(CommandList command) {
	TransmitPackage package(command);
	send(package);
}

void MotorControllerBoard::clearAllErrors() {
	cout << "INFO    : [" << boardName << "] Clearing all errors." << endl;
	TransmitPackage transmitPackage(CMD_CLEAR_ERRORS);
	for (size_t i = 0; i < 7; i++) {
		transmitPackage.addData((uint16_t) 0xFFFF);
	}
	send(transmitPackage);
}

void MotorControllerBoard::clearErrors(ReceivePackage &receivePackage) {
	cout << "INFO    : [" << boardName << "] Clearing errors." << endl;
	TransmitPackage transmitPackage(CMD_CLEAR_ERRORS);
	for (size_t i = 0; i < 7; i++) {
		transmitPackage.addData(receivePackage.getData<uint16_t>(i));
	}
	send(transmitPackage);
}

void MotorControllerBoard::loopback() {
	TransmitPackage package(CMD_LOOPBACK);
	for (size_t i = 0; i < 7; i++) {
		package.addData((uint8_t) i);
	}
	send(package);
}

void MotorControllerBoard::changeGain(unsigned char value) {
	TransmitPackage package(CMD_SET_GAIN);
	package.addData(value);
	send(package);
}

void MotorControllerBoard::changeLedGreen(bool flag) {
	TransmitPackage package(CMD_SET_LED_GREEN);
	uint8_t value = flag ? 1 : 0;
	package.addData(value);
	send(package);
}

void MotorControllerBoard::changeLedYellow(bool flag) {
	TransmitPackage package(CMD_SET_LED_YELLOW);
	uint8_t value = flag ? 1 : 0;
	package.addData(value);
	send(package);
}

void MotorControllerBoard::changeMotorTimeout(unsigned short value) {
	TransmitPackage package(CMD_SET_MOTOR_TIMEOUT);
	package.addData((uint16_t) value);
	send(package);
}

void MotorControllerBoard::changePidPrimaryProperties(unsigned short p, unsigned short i, unsigned short d, unsigned short iTh, unsigned long iMax) {
	TransmitPackage package(CMD_SET_PID_PRIMARY_PROPERTIES);

	package.addData((uint16_t) p);
	package.addData((uint16_t) i);
	package.addData((uint16_t) d);
	package.addData((uint16_t) iTh);
	package.addData((uint32_t) iMax);
	send(package);
}

void MotorControllerBoard::changePwmManual(short value) {
	TransmitPackage package(CMD_SET_PWM_MANUAL);
	package.addData((int16_t) value);
	send(package);

}

void MotorControllerBoard::changePwmLimit(unsigned short value) {
	TransmitPackage package(CMD_SET_PWM_LIMIT);
	package.addData((uint16_t) value);
	send(package);
}

void MotorControllerBoard::changePrimarySetpoint(short value) {
	TransmitPackage package(CMD_SET_PRIMARY_SETPOINT);
	package.addData((int16_t) value);
	send(package);
}

void MotorControllerBoard::changeDrv8301() {
	uint16_t control1 = 0;
	// control1 |=  0 << 0; // GATE_CURRENT : Gate drive peak current 1.7 A, gives a spike of 750mV peak on shunt resistor
	// control1 |=  1 << 0; // GATE_CURRENT : Gate drive peak current 0.7 A
	control1 |=  1 << 1; // GATE_CURRENT : Gate drive peak current 0.25 A, gives a spike of 200mV peak on shunt resistor
	control1 |=  1 << 2;	// GATE_RESET : Reset gate driver latched faults (reverts to 0) (required if you got the drv8301 error shutdown occurred (FAULT))
	control1 |=  1 << 3;	// PWM_MODE : 3 PWM inputs
	control1 |=  0 << 4;	// OCP_MODE : Current limit (works together with OC_ADJ_SET)
//  control1 |=  1 << 4;	// OCP_MODE : OC latch shut down (requires a gate reset to resolve)
	// OC_ADJ_SET VDS = iOverCurrent x rdsOnFet (irf7749) => 60mV / 1.1mOhm = 54.5A => does not work, gives quickly errors when changing direction
	// use current limit instead of shutdown
//	control1 |= 0 << 6; 	// OC_ADJ_SET : Over current Adjustment = 60mV
	control1 |= 10 << 6;	// OC_ADJ_SET : Over current Adjustment = 197mV
//	control1 |= 31 << 6;	// OC_ADJ_SET : Over current Adjustment = 2.4V

	uint16_t control2 = 0;
	control2 |= 0 << 0;		// OCTW_MODE : Report both over temperature (OT) and over current (OC) at nOCTW pin
	control2 |= 2 << 2;		// GAIN : Gain of shunt amplifier: 40 V/V, with 30A ADC range between 0.05V to 2.45V, so best tradeoff between high current and noise
//	control2 |= 1 << 4;		// DC_CAL_CH1 : calibrate shunt amplifier 1
//	control2 |= 1 << 5;		// DC_CAL_CH2 : calibrate shunt amplifier 2
	control2 |= 0 << 6;		// OC_TOFF : Cycle by cycle = the MOSFET on which over current has been detected on will shut off until the next PWM cycle

	TransmitPackage package(CMD_SET_DRV8301);
	package.addData(control1);
	package.addData(control2);
	send(package);
}

void MotorControllerBoard::changePwmDelta(unsigned long value) {
	TransmitPackage package(CMD_SET_PWM_DELTA);
	package.addData((uint32_t) value);
	send(package);
}

void MotorControllerBoard::changeMode(ModeList mode) {
	TransmitPackage package(CMD_SET_MODE);
	package.addData((uint8_t) mode);
	send(package);
}

void MotorControllerBoard::changePidAngleProperties(unsigned short p, unsigned short i, unsigned short d, unsigned short iTh, unsigned long iMax) {
	TransmitPackage package(CMD_SET_PID_ANGLE_PROPERTIES);
	package.addData((uint16_t) p);
	package.addData((uint16_t) i);
	package.addData((uint16_t) d);
	package.addData((uint16_t) iTh);
	package.addData((uint32_t) iMax);
	send(package);
}

void MotorControllerBoard::changeAngleSetpoint(short value) {
	TransmitPackage package(CMD_SET_ANGLE_SETPOINT);
	package.addData((int16_t) value);
	send(package);
}

void MotorControllerBoard::changeAngleDirection(bool flag) {
	TransmitPackage package(CMD_SET_ANGLE_DIRECTION);
	uint8_t value = flag ? 1 : 0;
	package.addData(value);
	send(package);
}

/** \brief Converts a raw ADC value from the XMEGA to ADC voltage */
float MotorControllerBoard::adcToAdcVoltage(float adc) {
	// the adc is 12 bits, range 0 to 4095
	// the adc works with an offset (to be able to detect zero crossing)
	// the vRef is 2.5v
	// each bit is 2.5v/4095 = 0.61mV
	// 4095 = vRef - deltaV = vRef - (0.05 * vRef) = 0.95 * vRef = 2.375V
	// 205 = 0.05 * 4095 = 0V
	// 0 = - deltaV = - deltaV = 0.05 * vRef = -0.125V (not sure the ADC will return 0)

	return (2.5 * adc / 4095.0) - 0.125;
}

/** \brief Converts an ADC voltage to board voltage. */
float MotorControllerBoard::adcVoltageToPowerSupply(float adcVoltage) {
	return adcVoltage * (3.0 + 30.0) / 3.0; // values from schematic
}

/** \brief Converts an ADC voltage to NTC resistance. */
float MotorControllerBoard::adcVoltageToNtcResistance(float adcVoltage) {
	// for as well board temperature NTC as motor NTC temperature
	// the pull up resistor is 4k7 and connected to 3v3
	// the adcVoltage is measured over the ntc resistor
	float vcc3v3 = 3.27;
	float pullUp = 4700;
	// return current resistance value of ntc resistor
	return adcVoltage * pullUp / (vcc3v3 - adcVoltage);
}

/** \brief Converts an NTC resistance to board temperature. */
float MotorControllerBoard::ntcResistanceToBoardTemperature(float ntcResistance) {
	// regarding Veds the board contains a Panasonic ERT-J1VT472J
	// nominal resistance at 25C is 4k7 +/- 5%, B value at 25/50K 4500K +/- 2%
	const float ntcResistanceAt25Degrees = 4700;
	const float ntcNominalTemperature = 25;
	const float ntcBCoefficient = 4500;
	// Using Steinhart-Hart formula
	// ambientTemp = 1 / ( ln( ntcR/ntcNomR)/ntcBcoef + 1/nominalTemp )
	float kelvin = 1.0 / (log(ntcResistance / ntcResistanceAt25Degrees) / ntcBCoefficient + 1.0 / (ntcNominalTemperature + 273.15));
	float celcius = kelvin - 273.15;
	return celcius;
}

/** \brief Converts an NTC resistance to motor temperature using 3950 10K 1% NTC. */
float MotorControllerBoard::ntcResistanceToMotorTemperature(float ntcResistance) {
	// pull up to 3v3 is 4k7
	const float ntcResistanceAt25Degrees = 5000;
	const float ntcNominalTemperature = 25;
	const float ntcBCoefficient = 3500;
	// Using Steinhart-Hart formula
	// ambientTemp = 1 / ( ln( ntcR/ntcNomR)/ntcBcoef + 1/nominalTemp )
	float kelvin = 1.0 / (log(ntcResistance / ntcResistanceAt25Degrees) / ntcBCoefficient + 1.0 / (ntcNominalTemperature + 273.15));
	float celcius = kelvin - 273.15;
	return celcius;
}
