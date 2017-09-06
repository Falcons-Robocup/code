 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * MotorControllerBoard.hpp
 *
 *  Created on: May 3, 2016
 *      Author: robocup
 */

#ifndef INCLUDE_INT_MOTORS_MOTORCONTROLLERBOARD_HPP_
#define INCLUDE_INT_MOTORS_MOTORCONTROLLERBOARD_HPP_

#include <int/PeripheralsInterfaceData.hpp>
#include <chrono>
#include <vector>

#include "int/Communication.hpp"
#include "int/DeviceManager.hpp"

using namespace std;

class MotorControllerBoard {
public:
	MotorControllerBoard(ApplicationId id, DeviceManager &deviceManager);
	virtual ~MotorControllerBoard();

	bool isConnected();
	bool isConfigured();

protected:
	ApplicationId applicationId;
	string boardName;
	DeviceManager &deviceManager;

	bool configured;
	size_t updateCounter;

	chrono::steady_clock::time_point lastUpdateTime;
	vector<ResponseList> expectedResponses;

	MotorControllerBoardData motorControllerData;

	virtual void update();
	virtual void forceConfiguration();
	virtual void configure();
	virtual bool isConfigurationDone();

	void send(TransmitPackage package);

	void handleCommError(unsigned short commError);
	void handleEncError(unsigned short encError);
	void handlePidError(unsigned short pidError);
	void handlePwmError(unsigned short pwmError);
	void handleSafetyError(unsigned short safetyError);
	void handleSchedulerError(unsigned short schedulerError);
	void handleDrv8301Error(unsigned short drv8301Error);

	// Data retrieve functions.
	void requestCommand(CommandList command);

	// Data send functions.
	void clearErrors(ReceivePackage &package);
	void clearAllErrors();
	void loopback();

	virtual void handleResponse(ReceivePackage &package);
	virtual void handleDefaultResponse(ReceivePackage &package);
	virtual void handleErrorResponse(ReceivePackage &package);
	virtual void handleGainResponse(ReceivePackage &package);
	virtual void handleLedGreenResponse(ReceivePackage &package);
	virtual void handleLedYellowResponse(ReceivePackage &package);
	virtual void handleLoopbackResponse(ReceivePackage &package);
	virtual void handleMotorTimeoutResponse(ReceivePackage &package);
	virtual void handlePidPrimaryPropertiesResponse(ReceivePackage &package);
	virtual void handlePwmLimitResponse(ReceivePackage &package);
	virtual void handlePrimarySetpointResponse(ReceivePackage &package);
	virtual void handlePwmManualResponse(ReceivePackage &package);
	virtual void handleDrv8301Response(ReceivePackage &package);
	virtual void handlePwmDeltaResponse(ReceivePackage &package);
	virtual void handleModeResponse(ReceivePackage &package);
	virtual void handleAngleTachoZeroResponse(ReceivePackage &package);
	virtual void handlePidAnglePropertiesResponse(ReceivePackage &package);
	virtual void handleAngleSetpointResponse(ReceivePackage &package);
	virtual void handleAngleDirectionResponse(ReceivePackage &package);

	void changeGain(unsigned char value);
	void changeLedGreen(bool flag);
	void changeLedYellow(bool flag);
	void changeMotorTimeout(unsigned short value);
	void changePidPrimaryProperties(unsigned short p, unsigned short i, unsigned short d, unsigned short iTh, unsigned long iMax);
	void changePwmManual(short value);
	void changePwmLimit(unsigned short value);
	void changePrimarySetpoint(short value);
	void changeDrv8301();
	void changePwmDelta(unsigned long value);
	void changeMode(ModeList Mode);
	void changePidAngleProperties(unsigned short p, unsigned short i, unsigned short d, unsigned short iTh, unsigned long iMax);
	void changeAngleSetpoint(short value);
	void changeAngleDirection(bool flag);

	static float adcToAdcVoltage(float adc);
	static float adcVoltageToPowerSupply(float adcVoltage);
	static float adcVoltageToNtcResistance(float adcVoltage);
	static float ntcResistanceToBoardTemperature(float ntcResistance);
	static float ntcResistanceToMotorTemperature(float ntcResistance);
};

#endif /* INCLUDE_INT_MOTORS_MOTORCONTROLLERBOARD_HPP_ */
