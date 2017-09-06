 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * IoBoard.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: Edwin Schreuder
 */

#include <exception>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <cDiagnosticsEvents.hpp>

#include "int/ioBoard/IoBoardCommunication.hpp"
#include "int/ioBoard/IoBoard.hpp"

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

const string IoBoardFirmwareVersion = "00000001";

map<IoBoard::SetCommand, unsigned char> IoBoardSetCommandValues = {
		{IoBoard::SHOOT, 0x02},
		{IoBoard::HOME, 0x05},
		{IoBoard::LEVER_SPEED, 0x06},
		{IoBoard::HEIGHT, 0x07},
		{IoBoard::BOOTLOADER, 0xAA}
};

map<IoBoard::GetCommand, unsigned char> IoBoardGetCommandValues = {
		{IoBoard::STATUS, 0x08},
		{IoBoard::VERSION, 0x56},
};

IoBoard::IoBoard(string portName) :
	portName(portName)
{
	communication = new IoBoardCommunication(portName);

	status = {false};
	measuredStatus = {false};
}

void IoBoard::initialize()
{
	if (!isFirmwareVersionCorrect())
	{
		upgradeFirmware(portName);

		if (!isFirmwareVersionCorrect()) {
			throw(runtime_error("Firmware version still not correct after upgrade."));
		}
	}
}

bool IoBoard::isFirmwareVersionCorrect() {
	bool firmwareVersionCorrect = false;

	cout << "Retrieving firmware version." << endl;

	IoBoardCommunication::Data data = communication->get(IoBoardGetCommandValues[VERSION]);

	cout << "Firmware version retrieved." << endl;

	string firmwareVersion(data.begin(), data.end());

	if (IoBoardFirmwareVersion != firmwareVersion) {
		cout << "Firmware version of the IO board is incorrect: " << firmwareVersion << " instead of " << IoBoardFirmwareVersion << "." << endl;
		TRACE_ERROR("Firmware version of the IO board is incorrect: %s", firmwareVersion.c_str());
	}
	else {
		cout << "IO board firmware version: " << firmwareVersion << "." << endl;
		firmwareVersionCorrect = true;
	}

	return firmwareVersionCorrect;
}

void IoBoard::upgradeFirmware(string portName) {

//	try {
//		cout << "Setting IO board to bootloader mode." << endl;
//		TRACE_INFO("Setting IO board to bootloader mode.");
//	}
//	catch (exception &e) {
//		throw(e);
//	}
//
//	delete communication;
//
//	cout << "IO board firmware upgrade started." << endl;
//	TRACE_INFO("Upgrading firmware to version %s.", IoBoardFirmwareVersion.c_str());
//
//	if (system(string("avrdude -p atxmega64a3 -P " + portName + "-c avr109 -b 115200 -U flash:w:../scripts/IoBoard.hex").c_str()) != 0) {
//		cerr << "Failed to upgrade the IO board firmware." << endl;
//		TRACE_ERROR("Failed to upgrade the IoBoard firmware.");
//		throw(runtime_error("Failed to upgrade the IoBoard firmware."));
//	}
//
//	communication = new IoBoardCommunication(portName);
//
//	cout << "IO board firmware upgrade finished." << endl;
//	TRACE_INFO("IO board firmware upgrade finished.");
}

IoBoard::Status IoBoard::getStatus()
{
	IoBoardCommunication::Data data = communication->get(IoBoardGetCommandValues[STATUS]);

	Status newMeasuredStatus;
	newMeasuredStatus.inPlay = data[0] != 0;
	newMeasuredStatus.softwareOn = data[1] != 0;

	if (newMeasuredStatus.inPlay == measuredStatus.inPlay) {
		// Only update the actual status when it has not changed since the previous cycle.
		status.inPlay = newMeasuredStatus.inPlay;
	}
	if (newMeasuredStatus.softwareOn == measuredStatus.softwareOn) {
		// Only update the actual status when it has not changed since the previous cycle.
		status.softwareOn = newMeasuredStatus.softwareOn;
	}
	measuredStatus = newMeasuredStatus;

	return status;
}

void IoBoard::setShoot(unsigned char shootPower)
{
	IoBoardCommunication::Data data;
	data[0] = shootPower;

	communication->set(IoBoardSetCommandValues[SHOOT], data);
}

void IoBoard::setHome()
{
	communication->set(IoBoardSetCommandValues[HOME], IoBoardCommunication::Data());
}

void IoBoard::setHeight(unsigned char height)
{
	IoBoardCommunication::Data data;
	data[0] = height;

	communication->set(IoBoardSetCommandValues[HEIGHT], data);
}

void IoBoard::setLeverSpeed(unsigned char speed)
{
	IoBoardCommunication::Data data;
	data[0] = speed;

	communication->set(IoBoardSetCommandValues[LEVER_SPEED], data);
}
