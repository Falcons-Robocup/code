 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main_ioBoard.cpp
 *
 *  Created on: Apr 26, 2017
 *      Author: Edwin Schreuder
 */

#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

#include <sys/stat.h>

#include <cDiagnosticsEvents.hpp>
#include <FalconsCommon.h>
#include <peripheralsInterface/ioBoardConfig.h>

#include "int/ioBoard/IoBoard.hpp"
#include "int/Kicker.hpp"

#include "int/adapters/cRosAdapterKicker.hpp"
#include "int/adapters/cRosAdapterRobotStatus.hpp"

#include "ext/peripheralInterfaceNames.hpp"

using std::atomic;
using std::cerr;
using std::cout;
using std::chrono::milliseconds;
using std::endl;
using std::exception;
using std::thread;

class PeripheralsInterfaceIoBoard {
public:
	PeripheralsInterfaceIoBoard(string portName);
	~PeripheralsInterfaceIoBoard();

private:
	IoBoard ioBoard;
	Kicker kicker;

	thread robotStatusUpdateThread;
	atomic<bool> robotStatusUpdateRunning;

	cRosAdapterKicker rosAdapterKicker;
	cRosAdapterRobotStatus rosAdapterRobotStatus;

	void robotStatusUpdate();

	void initializeIoBoard();

	void processRobotStatus(IoBoard::Status data);
	void processRobotInPlay(bool inPlay);
	void processRobotSoftwareOn(bool softwareOn);

	void startSoftware();
	void stopSoftware();

	bool inPlay;
	bool softwareOn;
};

PeripheralsInterfaceIoBoard::PeripheralsInterfaceIoBoard(string portName) :
	ioBoard(portName),
	kicker(ioBoard),
	rosAdapterKicker(kicker) {

	inPlay = false;
	softwareOn = false;

	initializeIoBoard();

	rosAdapterKicker.initialize();
	rosAdapterRobotStatus.initialize();

	robotStatusUpdateRunning = true;
	robotStatusUpdateThread = thread(&PeripheralsInterfaceIoBoard::robotStatusUpdate, this);
}

PeripheralsInterfaceIoBoard::~PeripheralsInterfaceIoBoard() {
	if (robotStatusUpdateThread.joinable()) {
		robotStatusUpdateRunning = false;
	}
}

void PeripheralsInterfaceIoBoard::initializeIoBoard() {
	bool initialized = false;

	while (!initialized) {
		try {
			ioBoard.initialize();
			initialized = true;
			TRACE_INFO("Connection with IoBoard established.");
		}
		catch (exception &e) {
			TRACE_ERROR_TIMEOUT(30.0, "First connection with IoBoard not yet established, retrying.");
		}
	}
}

void PeripheralsInterfaceIoBoard::robotStatusUpdate() {

	bool board_online = true;

	while(robotStatusUpdateRunning) {

		try {
			IoBoard::Status status = ioBoard.getStatus();

			processRobotStatus(status);

			if (!board_online) {
				TRACE_INFO("Communication with IO board restored!");
				cout << "Communication with IO board restored!" << endl;
				board_online = true;
			}
		}
		catch(exception &e) {
			// When communication to the IoBoard fails, set the robot to out of play.
			// This ensures that robot is set to out of play when the EMO is pressed.
			inPlay = false;
			rosAdapterRobotStatus.setRobotStatus(inPlay);

			if (board_online) {
				board_online = false;
				TRACE_ERROR("Communication with IO board seems offline!");
				cerr << "Communication with IO board seems offline!" << endl;
			}
		}

		this_thread::sleep_for(milliseconds(500));
	}
}

void PeripheralsInterfaceIoBoard::processRobotStatus(IoBoard::Status status)
{
	processRobotInPlay(status.inPlay);
	processRobotSoftwareOn(status.softwareOn);
}

void PeripheralsInterfaceIoBoard::processRobotInPlay(bool newInPlay)
{
	// Just publish the inPlay status every 500 ms, this way wordmodel always has
	// the latest
//	if (newInPlay != inPlay) {
		inPlay = newInPlay;
		rosAdapterRobotStatus.setRobotStatus(inPlay);

		if (inPlay) {
			cout << "Robot now in play." << endl;
		}
		else {
			cout << "Robot now out of play." << endl;
		}
//	}
}

void PeripheralsInterfaceIoBoard::processRobotSoftwareOn(bool newSoftwareOn)
{
	if (newSoftwareOn != softwareOn)
	{
		try {
			if (newSoftwareOn) {
				startSoftware();
				TRACE_INFO("Software On switch triggered, software is being turned on.");
				cout << "Software On switch triggered, software is being turned on." << endl;
			}
			else {
				stopSoftware();
				TRACE_INFO("Software On switch triggered, software is being turned off.");
				cout << "Software On switch triggered, software is being turned off." << endl;
			}

			softwareOn = newSoftwareOn;
		}

		catch (exception &e) {
			TRACE_ERROR("Failed to stop or start software, retrying in 1 second: %s", e.what());
			cerr << "Failed to stop or start software, retrying in 1 second: " << e.what() << endl;
		}
	}
}

void PeripheralsInterfaceIoBoard::startSoftware()
{
	int result = system("robotControl start");

	if (result != 0) {
		throw(runtime_error("robotControl start returned " + to_string(result)));
	}
}

void PeripheralsInterfaceIoBoard::stopSoftware()
{
	// TODO: Investigate JFEI, robotControl stop doesn't work
	int result = system("jobStopAll");

	if (result < 0) {
		throw(runtime_error("jobStopAll returned " + to_string(result)));
	}
}

void loadIoBoardSettings() {

	// Load the specific IO board settings for this robot.
	loadConfig("IoBoard");
}

string getSerialPortName() {

	// Create the NodeHandle to read the parameters from
	ros::NodeHandle nh = ros::NodeHandle(peripheralInterfaceNodeNames::ioBoard);

	// Read parameters from NodeHandle
	peripheralsInterface::ioBoardConfig config;
	config.__fromServer__(nh);

	return config.PortName;
}

int main(int argc, char **argv) {

	cout << "Starting ROS for name " << peripheralInterfaceNodeNames::ioBoard << endl;
	ros::init(argc, argv, peripheralInterfaceNodeNames::ioBoard);

	cout << "Loading IO board settings" << endl;
	loadIoBoardSettings();

	cout << "Retrieve the serial port name to instantiate the IO board communication." << endl;
	string portName = getSerialPortName();

	cout << "Starting peripheralsInterface IO board on port " << portName << "." << endl;
	PeripheralsInterfaceIoBoard peripheralsInterfaceIoBoard(portName);

	cout << "ROS spin." << endl;
	ros::spin();
}
