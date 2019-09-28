 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <chrono>
#include <exception>
#include <functional>
#include <thread>

#include <pwd.h>

#include "cDiagnostics.hpp"
#include "FalconsCommon.h"
#include "tracing.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "int/adapters/cRosAdapterMotionConfig.hpp"
#include "int/adapters/cRosAdapterBallhandlersConfig.hpp"

#include "int/PeripheralsInterfaceData.hpp"
#include "int/PeripheralsInterfaceTypes.hpp"

#include "int/Motion.hpp"
#include "int/Ballhandlers.hpp"

#include "int/DeviceManager.hpp"
#include "int/Diagnostics.hpp"

#include "int/motors/MotionBoard.hpp"
#include "int/motors/BallhandlerBoard.hpp"

using namespace std;

class peripheralsInterfaceMain {
public:
	peripheralsInterfaceMain();
	~peripheralsInterfaceMain();

private:
	void loadConfiguration();
	void loadRosAdapters();
	void loadDeviceManager();
	void loadDiagnostics();
	void loadController();

	void controllerTimer();

	PeripheralsInterfaceData _piData;
	DeviceManager _deviceManager;

	MotionBoard _rightMotionBoard;
	MotionBoard _leftMotionBoard;
	MotionBoard _rearMotionBoard;

	BallhandlerBoard _rightBallhandlerBoard;
	BallhandlerBoard _leftBallhandlerBoard;

	Motion _motion;
	Ballhandlers _ballhandlers;

	// RTDB adapters
	cRTDBInputAdapter _rtdbInputAdapter;
	cRTDBOutputAdapter _rtdbOutputAdapter;

//	// RosAdapter objects
	cRosAdapterMotionConfig _rosAdapterMotionConfig;
	cRosAdapterBallhandlersConfig _rosAdapterBallhandlersConfig;

	Diagnostics _diagnostics;

	// Timer threads
	thread _controllerThread;

	bool _ballhandlersAvailable;
};

peripheralsInterfaceMain::~peripheralsInterfaceMain() {
}

peripheralsInterfaceMain::peripheralsInterfaceMain():
		_deviceManager(_piData),
		_rightMotionBoard(MOTION_BOARD_RIGHT, _deviceManager),
		_leftMotionBoard(MOTION_BOARD_LEFT, _deviceManager),
		_rearMotionBoard(MOTION_BOARD_REAR, _deviceManager),
		_rightBallhandlerBoard(BALLHANDLER_BOARD_RIGHT, _deviceManager),
		_leftBallhandlerBoard(BALLHANDLER_BOARD_LEFT, _deviceManager),
		_motion(_piData, _leftMotionBoard, _rightMotionBoard, _rearMotionBoard),
		_ballhandlers(_piData, _leftBallhandlerBoard, _rightBallhandlerBoard),
		_rtdbInputAdapter(_piData),
		_rtdbOutputAdapter(_piData),
		_rosAdapterMotionConfig(_piData),
		_rosAdapterBallhandlersConfig(_piData),
		_diagnostics(_piData, _ballhandlersAvailable),
		_ballhandlersAvailable(!isGoalKeeper())
{
	loadDeviceManager();
	loadRosAdapters();
	loadController();

	// Wait one or so second for the boards to be identified.
	this_thread::sleep_for(chrono::milliseconds(1200));
	loadDiagnostics();
}

void peripheralsInterfaceMain::loadRosAdapters() {

	cout << "INFO    : Initializing Motion Config ROS adapter." << endl;
	_rosAdapterMotionConfig.initialize();

	if (_ballhandlersAvailable) {
		cout << "INFO    : Initializing Ballhandlers Config ROS adapter." << endl;
		_rosAdapterBallhandlersConfig.initialize();
	}

}

void peripheralsInterfaceMain::loadDeviceManager() {
	_deviceManager.start();
}

void peripheralsInterfaceMain::loadDiagnostics() {
	_diagnostics.start();
}

void peripheralsInterfaceMain::loadController() {
	_controllerThread = thread(&peripheralsInterfaceMain::controllerTimer, this);
}

void peripheralsInterfaceMain::controllerTimer() {
	TRACE("Starting controller.");

	while (true) {
		// Calculate the time when the next iteration should be started.
		chrono::system_clock::time_point end_time = chrono::system_clock::now() + chrono::milliseconds(10);

		_rtdbInputAdapter.getMotorVelocitySetpoint();
		_rtdbInputAdapter.getBallHandlersMotorSetpoint();

		_motion.update();
		_ballhandlers.update();

		_rtdbOutputAdapter.setMotorFeedback();
		_rtdbOutputAdapter.setBallHandlersFeedback();

		this_thread::sleep_until(end_time);

		WRITE_TRACE;
	}
}

void loadConfiguration() {

	// Load motor parameters
	loadConfig("Motors");

	string configFileCmd = string("rosparam load ") + getpwuid(getuid())->pw_dir + string("/falcons/code/config/BallHandlers.yaml");
	int result = system(configFileCmd.c_str());
	if (result) {
		TRACE_ERROR("Could not load Ballhandlers yaml file.");
		cerr << "ERROR   : Could not load Ballhandlers yaml file." << endl;
		throw(runtime_error("Could not load Ballhandlers yaml file."));
	}
	// Load ballhandler parameters
	loadConfig("BallHandlers");
}

int main(int argc, char **argv) {

	ros::init(argc, argv, peripheralsInterfaceNodeNames::motors);

    INIT_TRACE;

	// Load the active YAML configuration.
	loadConfiguration();

	peripheralsInterfaceMain peripheralsInterface;

	ros::spin();
}
