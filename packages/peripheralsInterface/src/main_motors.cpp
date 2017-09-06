 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

#include <cDiagnosticsEvents.hpp>
#include <FalconsCommon.h>

#include "int/adapters/cRosAdapterMotion.hpp"
#include "int/adapters/cRosAdapterMotionConfig.hpp"
#include "int/adapters/cRosAdapterWorldModel.hpp"
#include <int/adapters/cRosAdapterBallhandlers.hpp>
#include <int/adapters/cRosAdapterBallhandlersConfig.hpp>

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
	void loadControlLoops();

	void publisherTimer();

	PeripheralsInterfaceData _piData;
	DeviceManager _deviceManager;

	MotionBoard _rightMotionBoard;
	MotionBoard _leftMotionBoard;
	MotionBoard _rearMotionBoard;

	BallhandlerBoard _rightBallhandlerBoard;
	BallhandlerBoard _leftBallhandlerBoard;

	Motion _motion;
	Ballhandlers _ballhandlers;

//	// RosAdapter objects
	cRosAdapterMotion _rosAdapterMotion;
	cRosAdapterMotionConfig _rosAdapterMotionConfig;
	cRosAdapterBallhandlers _rosAdapterBallhandlers;
	cRosAdapterBallhandlersConfig _rosAdapterBallhandlersConfig;
	cRosAdapterWorldModel _rosAdapterWorldModel;

	Diagnostics _diagnostics;

	// Timer threads
	thread _publisherThread;

	bool _ballhandlersAvailable;
	bool _worldModelAvailable;
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
		_rosAdapterMotion(_piData),
		_rosAdapterMotionConfig(_piData),
		_rosAdapterBallhandlers(_piData),
		_rosAdapterBallhandlersConfig(_piData),
		_rosAdapterWorldModel(_piData),
		_diagnostics(_piData, _ballhandlersAvailable),
		_ballhandlersAvailable(!isGoalKeeper()),
		_worldModelAvailable(true)
{
	loadDeviceManager();
	loadRosAdapters();

	_publisherThread = thread(&peripheralsInterfaceMain::publisherTimer, this);

	loadControlLoops();
	// Wait one or so second for the boards to be identified.
	this_thread::sleep_for(chrono::milliseconds(1200));
	loadDiagnostics();
}

void peripheralsInterfaceMain::loadRosAdapters() {
	cout << "INFO    : Initializing Motion ROS adapter." << endl;
	_rosAdapterMotion.initialize();

	cout << "INFO    : Initializing Motion Config ROS adapter." << endl;
	_rosAdapterMotionConfig.initialize();

	if (_ballhandlersAvailable) {
		cout << "INFO    : Initializing Ballhandlers ROS adapter." << endl;
		_rosAdapterBallhandlers.initialize();

		cout << "INFO    : Initializing Ballhandlers Config ROS adapter." << endl;
		_rosAdapterBallhandlersConfig.initialize();
	}

	if (_worldModelAvailable) {
		cout << "INFO    : Initializing World Model ROS adapter." << endl;
		_rosAdapterWorldModel.initialize();
	}
}

void peripheralsInterfaceMain::loadDeviceManager() {
	_deviceManager.start();
}

void peripheralsInterfaceMain::loadDiagnostics() {
	_diagnostics.start();
}

void peripheralsInterfaceMain::loadControlLoops() {
	_motion.start();

	if (_ballhandlersAvailable) {
		_ballhandlers.start();
	}
}

void peripheralsInterfaceMain::publisherTimer() {
	cout << "INFO    : Starting publisher." << endl;

	while (true) {
		_rosAdapterMotion.publishPidParameters();

		_rosAdapterMotion.publishRobotSpeed();

		if (_ballhandlersAvailable) {
			_rosAdapterBallhandlers.publishPidParameters();
		}

		if (_worldModelAvailable) {
			_rosAdapterWorldModel.notifyWorldModel();
		}

		if (_piData.getLeftMotionBoard().getSettings().motorPlotEnabled){
			//Trace motor 1-3
			MotionBoardDataOutput mLe, mRi, mRe;
			mLe = _piData.getLeftMotionBoard().getDataOutput();
			mRi = _piData.getRightMotionBoard().getDataOutput();
			mRe = _piData.getRearMotionBoard().getDataOutput();

			//double t = timeConvert
			/*Labels defined in test script:
			 * 		"mLeSetpoint;mLeVelocity;mLeError;mLeProportional;mLeIntegral;mLeDerivative;mLePIDOutput;mLeDisplDist;mLeDisplTicks;mLePWM;"\
					"mRiSetpoint;mRiVelocity;mRiError;mRiProportional;mRiIntegral;mRiDerivative;mRiPIDOutput;mRiDisplDist;mRiDisplTicks;mRiPWM;"\
					"mReSetpoint;mReVelocity;mReError;mReProportional;mReIntegral;mReDerivative;mRePIDOutput;mReDisplDist;mReDisplTicks;mRePWM;"\
			*/
			TRACE("kstdata %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f; %7.4f;",
					mLe.motorController.setpoint,mLe.motion.velocity,mLe.motion.velocityError,mLe.motion.errorMs,mLe.motion.integralMs,mLe.motion.derivativeMs,\
					mLe.motion.pidOutputMs,mLe.motion.displacementDistance,mLe.motion.displacementEncTicks,mLe.motorController.pwm,\
					mRi.motorController.setpoint,mRi.motion.velocity,mRi.motion.velocityError,mRi.motion.errorMs,mRi.motion.integralMs,mRi.motion.derivativeMs,\
					mRi.motion.pidOutputMs,mRi.motion.displacementDistance,mRi.motion.displacementEncTicks,mRi.motorController.pwm,\
					mRe.motorController.setpoint,mRe.motion.velocity,mRe.motion.velocityError,mRe.motion.errorMs,mRe.motion.integralMs,mRe.motion.derivativeMs,\
					mRe.motion.pidOutputMs,mRe.motion.displacementDistance,mRe.motion.displacementEncTicks,mRe.motorController.pwm);

		}
		if (_piData.getLeftBallhandlerBoard().getSettings().ballhandlerPlotEnabled) {
			//Trace ballhandler 1-2
			//TODO: TRACE();

		}
		this_thread::sleep_for(chrono::milliseconds(10));
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

	ros::init(argc, argv, peripheralInterfaceNodeNames::motors);

	// Load the active YAML configuration.
	loadConfiguration();

	peripheralsInterfaceMain peripheralsInterface;

	ros::spin();
}
