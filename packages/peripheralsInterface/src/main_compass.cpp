 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main_compass.cpp
 *
 *  Created on: Apr 2, 2016
 *      Author: Edwin Schreuder
 */

#include <exception>

#include <dirent.h>
#include <unistd.h>
#include <pwd.h> // for getpwuid()

#include "ros/ros.h"

#include "FalconsCommon.h"

#include "cDiagnostics.hpp"
#include "cDiagnosticsEvents.hpp"

#include "rosMsgs/t_diag_compass.h"

#include "ext/peripheralInterfaceNames.hpp"

#include "int/adapters/cRosAdapterCompass.hpp"
#include "int/adapters/cRosAdapterCompassConfig.hpp"
#include "int/adapters/cRosAdapterIMU.hpp"
#include "int/adapters/cRosAdapterIMUConfig.hpp"

#include "int/compass/cCompass.hpp"
#include "int/IMU/IMU.hpp"

using namespace std;

class NoCompassException : public std::runtime_error
{
	public:
		NoCompassException(std::string const& error) : std::runtime_error(error) {};
};

cCompass * _compass;
cRosAdapterCompass * _rosAdapterCompass;
cRosAdapterCompassConfig * _rosAdapterCompassConfig;

IMU * _imu;
cRosAdapterIMU * _rosAdapterIMU;
cRosAdapterIMUConfig * _rosAdapterIMUConfig;

diagnostics::cDiagnosticsSender<rosMsgs::t_diag_compass> g_diagSender(diagnostics::DIAG_COMPASS, 0);

static void thetaUpdateCallback(double theta)
{
    rosMsgs::t_diag_compass msg;

    msg.theta = theta;

	g_diagSender.set(msg);
}

static void startIMUCommunication(void)
{
    // Create a new IMU object with default parameters.
	_imu = new IMU();
	_rosAdapterIMUConfig = new cRosAdapterIMUConfig(*_imu);
	boost::function<void(double)> f = boost::bind(thetaUpdateCallback, _1);
	_rosAdapterIMU = new cRosAdapterIMU(*_imu, f);
}

static void startCompassCommunication(void)
{
	std::vector<std::string> ttyCompass;

	DIR * dir = opendir("/dev/");
	struct dirent * entity;

	if (dir) {
		while ((entity = readdir(dir)) != NULL) {
			if (strncmp(entity->d_name, "ttyCompass", 10) == 0) {
				ttyCompass.emplace_back("/dev/" + string(entity->d_name));
			}
		}
		closedir(dir);
	}

	if (!ttyCompass.empty()) {
		_compass = new cCompass(ttyCompass.at(0));
		_rosAdapterCompassConfig = new cRosAdapterCompassConfig();
		boost::function<void(double)> f = boost::bind(thetaUpdateCallback, _1);
		_rosAdapterCompass = new cRosAdapterCompass(_compass, f);
	}
	else {
		throw runtime_error("Unable to identify Compass");
	}
}

static void startCommunication(void)
{
	// First, try to initialize the IMU. When this is not available, try to initialize the compass.
	// If no devices are found, throw an error.

	try {
		startIMUCommunication();
	}
	catch (exception &imu_error) {
		try {
			startCompassCommunication();
		}
		catch (exception &compass_error) {
			throw NoCompassException(string("Could not connect to IMU (") + string(imu_error.what()) + string(") or compass (") + compass_error.what() + string(")."));
		}
	}
}

int main(int argc, char **argv)
{
	int result = EXIT_SUCCESS;
    ros::init(argc, argv, peripheralInterfaceNodeNames::compassNodeName);
    //TRACE_INFO("Compass starting"); // must be after ros::init

	/* Reconfig run */
    struct passwd *pw = getpwuid(getuid());
	int robotNr = getRobotNumber();
    string configFileCmd;

	// Load compass parameters
	configFileCmd = string("rosparam load ") + string(pw->pw_dir) + string("/falcons/code/config/Compass.yaml");
	result = system(configFileCmd.c_str());
	if (result) {
		TRACE_ERROR("Could not load Compass yaml file.");
		cerr << "Could not load Compass yaml file." << endl;
	}

	// Load Robot specific compass parameters
	configFileCmd = string("rosparam load ") + string(pw->pw_dir) + string("/falcons/code/config/CompassR") + to_string(robotNr) + string(".yaml");
	result = system(configFileCmd.c_str());
	if (result) {
		TRACE_ERROR("Could not load BallhandlersR%d", robotNr);
		cerr << "Could not load BallhandlersR" << to_string(robotNr) << " yaml file." << endl;
	}

    try {
        // Try to start a compass
        startCommunication();

        // Start the ROS node.
        ros::spin();
    }
    catch(NoCompassException &e) {
        TRACE_ERROR("Could not establish a connection with the IMU or Compass. No device found: ", e.what());
        cerr << "Could not establish a connection with the IMU or Compass. No device found: " << e.what() << endl;
    	result = EXIT_FAILURE;
    }
    catch(exception &e) {
    	TRACE_ERROR("An unexpected error occurred in the compass node, node is shutdown: ", e.what());
    	cerr << "An unexpected error occurred in the compass node, node is shutdown: " << e.what() << endl;
    	result = EXIT_FAILURE;
    }
    catch(...) {
    	TRACE_ERROR("An unexpected error occurred in the compass node, node is shutdown.");
    	cerr << "An unexpected error occurred in the compass node, node is shutdown." << endl;
    	result = EXIT_FAILURE;
    }

    return result;
}
