 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * DeviceManager.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: Edwin Schreuder
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <unistd.h>

#include <cDiagnosticsEvents.hpp>

#include <int/PeripheralsInterfaceExceptions.hpp>
#include <int/PeripheralsInterfaceTypes.hpp>

#include "int/DeviceManager.hpp"
#include "int/Communication.hpp"

#include "int/motors/MotorControllerBoard.hpp"
#include "int/motors/MotionBoard.hpp"
#include "int/motors/BallhandlerBoard.hpp"

#define FIRMWARE_VERSION	(22)
#define BH_CONTROLLERS		(2)
#define MOTION_CONTROLLERS	(3)

using namespace std;

/* There should be a monitor that monitors all the serial devices that are connected to the system.
 * A distinction in the term "connected" can be made in that sense. Either:
 *  - The USB serial converter is disconnected, no communication possible.
 *  - The USB serial converter is connected but serial communication fails.
 *
 *  We want to separate the polling of all communication objects via threading. This way, the system does not lock up
 *  when one of the devices (e.g. motors) is not connected. What this also stimulates is that the software does not need to be restarted
 *  once the USB cable is plugged in later, or the EMO button is released after the software has already started.
 *
 *  On startup all motors need to be scanned, and an error should be generated when not all motors are connected.
 *  Preferably, we can monitor which motor is not connected correctly. This also makes for easy troubleshooting. For this, we
 *  want to issue one second delay between startup of the process, and logging of the error. After one second, the manager
 *  can decide based on what devices have responded, how many USB connections there are and how many USB serial connections
 *  are present what troubleshooting is necessary.
 *  - When no USB connections can be found, there is a problem with USB.
 *  - When x USB connection can be found, but no devices are responding,
 *    the EMO button is pushed down or all motors are disconnected and n-x motors USB connections are disconnected.
 *  - When x USB connections an be found, and y motors are responding,
 *    x-y motors are disconnected and n-x USB connections are disconnected.
 *
 *	In all these cases, a TRACE_ERROR is logged to notify that there is a hardware problem.
 *	In this case, the status of the peripheralsInterface will become NOK.
 *	If after a while all motors respond and an error was logged previously, a TRACE_INFO will
 *	be logged to notify that the hardware problems have been restored again. In this case, the
 *	status of peripheralsInterface will be set to OK.
 *
 *  Once the communication object has been created, a "GET_APPLICATION_ID" request will be sent a seperate thread will
 *  wait for the motor data. Once the motor sends a reply, the communication object will send the
 *  request for an application ID and put the data in a non-blocking read queue. The write object will instantiate and put
 *  all data in a blocking queue that can be read out by the writing thread.
 *	The read thread will lock a condition variable, that allows the write thread to start writing the data to the serial port.
 *	Once the condition variable runs out, again the write thread is waiting for the condition variable to become true before data
 *	transmission can be recommenced.
 *
 */

string getApplicationIdString(ApplicationId applicationId) {
	string name;

	switch (applicationId) {
	case MOTION_MOTOR_LEFT:
		name = "Motion board left ";
		break;
	case MOTION_MOTOR_RIGHT:
		name = "Motion board right";
		break;
	case MOTION_MOTOR_REAR:
		name = "Motion board rear ";
		break;
	case BH_MOTOR_LEFT:
		name = "BH board left     ";
		break;
	case BH_MOTOR_RIGHT:
		name = "BH board right    ";
		break;
	case IO_BOARD:
		name = "IO board          ";
	case UNKNOWN:
		name = "Unknown board     ";
		break;
	}

	return name;
}

Device::Device(string port) :
		applicationId(UNKNOWN), channel(shared_ptr<Communication>(new Communication(port))), blacklisted(false), connected(false) {

	TransmitPackage package(CMD_AA_GET_BOARD);
	channel->transmitPackage(package);
}

Device::~Device() {
}

string Device::getPort() {
	return channel->getPortName();
}

shared_ptr<Communication> Device::getChannel() {
	return channel;
}

bool Device::isOnline() {
	return channel->isOnline();
}

bool Device::isConnected() {

	if (!channel->isOnline()) {
		connected = false;
	}

	return connected;
}

bool Device::isBlacklisted() {
	return blacklisted;
}

void Device::blacklist() {
	blacklisted = true;
}

void Device::update() {

	while (channel->receivePackageAvailable()) {

		ReceivePackage package = channel->receivePackage();

		if (package.getResponseType() == RESP_AA_BOARD) {

			// Check the application id.
			if (package.getData<uint16_t>(2) > 6) {
				applicationId = UNKNOWN;
			}
			else {
				applicationId = (ApplicationId) package.getData<uint16_t>(2);
			}

			cout << "INFO    : [" << getApplicationIdString(applicationId) << "] ";
			cout << "Device ID: 0x" << hex << package.getData<uint16_t>(0) << dec << "; ";
			cout << "Vendor ID: 0x" << hex << package.getData<uint16_t>(1) << dec << "; ";;
			cout << "Application ID: 0x" << hex << package.getData<uint16_t>(2) << dec << "; ";
			cout << "Hardware Version: 0x" << hex << package.getData<uint16_t>(3) << dec << "; ";
			cout << "Software Version: 0x" << hex << package.getData<uint16_t>(4) << dec << "; ";
			cout << "XMEGA Device ID: 0x" << hex << package.getData<uint32_t>(3) << dec << "; ";
			cout << "XMEGA Serial: 0x" << hex << package.getData<uint32_t>(4) << dec << "; ";
			cout << "Main System Clock: 0x" << hex << package.getData<uint32_t>(5) << dec << endl;

			if (package.getData<uint16_t>(4) != FIRMWARE_VERSION) {
				// TODO: Perform firmware upgrade
				throw deviceManagerException::InvalidFirmwareVersionException(
						string("Incorrect firmware version: ") + to_string(package.getData<uint16_t>(4)));
			}

			connected = true;
		}
	}

	// If device is not yet connected, retry receiving the application id.
	if (!connected) {
		TransmitPackage package(CMD_AA_GET_BOARD);
		channel->transmitPackage(package);
	}
}

ApplicationId Device::getApplicationId() {
	return applicationId;
}

DeviceManager::DeviceManager(PeripheralsInterfaceData& piData):
		started(false), desiredDeviceCount(0), knownDeviceCount(0), connectedDeviceCount(0), piData(piData) {

	channelMap.insert({MOTION_MOTOR_LEFT, shared_ptr<Communication>()});
	channelMap.insert({MOTION_MOTOR_RIGHT, shared_ptr<Communication>()});
	channelMap.insert({MOTION_MOTOR_REAR, shared_ptr<Communication>()});
	channelMap.insert({BH_MOTOR_LEFT, shared_ptr<Communication>()});
	channelMap.insert({BH_MOTOR_RIGHT, shared_ptr<Communication>()});
}

DeviceManager::~DeviceManager() {

	// Stop the device manager.
	stop();

	// Wait for the update thread to be finished.
	if (updateThread.joinable()) {
		updateThread.join();
	}
}

void DeviceManager::start() {

	if (!started) {
		started = true;
		updateThread = thread(&DeviceManager::update, this);
	}
}

void DeviceManager::stop() {
	started = false;
}

size_t DeviceManager::getNumberOfDevices() {
	return devices.size();
}

shared_ptr<Communication> DeviceManager::getChannel(ApplicationId id) {

	shared_ptr<Communication> channel = NULL;

	try {
		channel = channelMap[id];
	}
	catch (exception &e) {
		throw runtime_error("Could not find application id " + to_string(id) + ": " + e.what());
	}

	if (channel == NULL) {
		throw runtime_error("Channel not available for " + getApplicationIdString(id) + ".");
	}

	return channel;
}

void DeviceManager::update() {

	try {
		while (started) {
			updateDevices();
			updatePorts();

			// TODO: subscribe to communication channels and automatically update when data is available.
			this_thread::sleep_for(chrono::milliseconds(1000));
		}
	}
	catch (exception &e) {
		cerr << "ERROR   : DeviceManager failed: " << e.what() << endl;
		TRACE_ERROR("DeviceManager failed: %s", e.what());
	}
}

void DeviceManager::updatePorts() {
	/*
	 * The port update method has as responsibility to check what serial ports are connected and if
	 * these are online. If so, a channel is added for that port.
	 */
	ports = scanPorts("/dev/", "ttyMotor");
	piData.setNumberOfConnectedDevices(ports.size());

	for (vector<string>::iterator port = ports.begin(); port != ports.end(); port++) {
		vector<Device>::iterator device;
		for (device = devices.begin(); device != devices.end(); device++) {
			if (device->getPort() == *port) {
				break;
			}
		}

		if (device == devices.end())
		{
			try {
				devices.emplace_back(Device(*port));
				cout << "INFO    : Starting communication on " << *port << "." << endl;
			}
			catch (exception &e) {
				cerr << "ERROR   : Could not construct new device: " << e.what() << endl;
			}
		}
	}
}

void DeviceManager::updateDevices() {
	/*
	 * The devices update method has as responsibility to verify that the communication is still online
	 * (e.g. USB communication is still online), i.e.:
	 *  - If this is not the case, the client device is removed and the channel is deleted.
	 *    When the USB channel is back online, it will be reinitialized and polled again.
	 *  - If the channel is still online, verify that it is still bound to a controller.
	 *    In case no controller has been appointed, verify its application ID. If
	 *    the application ID was not retrieved, remove the channel. If the
	 *    application ID is received and correct, instantiate a device and appoint this channel.
	 *    If the application ID is unexpected, log an error - if not yet done - but keep the channel intact.
	 */

	for (vector<Device>::iterator device = devices.begin(); device != devices.end(); ) {
		bool deleted = false;

		vector<string>::iterator port = find(ports.begin(), ports.end(), device->getPort());

		if (!device->isOnline() || (port == ports.end())) {
			unregisterDevice(*device);

			if (port == ports.end()) {
				cerr << "ERROR   : Connection with " << device->getPort() << " was lost." << endl;
				TRACE_ERROR("Connection with %s was lost.", device->getPort().c_str());
				device = devices.erase(device);
				deleted = true;
			}
			else {
				try {
					cout << "INFO    : Restarting connection with " << device->getPort() << "." << endl;
					TRACE("Restarting connection with %s.", device->getPort().c_str());
					*device = Device(*port);
				}
				catch (exception &e) {
					cerr << "ERROR   : Could not open connection with " << device->getPort() << ": " << e.what() << endl;
					TRACE_ERROR("Connection not open connection with %s: %s", device->getPort().c_str(), e.what());
				}
			}
		}
		else {
			// If the serial port is still online, check if its not yet connected or blacklisted.
			if (!device->isConnected()) {
				try {
					device->update();
					if (device->isConnected())
					{
						registerDevice(*device);
					}
				}
				catch (...) {
					cout << "INFO    : Communication on " << device->getPort() << " terminated." << endl;
					device = devices.erase(device);
					deleted = true;
				}
			}
		}

		if (!deleted) {
			device++;
		}
	}
}

void DeviceManager::registerDevice(Device& device) {

	// Check if the applicationId is already present in the map
	map<ApplicationId, shared_ptr<Communication>>::iterator it = channelMap.find(device.getApplicationId());

	if (it == channelMap.end()) {
		cerr << "ERROR   : Serial port " << device.getPort() << " connected to an unknown device." << endl;
		TRACE_ERROR("Serial port %s connected to an unknown device.", device.getPort().c_str());
		device.blacklist();
	}
	else if (channelMap[device.getApplicationId()] != NULL) {
		cerr << "ERROR   : Serial port " << device.getPort() << " connected to a duplicate device." << endl;
		TRACE_ERROR("Serial port %s connected to a duplicate device.", device.getPort().c_str());
		device.blacklist();
	}
	else {
		cout << "INFO    : Added " << getApplicationIdString(device.getApplicationId());
		cout << " on port " << device.getPort() << "." << endl;
		TRACE("Added %s on port %s.",
				getApplicationIdString(device.getApplicationId()).c_str(), device.getPort().c_str());
		channelMap[device.getApplicationId()] = device.getChannel();
	}
}

void DeviceManager::unregisterDevice(Device& device) {
	if (!device.isBlacklisted()) {
		map<ApplicationId, shared_ptr<Communication>>::iterator it = channelMap.find(device.getApplicationId());

		if (it != channelMap.end()) {
			channelMap[device.getApplicationId()] = shared_ptr<Communication>();
		}
	}
}

vector<string> DeviceManager::scanPorts(string directory, string name) {
	vector<string> devices;

	DIR * dir = opendir(directory.c_str());
	struct dirent * entity;

	if (dir) {
		while ((entity = readdir(dir)) != NULL) {
			if (strncmp(entity->d_name, name.c_str(), name.length()) == 0) {
				devices.emplace_back(directory + string(entity->d_name));
			}
		}
		closedir(dir);
	}

	return devices;
}
