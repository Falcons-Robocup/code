 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * MotorManager.hpp
 *
 *  Created on: Apr 7, 2016
 *      Author: Edwin Schreuder
 */

#ifndef DEVICEMANAGER_HPP_
#define DEVICEMANAGER_HPP_

#include <atomic>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "int/motors/Communication.hpp"
#include "int/motors/PeripheralsInterfaceData.hpp"

using namespace std;

enum ApplicationId {
	UNKNOWN = 0,
	MOTION_MOTOR_LEFT = 4,
	MOTION_MOTOR_RIGHT = 2,
	MOTION_MOTOR_REAR = 3,
	BH_MOTOR_LEFT = 5,
	BH_MOTOR_RIGHT = 1,
	IO_BOARD = 6
};

string getApplicationIdString(ApplicationId applicationId);

class Device {
public:
	Device(string port);
	~Device();

	void update();
	void blacklist();

	string getPort();
	shared_ptr<Communication> getChannel();

	bool isOnline();
	bool isConnected();
	bool isBlacklisted();

	ApplicationId getApplicationId();

private:
	ApplicationId applicationId;
	shared_ptr<Communication> channel;
	bool blacklisted;
	bool connected;
};

class DeviceManager {

public:
	DeviceManager(PeripheralsInterfaceData& piData);
	~DeviceManager();

	void start();
	void stop();

	shared_ptr<Communication> getChannel(ApplicationId id);
	size_t getNumberOfDevices();

private:
	void update();
	void updateDevices();
	void updatePorts();

	void registerDevice(Device& device);
	void unregisterDevice(Device& device);

	vector<string> scanPorts(string directory, string name);

	vector<string> ports;
	vector<Device> devices;

	atomic<bool> started;
	thread updateThread;

	size_t desiredDeviceCount;
	size_t knownDeviceCount;
	size_t connectedDeviceCount;

	PeripheralsInterfaceData& piData;

	map<ApplicationId, shared_ptr<Communication>> channelMap;
};

#endif /* MOTORMANAGER_HPP_ */
