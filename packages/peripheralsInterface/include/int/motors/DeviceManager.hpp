// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
