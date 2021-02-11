// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef INCLUDED_COMMUNICATION_HPP
#define INCLUDED_COMMUNICATION_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "CommunicationPackage.hpp"
#include "../Serial.hpp"

using namespace std;

#define COMMUNICATION_RECEIVE_BUFFER_SIZE (1024)

typedef void (*ReceiveNotificationFunction)();

class Communication {

public:
	Communication(std::string portName);
	~Communication();

	bool receivePackageAvailable();
	ReceivePackage receivePackage();

	void transmitPackage(TransmitPackage &package);

	bool isOnline();
	string getPortName();

	void reset();

	void subscribe(ReceiveNotificationFunction function);
	void unsubscribe();

private:
	void start();
	void stop();

	void receiveUpdate();
	void processReceiveData();
	void processReceivePackage(ReceivePackage &package);

	void transmitUpdate();
	void processTransmitPackage(TransmitPackage &package);

	Serial serialPort;
	ReceiveNotificationFunction callback;

	thread receiveThread;
	std::atomic<bool> receiveThreadShutDown;
	queue<ReceivePackage> receiveQueue;
	mutex receiveQueueMutex;

	thread transmitThread;
	std::atomic<bool> transmitThreadShutDown;
	queue<TransmitPackage> transmitQueue;
	mutex transmitQueueMutex;
	condition_variable transmitQueueAvailable;

	int wakeUpEventFd;

	size_t bufferSpace;
	size_t bytesSentDuringCurrentfarEndBufferSpaceRefresh;
	size_t bytesSentDuringMin1farEndBufferSpaceRefresh;
	size_t bytesSentDuringMin2farEndBufferSpaceRefresh;
	mutex bufferSpaceMutex;
	condition_variable bufferSpaceAvailable;

	unsigned char receiveBuffer[COMMUNICATION_RECEIVE_BUFFER_SIZE];
	size_t receiveBufferLength;
	size_t packageStartIndex;
	bool packageStartFound;

	unsigned char receiveAckId;
	unsigned char transmitAckId;
};

#endif /* INCLUDED_COMMUNICATION_HPP */
