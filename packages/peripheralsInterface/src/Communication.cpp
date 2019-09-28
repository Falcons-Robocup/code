 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Serial.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Edwin Schreuder
 */

#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <sys/eventfd.h>
#include <sys/select.h>
#include <fcntl.h>
#include <int/PeripheralsInterfaceExceptions.hpp>

#include "FalconsCommon.h"

#include "int/Communication.hpp"

#include "int/Serial.hpp"

using namespace std;

Communication::Communication(string portName):
		serialPort(portName, B115200, 0, FROM_BOARD_PACKET_HEADER_SIZE), callback(NULL) {

	wakeUpEventFd = eventfd(1, O_NONBLOCK);
	if (wakeUpEventFd == -1) {
		throw runtime_error("Could not create file descriptor for wake up event.");
	}

	reset();
	start();
}

Communication::~Communication() {
	// Stop the read/write threads.
	try {
		stop();
	}
	catch (exception &e) {
		cerr << "WARNING : Could not stop communication object." << endl;
	}

	close(wakeUpEventFd);
}

void Communication::subscribe(ReceiveNotificationFunction function) {
	callback = function;
}

void Communication::unsubscribe() {
	callback = NULL;
}

void Communication::start() {
	bufferSpace = 0;
	bytesSentDuringCurrentfarEndBufferSpaceRefresh = 0;
	bytesSentDuringMin1farEndBufferSpaceRefresh = 0;
	bytesSentDuringMin2farEndBufferSpaceRefresh = 0;

	receiveBufferLength = 0;
	packageStartIndex = 0;
	packageStartFound = false;
	transmitAckId = 0x00;
	receiveAckId = 0x00;

	receiveQueue = queue<ReceivePackage>();
	transmitQueue = queue<TransmitPackage>();

	// Start the read/write threads.
	receiveThreadShutDown = false;
	receiveThread = thread(&Communication::receiveUpdate, this);
	transmitThreadShutDown = false;
	transmitThread = thread(&Communication::transmitUpdate, this);

	// We need to make sure that the spawned threads are running and waiting for condition variables.
	// Therefore a small sleep is inserted (not ideal).
	this_thread::sleep_for(chrono::milliseconds(1));
}

void Communication::stop() {
	receiveThreadShutDown = true;
	transmitThreadShutDown = true;

	// Wake up the receive thread by writing the eventfd.
	uint64_t value = 1;
	ssize_t result = write(wakeUpEventFd, &value, sizeof(value));
	if (result < 0) {
		throw runtime_error("Could not send wakeup event.");
	}

	// Wake up the transmit thread by setting the transmitQueue and bufferSpaceAvailable.
	transmitQueueMutex.lock();
	transmitQueueAvailable.notify_one();
	transmitQueueMutex.unlock();
	bufferSpaceMutex.lock();
	bufferSpaceAvailable.notify_one();
	bufferSpaceMutex.unlock();

	if (transmitThread.joinable()) {
		transmitThread.join();
	}
	if (receiveThread.joinable()) {
		receiveThread.join();
	}
}

void Communication::reset() {
	// Reset the board. Run the reset loop twice, because the DTR could already be active.
	for (size_t i = 0; i < 2; i++) {
		// Turn on DTR
		serialPort.setDTR();
		// Turn off DTR
		serialPort.unsetDTR();
	}
	try {
		serialPort.flushPort();
	}
	catch (exception &e) {
		cerr << "ERROR   : " << e.what() << endl;
	}
}

bool Communication::receivePackageAvailable()
{
	receiveQueueMutex.lock();
	bool available = (receiveQueue.size() > 0);
	receiveQueueMutex.unlock();

	return available;
}

ReceivePackage Communication::receivePackage()
{
	receiveQueueMutex.lock();
	ReceivePackage package = receiveQueue.front();
	receiveQueue.pop();
	receiveQueueMutex.unlock();

	return package;
}

void Communication::transmitPackage(TransmitPackage &package)
{
	transmitQueueMutex.lock();
	transmitQueue.push(package);
	transmitQueueAvailable.notify_one();
	transmitQueueMutex.unlock();
}

bool Communication::isOnline() {
	bool online = false;

	if (!receiveThreadShutDown && !transmitThreadShutDown) {
		online = true;
	}

	return online;
}

string Communication::getPortName() {
	return serialPort.getPortName();
}

/*
 * PACKAGE RECEIVE FUNCTIONS
 */
void Communication::receiveUpdate(void) {
	/*
	 * The responsibility of the receiveUpdate function is to do a blocking read on the serial port,
	 * and update the receive buffer with the received data. When what looks like a full package has
	 * been received, process the receive buffer as incoming package and stuff it in the receive buffer.
	 */

	size_t readLength;
	fd_set active_fd_set;

	FD_ZERO(&active_fd_set);
	FD_SET(serialPort.getFileDescriptor(), &active_fd_set);
	FD_SET(wakeUpEventFd, &active_fd_set);

	try {
		while (true) {
			receiveQueueMutex.lock();
			size_t receiveQueueSize = receiveQueue.size();
			receiveQueueMutex.unlock();

			fd_set read_fd_set = active_fd_set;
			struct timeval timeout = {0, 100000};
			if (select(FD_SETSIZE, &read_fd_set, NULL, NULL, &timeout) < 0)
			{
				throw runtime_error("Select function returned an error.");
			}

			if (FD_ISSET(serialPort.getFileDescriptor(), &read_fd_set) || FD_ISSET(wakeUpEventFd, &read_fd_set)) {
				if (FD_ISSET(serialPort.getFileDescriptor(), &read_fd_set)) {
					readLength = serialPort.readPort(&receiveBuffer[receiveBufferLength], COMMUNICATION_RECEIVE_BUFFER_SIZE - receiveBufferLength);

					if (readLength > 0) {
						receiveBufferLength += readLength;
						processReceiveData();

						if (receiveBufferLength == packageStartIndex) {
							// To ensure that the buffer does not overflow, set the receivebuffer to zero when
							// The start of the next packet is outside data in the receivebuffer.
							receiveBufferLength = 0;
							packageStartIndex = 0;
						} else {
							if (receiveBufferLength == COMMUNICATION_RECEIVE_BUFFER_SIZE) {
								// If the buffer is full but an partial package is present, copy the already
								// received bytes to the beginning of the buffer and wait for new data.
								memcpy(receiveBuffer, &receiveBuffer[packageStartIndex], COMMUNICATION_RECEIVE_BUFFER_SIZE - packageStartIndex);
								receiveBufferLength -= packageStartIndex;
								packageStartIndex = 0;
							}
						}
					}
					else {
						throw runtime_error("Serial port has shut down.");
					}
				}

				if (FD_ISSET(wakeUpEventFd, &read_fd_set)) {
					uint64_t value;
					ssize_t result = read(wakeUpEventFd, &value, sizeof(value));

					if ((result < 0) && receiveThreadShutDown) {
						throw runtime_error("receiveThreadShutDown was raised.");
					}
				}
				else {
					receiveQueueMutex.lock();
					if (receiveQueue.size() > receiveQueueSize) {
						if (callback) {
							callback();
						}
					}
					receiveQueueMutex.unlock();
				}
			}
			else {
				throw runtime_error("Timeout occurred, communication no longer online.");
			}
		}
	}
	catch (exception &e) {
//		cerr << "Receive thread has stopped. Communication NOK: " << e.what() << endl;
	}

	receiveThreadShutDown = true;
}

void Communication::processReceiveData() {
	/*
	 * The responsibility of the processReceiveData function is scan the input data buffer to process the incoming data
	 * and try to form a package with what is now in the input buffer. If so, return that a full package has been found.
	 */
	bool newPackageReceived;

	do {
		newPackageReceived = false;

		// Find the SOF of a new package if necessary.
		for (size_t bufferIndex = packageStartIndex; (bufferIndex < receiveBufferLength) && !packageStartFound; bufferIndex++) {
			if (receiveBuffer[bufferIndex] == SOF) {
				packageStartIndex = bufferIndex;
				packageStartFound = true;
			}
		}

		// If a SOF is found, try to see if a full package is already in the buffer.
		if (packageStartFound) {
			if ((packageStartIndex + 6) < receiveBufferLength) {
				size_t packageLength = receiveBuffer[packageStartIndex + 6] + FROM_BOARD_PACKET_HEADER_SIZE;
				if ((packageLength + packageStartIndex) < receiveBufferLength) {
					// A new package is in the buffer, so create a receive package object and process.
					ReceivePackage package(&(receiveBuffer[packageStartIndex]), packageLength);

					processReceivePackage(package);

					packageStartIndex += packageLength;
					packageStartFound = false;

					newPackageReceived = true;
				}
			}
		}
	} while (newPackageReceived);
}

void Communication::processReceivePackage(ReceivePackage &package) {
	/*
	 * This function is responsible for verifying that the received package is
	 * valid (CRC correct) and add it to the queue of received packages.
	 * When the AckId is invalid, an error should be logged.
	 */

	if (!package.isPacketValid()) {
		cerr << "ERROR   : A package with incorrect CRC was received, package is lost." << endl;
	} else {
		if (package.getAckId() != receiveAckId) {
			cerr << "ERROR   : AckId of current package (" << (int) package.getAckId();
			cerr << ") was unequal to the expected value (" << (int) receiveAckId << ")." << endl;
		}
		receiveAckId = package.getAckId() + 1;

		receiveQueueMutex.lock();
		receiveQueue.push(package);
		receiveQueueMutex.unlock();

		bufferSpaceMutex.lock();

		bufferSpace = package.getBufferSpace();
		bytesSentDuringMin2farEndBufferSpaceRefresh =
				bytesSentDuringMin1farEndBufferSpaceRefresh;
		bytesSentDuringMin1farEndBufferSpaceRefresh =
				bytesSentDuringCurrentfarEndBufferSpaceRefresh;
		bytesSentDuringCurrentfarEndBufferSpaceRefresh = 0;

		bufferSpaceAvailable.notify_one();
		bufferSpaceMutex.unlock();
	}
}

/*
 * PACKAGE TRANSMIT FUNCTIONS
 */
void Communication::transmitUpdate(void) {
	/*
	 * The responsibility of the transmitUpdate function is to do a blocking wait on the transmit package queue,
	 * and once a new package is available send it.
	 */
	unique_lock<mutex> lock(transmitQueueMutex);

	try {
		while (true) {
			/* Lock the transmitQueue and receive a package queued to be updated.
			 * Here a condition variable is used to ensure that we only wake up when there is actually data added. */

			transmitQueueAvailable.wait(lock);

			if (transmitThreadShutDown) {
				throw runtime_error("transmitThreadShutDown was raised.");
			}

			while (transmitQueue.size() > 0)
			{
				// Get the first package in the queue.
				TransmitPackage package = transmitQueue.front();

				// Remove the item from the queue.
				transmitQueue.pop();

				// Unlock the transmitQueueMutex, as we already have a copied the data.
				lock.unlock();

				// Process the package (e.g. send it).
				processTransmitPackage(package);

				// Lock the transmitQueueMutex so we can recheck if there is data to send.
				lock.lock();
			}
		}
	}
	catch (exception &e) {
//		cerr << "Transmit thread has stopped. Communication NOK: " << e.what() << endl;
	}

	transmitThreadShutDown = true;
}

void Communication::processTransmitPackage(TransmitPackage &package) {
	/*
	 * The responsibility of the processTransmitPackage is to wait for far end buffer size to be
	 * sufficient to be able to send the incoming transmit package. If the buffer size is sufficient,
	 * we send the package.
	 */
	bool packageSent = false;
	unique_lock<mutex> lock(bufferSpaceMutex);

	while (!packageSent) {
		bufferSpaceAvailable.wait(lock);

		if (transmitThreadShutDown) {
			throw runtime_error("transmitThreadShutDown was raised.");
		}

		size_t worstCaseFarEndAvailableSize = bufferSpace
				- bytesSentDuringCurrentfarEndBufferSpaceRefresh
				- bytesSentDuringMin1farEndBufferSpaceRefresh
				- bytesSentDuringMin2farEndBufferSpaceRefresh;

		if (worstCaseFarEndAvailableSize > package.getLength())
		{
			bytesSentDuringCurrentfarEndBufferSpaceRefresh += package.getLength();
			package.setAckId(transmitAckId);
			transmitAckId++;

			serialPort.writePort(package.getBuffer(), package.getLength());
			packageSent = true;
		}
		lock.unlock();
	}
}
