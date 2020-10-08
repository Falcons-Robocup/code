 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
