 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm> // std::sort
#include <cerrno>
#include <dirent.h> // for DIR and struct dirent
#include <fcntl.h> // for O_RDWR
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ttyUsb.hpp"
#include "motorBoard.hpp"

using namespace std;

typedef enum threadIdentifierList {
	THREAD_AA_DO_NOT_USE_ZERO = 0,
	THREAD_BALL_LEFT_RECEIVE,
	THREAD_BALL_LEFT_SEND,
	THREAD_BALL_RIGHT_RECEIVE,
	THREAD_BALL_RIGHT_SEND,
	THREAD_COMMUNICATION,
	THREAD_COMPASS,
	THREAD_WHEEL_LEFT_RECEIVE,
	THREAD_WHEEL_LEFT_SEND,
	THREAD_WHEEL_REAR_RECEIVE,
	THREAD_WHEEL_REAR_SEND,
	THREAD_WHEEL_RIGHT_RECEIVE,
	THREAD_WHEEL_RIGHT_SEND,
	THREAD_ZZ_DO_NOT_USE_LAST
} threadIdentifierT;

// globals that are shared between multiple threads
serial ballLeftSerial, ballRightSerial, compassSerial, wheelLeftSerial, wheelRearSerial, wheelRightSerial;

bool ballReceiveStopThread;
size_t ballLeftReceivePrintCounter = 0;
size_t ballLeftSendPrintCounter = 0;

// test variables for the communication thread to send some test data to the boards
bool wheelRearIncrease = false;
int wheelRearSetPoint = 0;
bool wheelLeftIncrease = false;
int wheelLeftSetPoint = 0;
bool wheelRightIncrease = false;
int wheelRightSetPoint = 0;

bool ballLeftIncrease = false;
int ballLeftSetPoint = 0;
uint16_t ballLeftAngleZero = 0;
bool ballRightIncrease = false;
int ballRightSetPoint = 0;
uint16_t ballRightAngleZero = 0;


uint8_t compassValue = 0;

ttyUsb::ttyUsb( ) {

	deviceList.clear(); // when starting none of the ttyUSB devices shall be used

	// initialize all required functions
	init( &ball.left.receive, THREAD_BALL_LEFT_RECEIVE, "BLER", "ball left receive" );
	init( &ball.left.send, THREAD_BALL_LEFT_SEND, "BLES", "ball left send" );

	init( &ball.right.receive, THREAD_BALL_RIGHT_RECEIVE, "BRIR", "ball right receive" );
	init( &ball.right.send, THREAD_BALL_RIGHT_SEND, "BRIS", "ball right send" );

	init( &communication, THREAD_COMMUNICATION, "COMU", "communication" );

	init( &compass, THREAD_COMPASS, "CMPS", "compass" );


	init( &wheel.left.receive, THREAD_WHEEL_LEFT_RECEIVE, "WLER", "wheel left receive" );
	init( &wheel.left.send, THREAD_WHEEL_LEFT_SEND, "WLES", "wheel left send" );

	init( &wheel.rear.receive, THREAD_WHEEL_REAR_RECEIVE, "WRER", "wheel rear receive" );
	init( &wheel.rear.send, THREAD_WHEEL_REAR_SEND, "WRES", "wheel rear send" );

	init( &wheel.right.receive, THREAD_WHEEL_RIGHT_RECEIVE, "WRIR", "wheel right receive" );
	init( &wheel.right.send, THREAD_WHEEL_RIGHT_SEND, "WRIS", "wheel right send" );
}

// initialize all values
void ttyUsb::init( threadSt* context, size_t identifier, string prefix, string name ) {
	context->identifier = identifier;
	context->prefix = prefix;
	context->name = name;
	initDynamicValues( context, -1 );  // use -1 for deviceIndex when no device is not available (e.g. communication thread)
}

// initialize all dynamic values
void ttyUsb::initDynamicValues( threadSt* context, int deviceIndex ) {
	context->counter = 0;
	context->counterPrevious = 0;
	context->deviceIndex = deviceIndex;
	context->freezeCounter = 0;
	context->started = false;
	context->threadId = 0;
}

void ttyUsb::threadStart( threadSt *context, int deviceIndex ) {
	initDynamicValues( context, deviceIndex );

	if( pthread_create( &(context->threadId), NULL, threadProcess, context ) ) {
		printf( "%s ERROR   : %s, cannot start thread for %s\n", context->prefix.c_str(), strerror(errno), context->name.c_str() );
		exit( EXIT_FAILURE );
	}
	// It might take some time before the thread is started.
	// To prevent unexpected behavior wait until the thread has been started
	while( ! context->started ) {
		// printf("%s INFO    : waiting for %s to start xxxxxxxxxxxxxxx\n", context->prefix.c_str(), context->name.c_str() );
		usleep( 100 ); // give it 100us to switch to other task and try again
	}
}

void ttyUsb::threadCancel( threadSt* context ) {
	if( pthread_cancel( context->threadId ) ) {
		printf( "%s ERROR   : %s, cannot cancel thread for %s\n", context->prefix.c_str(), strerror(errno), context->name.c_str() );
		exit( EXIT_FAILURE );
	}

	void *res;
	if( pthread_join(ball.left.receive.threadId, &res) ) {
		printf( "%s ERROR   : %s, cannot join thread after cancel for %s\n", context->prefix.c_str(), strerror(errno), context->name.c_str() );
		exit( EXIT_FAILURE );
	}
	context->started = false;
}

void ttyUsb::threadTest( threadSt* context ) {
	if( context->started ) {
		printf("%s INFO    : still running for %s with counter %lu\n", context->prefix.c_str(), context->name.c_str(), context->counter );
	} else {
		// printf("%s INFO    : try to start thread for %s\n", context->prefix.c_str(), context->name.c_str() );
		threadStart( context, -1 );
		threadCancel( context );
	}
}

// return true if thread is running
bool ttyUsb::threadCheck( threadSt* context ) {
	bool retVal = false;
	if( context->started ) {
		// printf("%s INFO    : still running for %s with counter %lu\n", context->prefix.c_str(), context->name.c_str(), context->counter );
		// check if thread is still doing something by checking the value of the counter
		if( context->counter == context->counterPrevious ) {
			context->freezeCounter++;
		} else {
			context->freezeCounter = 0;
		}
		context->counterPrevious = context->counter;
		if( context->freezeCounter > 1000 ) {
			printf( "%s ERROR   : %s thread is not active for the last %lu tests\n", context->prefix.c_str(), context->name.c_str(), context->freezeCounter );
		} else {
			retVal = true; // thread started and functioning
		}
	}
	return retVal;
}

// when used (threads started) indicated this in the deviceList
void ttyUsb::setDeviceUsed( int deviceIndex) {
	// Note: the deviceList is only used by the ttyUsb::process
	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		if( deviceList[ii].index == deviceIndex ) {
			printf( "___  INFO    : add device /dev/ttyUSB%d to device list\n", deviceList[ii].index );
			deviceList[ii].used = true;
		}
	}
}

// remove device from device list because e.g. the USB device is disconnected
void ttyUsb::removeDeviceFromList( int deviceIndex) {
	// Note: the deviceList is only used by the ttyUsb::process
	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		if( deviceList[ii].index == deviceIndex ) {
			printf( "___  INFO    : remove device /dev/ttyUSB%d from device list\n", deviceList[ii].index );
			deviceList[ii].used = false;
			deviceList[ii].function = FUNCTION_UNINITIALIZED;
		}
	}
}

// only for testing, not required for the robocup functionality
void tester( serial *ser, int &setPoint, bool &increase, uint16_t angleZero, bool ball, size_t counter, char *prefix) {
	for( size_t ii = 0; ii < ser->fromBoardPacket.size( ); ii++ ) {
		fromBoardPacketT rxPacket = ser->fromBoardPacket.front( );
		ser->fromBoardPacket.pop( );
		if( rxPacket.responseType == RESP_ERROR ) {
			printf("%s WARNING : received the following error message from far end\n", prefix );
			ser->printReceivedPacket( rxPacket );
			ser->printRemoteErrors( rxPacket );
			ser->sendClearKnownErrors( rxPacket );
		} else {
			ser->printReceivedPacket( rxPacket );
		}
	}

	if( ser->getBoardConfigured( ) ) {
		// the board is configured, now we can once in a while send a new setpoint
		if( ball ) {
			// the ball handler boards and wheel motors use different setpoints
			if( ( counter % 100 ) == 0 ) {
				if( increase ) { setPoint++; } else { setPoint--; }
				if( setPoint > 100 ) { increase = false; }
				else if( setPoint < -100 ) { increase = true; }
				ser->send2BytesPayloadXXXX( CMD_SET_ANGLE_SETPOINT, (uint16_t)(angleZero + setPoint) );
			}
		} else {
			if( ( counter % 400 ) == 0 ) {
				// send often set point, but not to fast to minimize response latency
				if( increase ) { setPoint++; } else { setPoint--; }
				if( setPoint > 20 ) { increase = false; }
				else if( setPoint < -20 ) { increase = true; }
				ser->send2BytesPayloadXXXX( CMD_SET_PRIMARY_SETPOINT, (uint16_t)setPoint );
			}
		}
	}
}

void updateCommunication( threadSt *context ){
	// TODO: add send information to boards to communication function
	tester( &ballLeftSerial, ballLeftSetPoint, ballLeftIncrease, ballLeftAngleZero, true, context->counter, (char *)"BLER" ); // ball left test
	tester( &ballRightSerial, ballRightSetPoint, ballRightIncrease, ballRightAngleZero, true, context->counter, (char *)"BRIR" ); // ball right test
	tester( &wheelLeftSerial, wheelLeftSetPoint, wheelLeftIncrease, 0, false, context->counter, (char *)"WLER" ); // wheel left test
	tester( &wheelRearSerial, wheelRearSetPoint, wheelRearIncrease, 0, false, context->counter, (char *)"WRER" ); // wheel rear test
	tester( &wheelRightSerial, wheelRightSetPoint, wheelRightIncrease, 0, false, context->counter, (char *)"WRIR" ); // wheel right test

	// ## compass ##
	if( ( ( context->counter % 20000 ) == 0 ) && ( compassSerial.getDeviceOpen( ) ) ) {
		printf( "CMPS INFO    : compass value %3u\n", compassValue );
	}
	usleep( 200 );
}

void receiveHelper( serial *ser, threadSt *context ) {
	// the receiver thread opens and closes the device
	ser->moborBoardOpen( context->deviceIndex );
	while( ! ser->getFailing( ) ) {
		ser->checkReceive( ); // check if bytes have been received lately
		ser->receiveBytes( false, true ); // collect new bytes and store in queue
		context->counter++;
		// This tread will NOT consume 100% CPU time because the serial read is blocking until the VTIME expires (e.g. 300ms)
	}
	// before closing the device the receiver needs to wait until the sender has released the device
	printf( "%s INFO    : thread for %s wants to stop, wait until sender thread stopped\n", context->prefix.c_str(), context->name.c_str() );
	while( ! ser->getSendStopped( ) ) { usleep( 1000 ); }
	ser->setBoardConfigured( false ); // for a new connection the board shall be configured again
	ser->closeDevice( true );
}

void sendHelper( serial *ser, threadSt *context, bool ball, uint16_t &angleZero ) {
	// the sender needs to wait until the receiver opens the device
	while( ! ser->getDeviceOpen( ) ) { usleep( 100 ); } // wait until receiver thread has opened the device
	while( ! ser->getFailing( ) ) {
		if( ser->toBoardPacket.size( ) > 0 ) {
			ser->sendToBoardPackets( );
		} else if ( ! ser->getBoardConfigured( ) ) {
			// setup only
			if( ball ) {
				ser->ballHandlerBoardConfigure( );
				angleZero = ser->getAngleZero();
			} else {
				ser->wheelBoardConfigure( );
			}
		}
		usleep( 100 ); // switch treads, TODO: awake when triggered by communication thread instead of using usleep
		context->counter++;
	}
	ser->setSendStopped( ); // inform receiver it can close the device
}


void* ttyUsb::threadProcess( void *arg ) {
	struct threadSt *context = (struct threadSt*) arg;
	uint16_t tmp;
	if( context->started ) {
		printf( "%s ERROR   : %s thread was already running, this shall never happen, abort\n", context->prefix.c_str(), context->name.c_str() );
		exit( EXIT_FAILURE );
	}
	context->started = true;
	printf( "%s INFO    : thread for %s has been started, counter %lu\n", context->prefix.c_str(), context->name.c_str(), context->counter );
	if( context->identifier == THREAD_COMMUNICATION ) {
		while( true ) {
			updateCommunication( context );
			// TODO: awake this thread when triggered by one of the receiver threads or outside world instead of usleep / poll
			usleep( 100 ); // switch threads
			context->counter++;
		}
	}
	else if( context->identifier == THREAD_WHEEL_REAR_RECEIVE ) { receiveHelper( &wheelRearSerial, context ); }
	else if( context->identifier == THREAD_WHEEL_REAR_SEND ) { sendHelper( &wheelRearSerial, context, false, tmp ); }
	else if( context->identifier == THREAD_WHEEL_RIGHT_RECEIVE ) { receiveHelper( &wheelRightSerial, context ); }
	else if( context->identifier == THREAD_WHEEL_RIGHT_SEND ) { sendHelper( &wheelRightSerial, context, false, tmp ); }
	else if( context->identifier == THREAD_WHEEL_LEFT_RECEIVE ) { receiveHelper( &wheelLeftSerial, context ); }
	else if( context->identifier == THREAD_WHEEL_LEFT_SEND ) { sendHelper( &wheelLeftSerial, context, false, tmp ); }
	else if( context->identifier == THREAD_BALL_LEFT_RECEIVE ) { receiveHelper( &ballLeftSerial, context ); }
	else if( context->identifier == THREAD_BALL_LEFT_SEND ) { sendHelper( &ballLeftSerial, context, true, ballLeftAngleZero ); }
	else if( context->identifier == THREAD_BALL_RIGHT_RECEIVE ) { receiveHelper( &ballRightSerial, context ); }
	else if( context->identifier == THREAD_BALL_RIGHT_SEND ) { sendHelper( &ballRightSerial, context, true, ballRightAngleZero ); }
	else if( context->identifier == THREAD_COMPASS ) {
		compassSerial.compassOpen( context->deviceIndex );
		while( ! compassSerial.getFailing( ) ) {
			compassValue = compassSerial.compassGetAngle( );
			// This tread will not consume 100% CPU time because the serial read is blocking until the VTIME expires (e.g. 300ms)
		}
		compassSerial.closeDevice( true );
	} else {
		// TODO: when all functions are implemented print error when this occurs and abort
		while( true ) {
			usleep( 1000 ); // switch threads
			context->counter++;
		}
	}

	context->started = false; // inform higher level the thread is not running anymore
	printf( "%s ERROR   : %s thread exit (probably non functional)\n", context->prefix.c_str(), context->name.c_str() );
	return NULL;	
}

// update the list of available ttyUSB devices
// keep the properties for already know devices (from previous scans)
void ttyUsb::updateDeviceList( ) {
	DIR *dp;
	struct dirent *ep;

	dp = opendir("/dev");
	if( dp == NULL ) {
		printf( "USB  ERROR   : %s when opening /dev for reading\n", strerror(errno) );
		exit( EXIT_FAILURE );
	}

	// create list of all currently available ttyUSB devices
	vector<deviceSt> newList;
	while( ( ep = readdir(dp) ) ) {
		if( memcmp( ep->d_name, "ttyUSB", 6 ) == 0 ) { // check if the file handle is ttyUSB by comparing the first 6 characters of ep->d_name
			char stringIndex[4] = "";
			// extract the number from ttyUSBxxx
			strncpy( stringIndex, ep->d_name +6, 3 ); // unlikely there will be more then 99 ttyUSB devices (and on char for "\n")
			int index = atoi(stringIndex);
			// printf( "USB  INFO    : found device /dev/ttyUSB%d\n", index );
			// add the new found device to the list
			deviceSt device;
			device.index = index;
			device.used = false; // set default to not used
			device.function = FUNCTION_UNINITIALIZED; // set default to not initialized
			newList.push_back(device);
		}
	}
	closedir( dp );
	// now we have a complete list of all current available ttyUSB devices

	// determine which of the ttyUSB devices where already known
	for( size_t kk = 0; kk < newList.size(); kk++ ) {
		// Note: the deviceList is only used by the ttyUsb::process
		for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
			if( deviceList[ii].index == newList[kk].index ) {
				// so this device was already found in an earlier scan
				if( deviceList[ii].used ) {
					newList[kk].used = true;
					newList[kk].function = deviceList[ii].function;
				}
			}
		}
	}
	sort(newList.begin(), newList.end()); // order the /dev/ttyUSB0 to /dev/ttyUSB99

	// now we have all ttyUSB devices and if they already where used
	// Note: the deviceList is only used by the ttyUsb::process
	deviceList = newList;
}

// register device
void ttyUsb::identifyBoards( ) {
	// Note: the deviceList is only used by the ttyUsb::process
	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		if( deviceList[ii].function == FUNCTION_UNINITIALIZED ) {
			serial ser;
			deviceList[ii].function = ser.getFunction( deviceList[ii].index );
		}
	}
}

// returns true if all boards are available (boards connected to /dev/ttyUSBx port)
bool ttyUsb::allBoardsAvailable( ) {
	bool ballLeft = false;
	bool ballRight = false;
	bool compass = false;
	bool wheelLeft = false;
	bool wheelRear = false;
	bool wheelRight = false;

	// Note: the deviceList is only used by the ttyUsb::process
	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		if( deviceList[ii].function == FUNCTION_BALL_LEFT ) { ballLeft = true; }
		if( deviceList[ii].function == FUNCTION_BALL_RIGHT ) { ballRight = true; }
		if( deviceList[ii].function == FUNCTION_COMPASS ) { compass = true; }
		if( deviceList[ii].function == FUNCTION_WHEEL_LEFT ) { wheelLeft = true; }
		if( deviceList[ii].function == FUNCTION_WHEEL_REAR ) { wheelRear = true; }
		if( deviceList[ii].function == FUNCTION_WHEEL_RIGHT ) { wheelRight = true; }
	}
	return( ballLeft && ballRight && compass && wheelLeft && wheelRear && wheelRight );
}

void ttyUsb::registerOneFunction( threadSt* context, int deviceIndex ) {
	if( context->started ) {
		if( context->deviceIndex == deviceIndex ) {
			// thread is already running and using the correct index (/dev/ttyUSB device)
		} else {
			printf( "%s ERROR   : function %s already running and using /dev/ttyUSB%d but new /dev/ttyUSB%d found that also requires this function\n",
					context->prefix.c_str(), context->name.c_str(), context->deviceIndex, deviceIndex );
			exit( EXIT_FAILURE );
		}
	} else {
		// function was not yet running, start now
		// printf("%s INFO    : try to start thread for %s\n", context->prefix.c_str(), context->name.c_str() );
		threadStart( context, deviceIndex );
	}
}

// for all device check if the function (threads) need to be started
void ttyUsb::registerNewFunctions( ) {
	// Note: the deviceList is only used by the ttyUsb::process
	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		if( ! deviceList[ii].used ) {
			// only start when not used (the registerOneFunction sets the used flag)
			if( deviceList[ii].function == FUNCTION_BALL_LEFT ) {
				ballLeftSerial.init( ball.left.receive.prefix, ball.left.send.prefix, ball.left.receive.name, true ); // clear all counters and errors and set names
				ballReceiveStopThread = false;
				setDeviceUsed( deviceList[ii].index );
				registerOneFunction( &ball.left.receive, deviceList[ii].index );
				registerOneFunction( &ball.left.send, deviceList[ii].index );
			} else if( deviceList[ii].function == FUNCTION_BALL_RIGHT ) {
				ballRightSerial.init( ball.right.receive.prefix, ball.right.send.prefix, ball.right.receive.name, true );
				setDeviceUsed( deviceList[ii].index );
				registerOneFunction( &ball.right.receive, deviceList[ii].index );
				registerOneFunction( &ball.right.send, deviceList[ii].index );
			} else if( deviceList[ii].function == FUNCTION_COMPASS ) {
				compassSerial.init( compass.prefix, compass.prefix, compass.name, true );
				setDeviceUsed( deviceList[ii].index );
				registerOneFunction( &compass, deviceList[ii].index );
			} else if( deviceList[ii].function == FUNCTION_WHEEL_LEFT ) {
				wheelLeftSerial.init( wheel.left.receive.prefix, wheel.left.send.prefix, wheel.left.receive.name, true );
				setDeviceUsed( deviceList[ii].index );
				registerOneFunction( &wheel.left.receive, deviceList[ii].index );
				registerOneFunction( &wheel.left.send, deviceList[ii].index );
			} else if( deviceList[ii].function == FUNCTION_WHEEL_REAR ) {
				wheelRearSerial.init( wheel.rear.receive.prefix, wheel.rear.send.prefix, wheel.rear.receive.name, true );
				setDeviceUsed( deviceList[ii].index );
				registerOneFunction( &wheel.rear.receive, deviceList[ii].index );
				registerOneFunction( &wheel.rear.send, deviceList[ii].index );
			} else if( deviceList[ii].function == FUNCTION_WHEEL_RIGHT ) {
				wheelRightSerial.init( wheel.right.receive.prefix, wheel.right.send.prefix, wheel.right.receive.name, true );
				setDeviceUsed( deviceList[ii].index );
				registerOneFunction( &wheel.right.receive, deviceList[ii].index );
				registerOneFunction( &wheel.right.send, deviceList[ii].index );
			}
		}
	}
	registerOneFunction( &communication, -1 );
}

// returns true if all functions are working correctly (all threads are running and updated)
bool ttyUsb::allFunctionsOperational( ) {
	if( ! threadCheck( &ball.left.receive ) ) { return false; }
	if( ! threadCheck( &ball.left.send ) ) { return false; }
	if( ! threadCheck( &ball.right.receive ) ) { return false; }
	if( ! threadCheck( &ball.right.send ) ) { return false; }
	if( ! threadCheck( &communication ) ) { return false; }
	if( ! threadCheck( &compass ) ) { return false; }
	if( ! threadCheck( &wheel.left.receive ) ) { return false; }
	if( ! threadCheck( &wheel.left.send ) ) { return false; }
	if( ! threadCheck( &wheel.rear.receive ) ) { return false; }
	if( ! threadCheck( &wheel.rear.send ) ) { return false; }
	if( ! threadCheck( &wheel.right.receive ) ) { return false; }
	if( ! threadCheck( &wheel.right.send ) ) { return false; }
	return true;
}

// remove device from device list when not used by any thread
void ttyUsb::unRegisterNonWorkingDevices( ){
	if( ball.left.receive.started || ball.left.send.started ) {
		// at least one thread is still using the device
	} else {
		if( ball.left.receive.deviceIndex != ball.left.send.deviceIndex ) {
			printf("BLE  ERROR   : inconsistent device index receive %d send %d, abort\n", ball.left.receive.deviceIndex, ball.left.send.deviceIndex );
			exit( EXIT_FAILURE );
		}
		removeDeviceFromList( ball.left.receive.deviceIndex );
	}

	if( ball.right.receive.started || ball.right.send.started ) {
		// at least one thread is still using the device
	} else {
		if( ball.right.receive.deviceIndex != ball.right.send.deviceIndex ) {
			printf("BRI  ERROR   : inconsistent device index receive %d send %d, abort\n", ball.right.receive.deviceIndex, ball.right.send.deviceIndex );
			exit( EXIT_FAILURE );
		}
		removeDeviceFromList( ball.right.receive.deviceIndex );
	}

	// compass uses one thread for send and receive
	if( ! compass.started ) { removeDeviceFromList( compass.deviceIndex ); }

	if( wheel.left.receive.started || wheel.left.send.started ) {
		// at least one thread is still using the device
	} else {
		if( wheel.left.receive.deviceIndex != wheel.left.send.deviceIndex ) {
			printf("WLE  ERROR   : inconsistent device index receive %d send %d, abort\n", wheel.left.receive.deviceIndex, wheel.left.send.deviceIndex );
			exit( EXIT_FAILURE );
		}
		removeDeviceFromList( wheel.left.receive.deviceIndex );
	}

	if( wheel.rear.receive.started || wheel.rear.send.started ) {
		// at least one thread is still using the device
	} else {
		if( wheel.rear.receive.deviceIndex != wheel.rear.send.deviceIndex ) {
			printf("WRE  ERROR   : inconsistent device index receive %d send %d, abort\n", wheel.rear.receive.deviceIndex, wheel.rear.send.deviceIndex );
			exit( EXIT_FAILURE );
		}
		removeDeviceFromList( wheel.rear.receive.deviceIndex );
	}

	if( wheel.right.receive.started || wheel.right.send.started ) {
		// at least one thread is still using the device
	} else {
		if( wheel.right.receive.deviceIndex != wheel.right.send.deviceIndex ) {
			printf("WRI  ERROR   : inconsistent device index receive %d send %d, abort\n", wheel.right.receive.deviceIndex, wheel.right.send.deviceIndex );
			exit( EXIT_FAILURE );
		}
		removeDeviceFromList( wheel.right.receive.deviceIndex );
	}
}

void ttyUsb::showDeviceList( ) {
	// Note: the deviceList is only used by the ttyUsb::process
	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		if( deviceList[ii].function == FUNCTION__AA_DO_NOT_USE_ZERO || deviceList[ii].function == FUNCTION_ZZ_DO_NOT_USE_LAST ) {
			printf("SER  ERROR   : illegal function %d, abort\n", deviceList[ii].function );
			exit( EXIT_FAILURE );
		}

		printf( "___  INFO    : found device /dev/ttyUSB%d, used %d, function ", deviceList[ii].index, deviceList[ii].used );
		if( deviceList[ii].function == FUNCTION_BALL_LEFT ) { printf( "ball left\n" ); }
		else if( deviceList[ii].function == FUNCTION_BALL_RIGHT ) { printf( "ball right\n" ); }
		else if( deviceList[ii].function == FUNCTION_COMPASS ) { printf( "compass\n" ); }
		else if( deviceList[ii].function == FUNCTION_UNINITIALIZED ) { printf( "uninitialized\n" ); }
		else if( deviceList[ii].function == FUNCTION_UNKNOWN ) { printf( "unknown\n" ); }
		else if( deviceList[ii].function == FUNCTION_WHEEL_LEFT ) { printf( "wheel left\n" ); }
		else if( deviceList[ii].function == FUNCTION_WHEEL_REAR ) { printf( "wheel rear\n" ); }
		else if( deviceList[ii].function == FUNCTION_WHEEL_RIGHT ) { printf( "wheel right\n" ); }
	}
}

void ttyUsb::process( ) {

	bool allOkay = false;

	if( allBoardsAvailable( ) ) {
		// printf( "USB  INFO    : all boards available\n" );

		if( allFunctionsOperational( ) ) {
			// printf( "     INFO    : all functions operational\n" );
			allOkay = true;
		} else {
			printf( "___  WARNING : #### function list incomplete, check boards and try to restart function(s) ####\n" );
			// check if a function does not have a device anymore, if so remove from list
			unRegisterNonWorkingDevices( );
			allOkay = false;
		}
	} else {
		unRegisterNonWorkingDevices( );
		printf( "___  WARNING : #### board list incomplete, search for missing board(s) ####\n" );
	}

	if( ! allOkay ) {
		updateDeviceList( ); // complete list of devices
		identifyBoards( ); // determine the board for each device
		showDeviceList( ); // show all devices (and function if already used)
		registerNewFunctions( ); // for new boards start the related threads
	}
	// reduce the verbose (scanning) when a board is missing or function inactive
	usleep( 1000000 );
}
