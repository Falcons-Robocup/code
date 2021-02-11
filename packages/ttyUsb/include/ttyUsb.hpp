// Copyright 2016 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef TTY_USB_HPP
#define TTY_USB_HPP

#include "serial.hpp"

struct threadSt {
	size_t counter;
	size_t counterPrevious; // used to verify if thread is still active
	size_t freezeCounter; // used to verify if thread is still active
	size_t identifier; // identifier for the thread (e.g. ball handler left, compass, .. )
	int deviceIndex; // device index to /dev/ttyUSBx
	std::string  name; // human readable description of function
	std::string prefix; // prefix in print lines to be able to use grep to filter
	bool started;
	pthread_t threadId;
};

class ttyUsb {
private:


	struct functionSt {
		threadSt receive;
		threadSt send;
	};

	struct ballSt {
		functionSt left;
		functionSt right;
	};

	struct wheelSt {
		functionSt left;
		functionSt rear;
		functionSt right;
	};

	struct deviceSt {
		int index;
		bool used;
		int function;
		bool operator<( const deviceSt& val ) const {
			// sorting this struct is performed on index, lower index first
			return index < val.index;
		}
	};

	ballSt ball;
	threadSt communication;
	threadSt compass;
	std::vector<deviceSt> deviceList;
	wheelSt wheel;

	bool allBoardsAvailable( );
	bool allFunctionsOperational( );
	void identifyBoards( );
	void init( threadSt* context, size_t identifier, std::string prefix, std::string name );
	void initDynamicValues( threadSt* context, int deviceIndex );
	void registerNewFunctions( );
	void registerOneFunction( threadSt* context, int deviceIndex );
	void removeDeviceFromList( int index);
	void setDeviceUsed( int deviceIndex);
	void showDeviceList( );
	void threadCancel( threadSt *context );
	bool threadCheck( threadSt* context );
	static void* threadProcess( void *arg );
	void threadStart( threadSt *context, int deviceIndex );
	void threadTest( threadSt* context );
	void unRegisterNonWorkingDevices( );

public:
	ttyUsb( );
	void process( );
	void updateDeviceList( );
};

#endif
