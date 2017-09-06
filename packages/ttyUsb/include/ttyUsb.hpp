 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
