// Copyright 2016 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdio.h>
#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <unistd.h>

#include "ttyUsbThread.hpp"

using namespace std;

ttyUsbThread::ttyUsbThread( ) {
	printf( "tty Usb Thread Constructor\n" );
	
	threadId = 0;
	counter = 0;
	started = false;
	sprintf( name, "not set name" );
	sprintf( prefix, "NOT" );
}

