// Copyright 2016 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef TTY_USB_THREAD_HPP
#define TTY_USB_THREAD_HPP

#include <pthread.h>

class ttyUsbThread {
private:
	pthread_t threadId;
	size_t counter;
	bool started;
	char name[64];
	char prefix[8];
	
public:
	ttyUsbThread( );
};

#endif

