// Copyright 2016 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdio.h>

#include "ttyUsb.hpp"

int main( ) {

	ttyUsb *usb = new ttyUsb( );

	for( size_t ii = 0; ii < 1000000; ii++ ) {
		usb->process( );
	}

	return 0;
}
