// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

// Board is reset by toggling DTR signal of the serial port

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> // for dtr control


int main(int argc, char** argv) {
	int opt = 0;
	_Bool quiet = 0;
	char portName[256] = "/dev/ttyUSB0";
	while( (opt = getopt(argc, argv, "hP:q") ) != -1 ) {
		switch(opt) {
		case 'h':
			printf("options:\n");
			printf("  -P serial port e.g. /dev/ttyUSB0\n");
			printf("  -q quiet\n");
			printf("  -h this help\n");
			return 0;
			break;
		case 'P':
			strcpy(portName, optarg);
			break;
		case 'q':
			quiet = 1;
			break;
		}
	}

	if( ! quiet ) {	printf("using serial interface: %s\n", portName); }

	int port = open(portName, O_RDWR );

	// run the reset loop twice, because the DTR could already be active
	for( int ii = 0; ii < 2; ii++ )
	{
		// turn on DTR
		int iFlags = TIOCM_DTR;
		ioctl(port, TIOCMBIS, &iFlags);

		// turn off DTR
		iFlags = TIOCM_DTR;
		ioctl(port, TIOCMBIC, &iFlags);
	}

	close(port);
	return 0;
}
