// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // getopt

#include "multiCam.hpp"

int main(int argc, char** argv)
{
	int opt = 0;
	bool guiEnabled = true;
	int robotIdArg = 0;

	while( (opt = getopt(argc, argv, "ci:") ) != -1 ) {
		switch(opt) {
		case 'c':
			guiEnabled = false;
			printf("INFO      : disable the GUI\n");
			break;
		case 'i':
			robotIdArg = atoi(optarg);
			printf("INFO      : using command argument to set robot id to %d\n", robotIdArg);
			break;
		}
	}

	multiCamLibrary *multCam = new multiCamLibrary( robotIdArg, guiEnabled );

	while( multCam->update() ) { }

	return 0;
}
