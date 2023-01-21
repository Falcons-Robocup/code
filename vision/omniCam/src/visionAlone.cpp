// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // getopt

#include "vision.hpp"

int main(int argc, char **argv) {
	int opt = 0;
	bool guiEnabled = true;
	int robotIdArg = 0;

	int skipFrame = 0;
	while( (opt = getopt(argc, argv, "ci:s:")) != -1 ) {
		switch( opt ) {
		case 'c':
			guiEnabled = false;
			printf("INFO      : disable the GUI\n");
			break;
		case 'i':
			robotIdArg = atoi(optarg);
			printf("INFO      : using command argument to set robot id to %d\n", robotIdArg);
			break;
		case 's': // skip
			skipFrame = atoi(optarg);
			printf("INFO      : skip the first %d frames of the video file\n", skipFrame);
			break;
		}
	}

	visionLibrary *vis = new visionLibrary(robotIdArg, guiEnabled, skipFrame);

	while( vis->update() ) {
	}

	return 0;
}
