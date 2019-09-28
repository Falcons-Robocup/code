// Copyright 2016-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <cerrno>
#include <fcntl.h>    /* For O_RDWR */
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h> // gettimeofday
#include <unistd.h> // getopt

#include "multiCamViewer.hpp"

using namespace std;
using namespace cv;

multiCamViewer::multiCamViewer() {

	size_t openCvVersion = CV_MAJOR_VERSION * 1000000 + CV_MINOR_VERSION * 1000 + CV_VERSION_REVISION;
	if (openCvVersion < 2004001) {
		printf("ERROR     : tested with openCV version 2.4.1 but now using older version %d.%d.%d\n", CV_MAJOR_VERSION,
		CV_MINOR_VERSION, CV_VERSION_REVISION);
		exit(EXIT_FAILURE);
	} else if (openCvVersion > 2004001) {
		printf("WARNING   : tested with openCV version 2.4.1 but now using newer version %d.%d.%d\n", CV_MAJOR_VERSION,
		CV_MINOR_VERSION, CV_VERSION_REVISION);
	} else {
		printf("INFO      : openCV version %d.%d.%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_VERSION_REVISION);

	}

	camSysRecv = new camSysReceive();
	camSysRecv->setImageGrabPath("."); // do not use "/dev/shm" which is used by multiCam

	// start thread that collects raspiSystem data from multiple camera's
	// this thread also saves the jpg files, with a time stamp, in the local local directory
	camSysRecvThread = thread(&camSysReceive::receive, camSysRecv);
}

multiCamViewer::~multiCamViewer() {
}

bool multiCamViewer::update() {
	Mat image;
	Point textLocation0 = Point(10, 20);
	// Point textLocation1 = Point(10, 40);
	Scalar color(0, 255, 255); // yellow
	Scalar color_err(0, 0, 255); // red
	char camText[256];
	stringstream strForm;

	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		camSysRecv->getCameraFrame(camIndex).copyTo(image);

		// Jan: this is the place to use the image
		// camIndex indicates which camera
		// cam0: front
		// cam1: left
		// cam2: rear
		// cam3: right

		// for debugging the camera synchronization it is better to look at the non rotated images
#ifdef NONO
		// rotate image depending on location on robot
		if( camIndex == 0 ) {
			// show front camera in camera viewer = rotate 90 degrees
			transpose(image, image);
			flip(image, image, 0);
		} else if( camIndex == 1 ) {
			// show left camera in camera viewer = rotate 180 degrees
			flip(image, image, -1);
		} else if( camIndex == 2 ) {
			// show rear camera in camera viewer = rotate 270 degrees
			transpose(image, image);
			flip(image, image, 1);
		} else {
			// show right camera in camera viewer = keep
		}
#endif

		camSystemSt cs = camSysRecv->getCamSystem(camIndex);
		uint16_t sysApplUptime = cs.sysApplUptime;
		hhMmSsSt sysUp = camSysRecv->secondsToHhMmSs(sysApplUptime);
		sprintf(camText, "cam %zu system  uptime %02u:%02u:%02u", camIndex, sysUp.hours, sysUp.minutes, sysUp.seconds);

		if (sysApplUptime < 10) {
			putText(image, camText, textLocation0, 1, 1, color_err, 1);
		} else {
			putText(image, camText, textLocation0, 1, 1, color, 1);
		}

		sprintf(camText, "cam %zu", camIndex);
		imshow(camText, image);
	}

	int key = waitKey(200);
	if (key >= 0) {
		key &= 255;
		printf("INFO      : key pressed (%d)\n", key);
	}
	if (key == 27 || key == 'q') { // escape or q
		return false;
	}

	return true;
}

int main(int argc, char** argv) {
	int opt = 0;
	while ((opt = getopt(argc, argv, "h")) != -1) {
		switch (opt) {
		case 'h':
			printf("INFO      : no arguments required\n");
			break;
		}
	}

	multiCamViewer *multCam = new multiCamViewer();

	printf("INFO      : wait for data from the camera's\n");
	while (multCam->update()) {
	}

	printf("INFO      : all done\n");

	return 0;
}

