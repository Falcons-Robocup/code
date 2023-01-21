// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <thread>
#include <unistd.h> // getopt, usleep
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "grabber.hpp"

int main(int argc, char **argv) {

	grabber grab;
	cv::Mat grabImage = cv::Mat::zeros(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
	std::thread grabThread;
	bool guiEnabled = true;
	int index = 0; // select one of the available camera'ss
	int opt = 0;

	printf("INFO    grab images from a Basler Dart camera through the pylon interface\n");

	while( (opt = getopt(argc, argv, "ci:")) != -1 ) {
		switch( opt ) {
		case 'c':
			guiEnabled = false;
			break;
		case 'i':
			index = atoi(optarg);
			break;
		}
	}

	grab.attach(index);
	printf("INFO    serial of selected camera is %s\n", grab.getSerial().c_str());
	grab.configure();
	grab.start();

	// start thread that receives the images from the selected camera
	grabThread = std::thread(&grabber::update, &grab);

	while( true ) {
		grabImage = grab.getImage();
		if( guiEnabled ) {
			cv::imshow("pylon image", grabImage);
		}
		cv::waitKey(20); // required to display image
	}

	printf("INFO    main all done\n");
	return EXIT_SUCCESS;
}
