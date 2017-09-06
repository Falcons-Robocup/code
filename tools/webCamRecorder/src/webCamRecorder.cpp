// Copyright 2016 Andre Pool, April 2016
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

// Simple application to record data from the webcam

#include <iostream>
#include <iomanip>
#include <pwd.h> // for getpwuid()
#include <signal.h>
#include <stdio.h>
#include <unistd.h> // for getuid()

#include <linux/videodev2.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cameraControl.hpp"

using namespace cv;
using namespace std;

#define WIDTH 1280
#define HEIGHT 720


bool shutDown = false; // global variable used to stop the main loop when cntr-c signal

// interrupter signal received e.g. ctrl-c
void sigIntHandler(int s) {
	printf("received interrupt signal, probably ctrl-c, do a clean shutdown\n");
	shutDown = true;
}

int main(int argc, char *argv[]) {
	// redirect cntr-c signal to do a clean shutdown
	signal(SIGINT, sigIntHandler);

	char configFile[256];
	sprintf( configFile, "%s/falcons/code/packages/vision/vision.yaml", getpwuid(getuid())->pw_dir );
	FileStorage fs(configFile, FileStorage::READ);

	cameraSt camera; // containing camera configuration used by camCtrl

	// gather (default) settings for usb camera
	FileNode cameraConf = fs["camera"];
	camera.backlightCompensation = cameraConf["backlightCompensation"];
	camera.brightness = cameraConf["brightness"];
	camera.contrast = cameraConf["contrast"];
	camera.exposureAbsolute = cameraConf["exposureAbsolute"];
	camera.exposureAuto = cameraConf["exposureAuto"];
	camera.exposureAutoPriority = cameraConf["exposureAutoPriority"];
	camera.focusAbsolute = 95; // robot1 TODO make this arg
	//camera.focusAbsolute = 75; // robot2 TODO make this arg
	//camera.focusAbsolute = 85; // robot4 TODO make this arg
	//camera.focusAbsolute = 100; // robot5 TODO make this arg
	//camera.focusAbsolute = 80; // robot6 TODO make this arg
	camera.focusAuto = cameraConf["focusAuto"];
	camera.gain = cameraConf["gain"];
	camera.led1Frequency = cameraConf["led1Frequency"];
	camera.led1Mode = cameraConf["led1Mode"];
	camera.panAbsolute = cameraConf["panAbsolute"];
	camera.powerLineFrequency = cameraConf["powerLineFrequency"];
	camera.saturation = cameraConf["saturation"];
	camera.sharpness = cameraConf["sharpness"];
	camera.tiltAbsolute = cameraConf["tiltAbsolute"];
	camera.whiteBalanceAuto = cameraConf["whiteBalanceAuto"];
	camera.whiteBalanceTemperature = cameraConf["whiteBalanceTemperature"];
	camera.zoomAbsolute = cameraConf["zoomAbsolute"];

	fs.release();

	string cameraWindow;
	cameraWindow = "camera window";
	namedWindow(cameraWindow, CV_WINDOW_NORMAL);
	createTrackbar( "backlightComp", cameraWindow, &camera.backlightCompensation, 1);
	createTrackbar( "brightness", cameraWindow, &camera.brightness, 255);
	createTrackbar( "contrast", cameraWindow, &camera.contrast, 255);
	// createTrackbar( "exposureAuto", cameraWindow, &camera.exposureAuto, 1);
	createTrackbar( "exposure", cameraWindow, &camera.exposureAbsolute, 2047);
	// createTrackbar( "exposurePrio", cameraWindow, &camera.exposureAutoPriority, 1);
	createTrackbar( "focusAbs", cameraWindow, &camera.focusAbsolute, 255);
	// createTrackbar( "focusAuto", cameraWindow, &camera.focusAuto, 1);
	createTrackbar( "gain", cameraWindow, &camera.gain, 255);
	// library on robot does not support led createTrackbar( "led1Frequency", cameraWindow, &camera.led1Frequency, 255);
	// library on robot does not support led createTrackbar( "led1Mode", cameraWindow, &camera.led1Mode, 3);
	// createTrackbar( "panAbsolute", cameraWindow, &camera.panAbsolute, 200);
	// createTrackbar( "powerLineFreq", cameraWindow, &camera.powerLineFrequency, 2);
	createTrackbar( "saturation", cameraWindow, &camera.saturation, 255);
	createTrackbar( "sharpness", cameraWindow, &camera.sharpness, 255);
	// createTrackbar( "tiltAbsolute", cameraWindow, &camera.tiltAbsolute, 200);
	createTrackbar( "whiteBalAuto", cameraWindow, &camera.whiteBalanceAuto, 1);
	createTrackbar( "whiteBalance", cameraWindow, &camera.whiteBalanceTemperature, 6500);
	// createTrackbar( "zoom", cameraWindow, &camera.zoomAbsolute, 5);

	cameraControl camCtrl; // control camera e.g. brightness, hue, exposure

	// store raw vidoe data to file for later use
    VideoWriter outputVideo;
	int fourcc = CV_FOURCC('M','J','P','G'); // always use this fourcc
    const string destination  = "/home/robocup/data/latest_and_greatest.avi";
    cout << "record to file " << destination << endl;
    // APOX: todo checkout why only 10fps
    outputVideo.open(destination, fourcc, 20, (Size(1280,720)), true);
    if (!outputVideo.isOpened()) {
    	cout  << "Could not open the output video for write: " << destination << endl;
		return -1;
    }


	Mat frame;
    char key = 0;
    VideoCapture capture(0);
	capture.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	while( ! shutDown  ) { // stop when cntr-c
		capture >> frame;
		if( frame.empty( ) ) {
			printf( "ERROR   : frame is empty\n" );
		}

		camCtrl.update( camera, true );

		imshow("press space to capture frame, q or esc to quit", frame);
		key = (char) waitKey(5); //delay N millis, usually long enough to display and capture input
		outputVideo << frame;

		switch( key ) {
			case 'q':
			case 'Q':
			case 27: //escape key
				return 0;
			default:
				break;
		}
	}
	return 0;
}

