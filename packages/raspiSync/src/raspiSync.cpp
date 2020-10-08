 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// This application is able to synchronize the 4 raspi camera's
// The camera's generate frames just above 40 fps (25ms).
// Camera 0 is used as reference
// When the raspi CPU receives the camera data it sends a short UDP packet to the CPU box.
// These packets get time stamped when arrived at the CPU box.
// Because there is quite some jitter on the network interface between the raspi and CPU box,
// there is jitter on the time stamps.
// To reduce the time jitter on the CPU box, 32 time stamps are averaged.
// The average time stamp of each camera is projected in a 25ms time window.
// Because the camera's run at a slightly higher frequency the average time stamp will
// move in the 25ms time window
// The average time of Camera 1 to 3 are then compared with the average time stamp of camera 0.
// When a camera average time stamp is too early, the camera will be delayed by adding pixels
// and when a camera average time stamp is to late, the camera will be speedup by removing some pixels
// The non default amount of pixels will only occur for a short time, otherwise a lot of overshoot
// will occur.
// Also the some threshold is added before starting the adjustment.
// The camera rate is generated from a stable oscillator on the raspi camera, so when the camera's
// are more or less in sync, minimal adjustment is required.

#include <cerrno>
#include <fcntl.h>    /* For O_RDWR */
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h> // gettimeofday
#include <unistd.h> // getopt

#include "raspiSync.hpp"

using namespace std;
using namespace cv;

raspiSync::raspiSync() {

	raspiCtrl = new raspiControl();
	raspiCtrl->multiCastSetup();

	clock_gettime(CLOCK_MONOTONIC, &nextWakeTime);

	struct timeval tv;
	gettimeofday(&tv, NULL);
	nextWakeTime.tv_nsec = 1000000000 - 401442000 - 59000; // 00000000; // 0.5 second
	nextWakeTime.tv_sec++;

	camGrabRecv = new camGrabReceive(raspiCtrl);

	camGrabRecvThread = thread(&camGrabReceive::receive, camGrabRecv); // start thread that collects the sync pulses from the 4 camera's

	timeFrame = Mat::zeros(200, 500, CV_8UC3);
	timeFrame = Scalar(255, 255, 255);

	markerColor[0] = Scalar(255, 0, 0); // blue
	markerColor[1] = Scalar(0, 0, 255); // red
	markerColor[2] = Scalar(51, 153, 0); // green
	markerColor[3] = Scalar(0, 153, 204); // yellow/brown
	cam0Color = Scalar(255, 0, 0);
	cam1Color = Scalar(0, 0, 255);
	cam2Color = Scalar(0, 255, 0);
	cam3Color = Scalar(0, 225, 225);
	referenceColor = Scalar(125, 125, 125);

	for (size_t ii = 0; ii < 4; ii++) {
		pixelsOffset[ii] = 16; // default offset value is 16
		pixelsOffsetPrev[ii] = 0; // force update at startup
		pixelsOffsetIntegrator[ii] = 0;
	}
}

raspiSync::~raspiSync() {
	raspiCtrl->multiCastClose();
}

bool raspiSync::update() {

	camTimeSt camTime = camGrabRecv->getCamTime();

	// Add the time you want to sleep
	nextWakeTime.tv_nsec += 500000000; // 0.5 second

	// Normalize the time to account for the second boundary
	if (nextWakeTime.tv_nsec >= 1000000000) {
		nextWakeTime.tv_nsec -= 1000000000;
		nextWakeTime.tv_sec++;
	}
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextWakeTime, NULL);

	struct timeval tv;
	gettimeofday(&tv, NULL);
	//	uint64_t myTime = (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;
	// printf("time %8.1f ms delta %10lu %10lu", (myTime % 60000000) / 1000.0, tv.tv_usec, nextWakeTime.tv_nsec); // wrap every 60 seconds, in 0.1ms resolution

	timeFrame = Scalar(255, 255, 255);
	int textVal = -20;
	// draw the vertical line markers and time offset in ms
	for (int ii = 50; ii < 500; ii += 50) {
		if (ii == 250) {
			// use a thicker marker for the vertical center line (0 ms)
			line(timeFrame, Point(ii, 0), Point(ii, 199), referenceColor, 2);
		} else {
			line(timeFrame, Point(ii, 0), Point(ii, 199), referenceColor, 1);
		}
		// print time text (in ms) above the time time plot
		char buf[256];
		sprintf(buf, "%d", textVal);
		int location = 0;
		if (textVal <= -10) {
			location = ii - 21;
		} else if (textVal < 0) {
			location = ii - 17;
		} else if (textVal < 10) {
			location = ii - 5;
		} else {
			location = ii - 7;
		}
		putText(timeFrame, buf, Point(location, 20), 1, 1, Scalar(0, 0, 0), 1);
		textVal += 5;
	}

	// convert time to polar because the time is periodic in 25ms
	// this creates a robust way to average the time shifted time stamps of each camera
	double camAngleAvg[4], camAngleNeg[4], camAnglePos[4];

	for (size_t cam = 0; cam < 4; cam++) {
		double xSum = 0.0;
		double ySum = 0.0;
		double camAngleList[32];
		for (size_t ii = 0; ii < 32; ii++) {
			// the times in the camera list rotate through the buffer
			uint64_t offset = camTime.cam[cam][ii] % 25000;
			double x = cos(2.0 * CV_PI * offset / 25000.0);
			double y = sin(2.0 * CV_PI * offset / 25000.0);
			camAngleList[ii] = atan2(y, x);
			xSum += x;
			ySum += y;

		}
		camAngleAvg[cam] = atan2(ySum, xSum); // -Pi to Pi

		// find the minimal and maximal angle
		camAngleNeg[cam] = 2.0 * CV_PI;
		camAnglePos[cam] = -2.0 * CV_PI;
		for (size_t ii = 0; ii < 32; ii++) {
			double deltaAngle = camAngleAvg[cam] - camAngleList[ii];
			if (deltaAngle > CV_PI) {
				deltaAngle -= 2.0 * CV_PI;
			} else if (deltaAngle < -CV_PI) {
				deltaAngle += 2.0 * CV_PI;
			}
			if (deltaAngle > CV_PI || deltaAngle < -CV_PI) {
				printf("ERROR  : cam %zu delta angle of %6.1f deg out of range\n", cam,
						360.0 * deltaAngle / (2.0 * CV_PI));
			}
			if (deltaAngle < camAngleNeg[cam]) {
				camAngleNeg[cam] = deltaAngle;
			}
			if (deltaAngle > camAnglePos[cam]) {
				camAnglePos[cam] = deltaAngle;
			}
		}

		if (camAngleNeg[cam] < -(CV_PI / 2.0)) {
			printf("WARNING : cam %zu minimal out of range %6.1f deg %5.1f ms\n", cam,
					360.0 * camAngleNeg[cam] / (2.0 * CV_PI), 25.0 * camAngleNeg[cam] / (2.0 * CV_PI));
		}
		if (camAnglePos[cam] > (CV_PI / 2.0)) {
			printf("WARNING : cam %zu maximal out of range %6.1f deg %5.1f ms\n", cam,
					360.0 * camAnglePos[cam] / (2.0 * CV_PI), 25.0 * camAnglePos[cam] / (2.0 * CV_PI));
		}
	}

	double cam0DistanceAngle[4];
	for (size_t cam = 0; cam < 4; cam++) {
		// determine distance to cam0
		cam0DistanceAngle[cam] = camAngleAvg[cam] - camAngleAvg[0];
		if (cam0DistanceAngle[cam] > CV_PI) {
			cam0DistanceAngle[cam] -= 2.0 * CV_PI;
		} else if (cam0DistanceAngle[cam] < -CV_PI) {
			cam0DistanceAngle[cam] += 2.0 * CV_PI;
		}
		if (cam0DistanceAngle[cam] > CV_PI || cam0DistanceAngle[cam] < -CV_PI) {
			printf("ERROR  : cam %zu delta angle of %6.1f deg to camera 0 out of range\n", cam,
					360.0 * cam0DistanceAngle[cam] / (2.0 * CV_PI));
		}

		// increase or decrease the integrator in case the camera time differs to much with camera 0
		if (cam0DistanceAngle[cam] > (2 * CV_PI * 5) / 360.0) { // 5 degrees
			pixelsOffsetIntegrator[cam]++;
		}
		if (cam0DistanceAngle[cam] < -(2 * CV_PI * 5) / 360.0) { // -5 degrees
			pixelsOffsetIntegrator[cam]--;
		}

	}

	// camera 0 angle (in 360 degrees), maximal negative and positive angle (jitter)
	printf("%6.1f %6.1f %6.1f", 360.0 * camAngleAvg[0] / (2.0 * CV_PI), 360.0 * camAngleNeg[0] / (2.0 * CV_PI),
			360.0 * camAnglePos[0] / (2.0 * CV_PI));

	// same for camera 1 to 4, but these also have the angle difference (time) to camera 0 and the integrator value
	// which is used to provide some threshold to the adjustment of the camera
	for (size_t cam = 1; cam < 4; cam++) {
		printf(" | %6.1f %6.1f %6.1f %6.1f %3d", 360.0 * camAngleAvg[cam] / (2.0 * CV_PI),
				360.0 * camAngleNeg[cam] / (2.0 * CV_PI), 360.0 * camAnglePos[cam] / (2.0 * CV_PI),
				360.0 * cam0DistanceAngle[cam] / (2.0 * CV_PI), pixelsOffsetIntegrator[cam]);

	}

	// fill the time plot / diagram
	for (size_t cam = 0; cam < 4; cam++) {
		int zeroOffset = 25000;

		// draw the markers of the 4 camera's in the diagram
		int averageTime = (int) (25000 * camAngleAvg[cam] / (2.0 * CV_PI));
		int marker = zeroOffset + averageTime;
		line(timeFrame, Point(marker / 100, cam * 50 + 0), Point(marker / 100, cam * 50 + 50), markerColor[cam], 2);

		// add the jitter range to the markers
		int negTime = (int) (25000 * camAngleNeg[cam] / (2.0 * CV_PI));
		int posTime = (int) (25000 * camAnglePos[cam] / (2.0 * CV_PI));
		int leftMarker = zeroOffset + averageTime + negTime;
		int rightMarker = zeroOffset + averageTime + posTime;
		line(timeFrame, Point(leftMarker / 100, cam * 50 + 25), Point(rightMarker / 100, cam * 50 + 25),
				markerColor[cam], 1);
	}
	printf("\n");

	// use the angle differences (integrator) to decide if a camera needs to be adjusted
	bool sendPacket = false;
	for (size_t cam = 1; cam < 4; cam++) {
		if (pixelsOffsetIntegrator[cam] > 10) {
			pixelsOffset[cam] = 15; // 15 is also going pretty fast
			pixelsOffsetIntegrator[cam] = 0; // reset integrator
		} else if (pixelsOffsetIntegrator[cam] < -10) {
			pixelsOffset[cam] = 24; // 23 same behavior as 16, but 24 is going pretty fast
			pixelsOffsetIntegrator[cam] = 0;
		} else {
			// use the adjusted value for only on loop (500ms) and then set back to default
			pixelsOffset[cam] = 16; // set back to default value
		}
		if (pixelsOffset[cam] != pixelsOffsetPrev[cam]) {
			// only send the update packet to cameras when at least one camera has an updated value
			sendPacket = true;
			pixelsOffsetPrev[cam] = pixelsOffset[cam];
		}
	}
	if (sendPacket) {
		raspiCtrl->updateSystemControlPixelsOffset(pixelsOffset);
	}

	imshow("raspiSync", timeFrame);
	int key = waitKey(20);
	switch (key) {
	case '1':
		exit(0);
	case 27: // escape
		exit(0);
	case 'q':
		exit(0);
	default:
		break;
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

	raspiSync *sync = new raspiSync();

	printf("INFO      : wait for data from the camera's\n");

	while (sync->update()) {
	}

	printf("INFO      : all done\n");

	return 0;
}
