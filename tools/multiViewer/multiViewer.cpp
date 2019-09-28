// Copyright 2017, 2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "dewarp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "multiViewer.hpp"
#include "dewarp.hpp"

using namespace cv;
using namespace std;

grabView::grabView(deWarper *D) {
	rx = new cameraReceive();
	this->D = D;
}

void grabView::drawLinePoints() {
	// show the received line points
	// less then half of the camera image is used for line points
	Mat linePointFrame = Mat::zeros(ROI_HEIGHT, ROI_WIDTH / 2, CV_8UC3); // y, x, depth
	for (size_t ii = 0; ii < linePointList.size(); ii++) {
		uint16_t x = linePointList[ii].x;
		uint16_t y = linePointList[ii].y;
		line(linePointFrame, Point(x, y), Point(x, y), Scalar(255, 255, 255), 1);
	}
	imshow("line pixels, q to quit", linePointFrame);
}

void grabView::drawCameraFrame() {
	Mat cameraBgr;
	cvtColor(cameraFrame, cameraBgr, COLOR_HSV2BGR); // convert to BGR
	Mat cameraResize;
	resize(cameraBgr, cameraResize, Size(), 2.0, 2.0, INTER_LINEAR); // double the size
	int cols = cameraResize.cols;
	int rows = cameraResize.rows;
// straight line
	line(cameraResize, Point(0, rows / 2), Point(cols, rows / 2), Scalar(255, 255, 255), 1);

	printf(".");
	fflush(stdout);
	imshow("camframe, q to quit", cameraResize);
}

void grabView::getReceiveData() {
	while (rx->getWaitForNew()) {
		usleep(100);
	}
	if (rx->receivedNewLinePoints()) {
		linePointList = rx->getLinePoints();
		drawLinePoints();
		drawDeWarp();
	} else if (rx->receivedNewCameraFrame()) {
		cameraFrame = rx->getCameraFrame();
		drawCameraFrame();
	} else {
		// can be obstacles or ball points
		// printf("ERROR   : unknown camera receive response\n");
	}
}

void grabView::drawDeWarp() {
	// show the received dewarped line points
	// less then half of the camera image is used for line points
	Mat dewarpFrame = Mat::zeros(ROI_HEIGHT, ROI_WIDTH / 2, CV_8UC3); // y, x, depth
	for (size_t ii = 0; ii < linePointList.size(); ii++) {
		uint16_t xPixel = linePointList[ii].x;
		uint16_t yPixel = linePointList[ii].y;
		if ((xPixel < 370) && (yPixel < 616)) {
			int16_t xField = 0;
			int16_t yField = 0;
			D->transform(xPixel, yPixel, xField, yField);
			xField /= 10;
			yField /= 10;
			yField += 300;
			line(dewarpFrame, Point(xField, yField), Point(xField, yField), Scalar(255, 255, 255), 1);
		}
	}
	imshow("dewarp pixels, q to quit", dewarpFrame);
}

void grabView::update() {

	std::string markerConfig = "markers";

#ifdef NONO
	namedWindow(markerConfig, CV_WINDOW_NORMAL);
	char buf[8];
	size_t maxMarker = MARKER_MAX;
	if (maxMarker > 12) {
		maxMarker = 12;
	}
	for (size_t ii = 0; ii < maxMarker; ii++) {
		sprintf(buf, "%zu", ii);
		createTrackbar(buf, markerConfig, &markerTrackbar[ii], SEND_IMAGE_WIDTH * 2);
	}
#endif
	getReceiveData(); // blocking

	usleep(1);
}

void grabView::startReceiveThread() {
	receiveThread = thread(&cameraReceive::receive, rx);
}

int main(int argc, char **argv) {
	if (argc < 3) {
		printf("ERROR: missing files: image and config\n");
		return 1;
	}

	// load image
	cv::Mat src = cv::imread(argv[1], 1);

	// construct our dewarper
	deWarper D;

	// load and verify
	D.load(argv[2]);
	D.calculate();
	D.verify();

	/*
	 // prompt for manual pixel inspection
	 while (true)
	 {
	 int px, py;
	 int16_t fx, fy;
	 printf("\nenter pixel (x,y): ");
	 fflush(stdout);
	 scanf("%d %d", &px, &py);
	 D.transform(px, py, fx, fy);
	 printf("result field: (%d,%d)\n", fx, fy);
	 }
	 return 0;
	 */

	// create window
	cv::namedWindow(argv[0], CV_WINDOW_AUTOSIZE);

	// loop: reload config every second, re-interpolate, show result
	for (size_t ii = 0; ii < 2; ii++) {
		int c = cv::waitKey(1000);
		if ((char) c == 27)
			break;

		// TODO: make use of clicking: store coordinates?
		// when doing triangulation-based interpolation, we could calibrate using drag and drop

		// logging
		printf("recalculating...");
		fflush(stdout);
		struct timeval tv;
		gettimeofday(&tv, NULL);
		double t0 = tv.tv_sec + 1e-6 * tv.tv_usec;

		// reload and calculate
		D.load(argv[2]);
		D.calculate();

		// logging
		gettimeofday(&tv, NULL);
		double now = tv.tv_sec + 1e-6 * tv.tv_usec;
		printf(" done (%.1fMB in %.1fms)\n", D.cacheSize(), (now - t0) / 1e-3);
		fflush(stdout);

		// show the result
		cv::imshow(argv[0], D.transform(src));
	}

	grabView *view = new grabView(&D);

	view->startReceiveThread();

	bool busy = true;
	while (busy) {
		view->update();
		int key = waitKey(20);
// if( key >= 0 ) { printf( "key: %d\n", key );  }
		switch (key) {
		case 1048603: // escape
		case 1179729: // caps lock q
		case 1048689: // q
		case 27: // escape
		case 'q':
			printf("INFO      : exit now\n");
			busy = false;
			break;
		default:
			break;
		}
	}
	return 0;
}

