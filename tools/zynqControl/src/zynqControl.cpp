// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "zynqControl.hpp"
#include "zynqGrabConfig.hpp"

using namespace cv;
using namespace std;

zynqControl::zynqControl() {
	zynqGrabConfig *config = new zynqGrabConfig();
	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		sock[camIndex] = new tcp_socket(camIndex, config);
	}
	lineValMinPrev = -1;
	lineSatMaxPrev = -1;
	lineTransferPixelsMaxPrev = -1;
	lineFloorWindowSizePrev = -1;
	lineFloorPixelsMinPrev = -1;
	lineWindowSizePrev = -1;
	linePixelsMinPrev = -1;
	ballValMinPrev = -1;
	ballSatMinPrev = -1;
	ballHueMinPrev = -1;
	ballHueMaxPrev = -1;
	ballWindowSizePrev = -1;
	ballPixelsMinPrev = -1;
	ballFalsePixelsMaxPrev = -1;
	floorValMinPrev = -1;
	floorSatMinPrev = -1;
	floorHueMinPrev = -1;
	floorHueMaxPrev = -1;
	obstacleValMaxPrev = -1;
	obstacleSatMaxPrev = -1;
	obstacleFloorWindowSizePrev = -1;
	obstacleLineWindowSizePrev = -1;
	obstacleBallWindowSizePrev = -1;
	obstacleWindowSizePrev = -1;
	obstacleTransferPixelsMaxPrev = -1;
	obstacleFloorPixelsMinPrev = -1;
	obstacleLinePixelsMinPrev = -1;
	obstacleBallPixelsMinPrev = -1;
	obstaclePixelsMinPrev = -1;
	redPrev = -1;
	greenPrev = -1;
	bluePrev = -1;
	testPatternPrev = -1;

	cameraReset = 0;
	EXCK_FREQ_PREV = -1;
	VTPXCK_DIV_PREV = -1;
	VTSYCK_DIV_PREV = -1;
	PREPLLCK_VT_DIV_PREV = -1;
	PREPLLCK_OP_DIV_PREV = -1;
	PLL_VT_MPY_PREV = -1;
	OPPXCK_DIV_PREV = -1;
	OPSYCK_DIV_PREV = -1;
	PLL_OP_MPY_PREV = -1;
}

void zynqControl::send_message(uint8_t id, uint32_t value) {
	printf("INFO    : send id %2d of %3zu bytes has been send\n", id, sizeof(uint32_t));
	for (size_t ii = 0; ii < 4; ii++) {
		if (!sock[ii]->send(id, value)) {
			printf("ERROR   : cannot set send id %2d of %3zu bytes\n", id, sizeof(uint32_t));
		}
	}
}

void zynqControl::send_message(uint8_t id) {
	printf("INFO    : send id %2d of   0 bytes has been send\n", id);
	for (size_t ii = 0; ii < 4; ii++) {
		if (!sock[ii]->send(id)) {
			printf("ERROR   : cannot set send id %2d of %3zu bytes\n", id, sizeof(uint32_t));
		}
	}
}

void zynqControl::update(bool interfaceLocal) {
	if (interfaceLocal) {
		sock[0]->connect("127.0.0.70", 44444);
		sock[1]->connect("127.0.0.71", 44444);
		sock[2]->connect("127.0.0.72", 44444);
		sock[3]->connect("127.0.0.73", 44444);
	} else {
		sock[0]->connect("10.0.0.70", 44444);
		sock[1]->connect("10.0.0.71", 44444);
		sock[2]->connect("10.0.0.72", 44444);
		sock[3]->connect("10.0.0.73", 44444);
	}

	int lineValMin = 169;
	int lineSatMax = 40;
	int lineTransferPixelsMax = 32; // needs to large for close by white lines
	int lineFloorWindowSize = 15;
	int lineFloorPixelsMin = 10;
	int lineWindowSize = 31; // need to be larger then 15, because there might be non white pixels on the line
	int linePixelsMin = 0; // small to be able to catch far away pixels

	int ballValMin = 125;
	int ballSatMin = 50;
	int ballHueMin = 15; // yellow is 60 degrees => 60/2=30
	int ballHueMax = 35;
	int ballWindowSize = 4;
	int ballPixelsMin = 2;
	int ballFalsePixelsMax = 4;

	int floorValMin = 55;
	int floorSatMin = 20; // should be pretty low, to block light spots which might be recognized as lines
	int floorHueMin = 55; // floor is 200 degrees => 200/2=100
	int floorHueMax = 95;

	int obstacleValMax = 70;
	int obstacleSatMax = 60;
	int obstacleFloorWindowSize = 20;
	int obstacleFloorPixelsMin = 10;
	int obstacleLineWindowSize = 10;
	int obstacleLinePixelsMin = 5;
	int obstacleBallWindowSize = 10;
	int obstacleBallPixelsMin = 5;
	int obstacleTransferPixelsMax = 20;
	int obstacleWindowSize = 30;
	int obstaclePixelsMin = 20;

	int red = 128; // not used
	int green = 97;
	int blue = 176;
	int testPattern = 0;
	int analogGain = 190;
	int shutter = 209; // use this one to compensate for the 50Hz mains frequency;

	lineConfig = "lines";
	namedWindow(lineConfig, CV_WINDOW_NORMAL);
	createTrackbar("line val min", lineConfig, &lineValMin, 255);
	createTrackbar("line sat max", lineConfig, &lineSatMax, 255);
	createTrackbar("line trans max", lineConfig, &lineTransferPixelsMax, 32);
	createTrackbar("line floor window", lineConfig, &lineFloorWindowSize, 32);
	createTrackbar("line pix floor max", lineConfig, &lineFloorPixelsMin, 32);
	createTrackbar("line window max", lineConfig, &lineWindowSize, 32);
	createTrackbar("line pixels min", lineConfig, &linePixelsMin, 32);

	ballConfig = "balls";
	namedWindow(ballConfig, CV_WINDOW_NORMAL);
	createTrackbar("ball val min", ballConfig, &ballValMin, 255);
	createTrackbar("ball sat min", ballConfig, &ballSatMin, 255);
	createTrackbar("ball hue min", ballConfig, &ballHueMin, 255);
	createTrackbar("ball hue max", ballConfig, &ballHueMax, 255);
	createTrackbar("ball window size", ballConfig, &ballWindowSize, 255);
	createTrackbar("ball pixel min", ballConfig, &ballPixelsMin, 255);
	createTrackbar("ball false pix", ballConfig, &ballFalsePixelsMax, 255);

	floorConfig = "floor";
	namedWindow(floorConfig, CV_WINDOW_NORMAL);
	createTrackbar("floor val min", floorConfig, &floorValMin, 255);
	createTrackbar("floor sat min", floorConfig, &floorSatMin, 255);
	createTrackbar("floor hue min", floorConfig, &floorHueMin, 255);
	createTrackbar("floor hue max", floorConfig, &floorHueMax, 255);

	obstacleConfig = "obstacle";
	namedWindow(obstacleConfig, CV_WINDOW_NORMAL);
	createTrackbar("obst val max", obstacleConfig, &obstacleValMax, 255);
	createTrackbar("obst sat max", obstacleConfig, &obstacleSatMax, 255);
	createTrackbar("obst floor win", obstacleConfig, &obstacleFloorWindowSize, 32);
	createTrackbar("obst floor min", obstacleConfig, &obstacleFloorPixelsMin, 32);
	createTrackbar("obst line win", obstacleConfig, &obstacleLineWindowSize, 32);
	createTrackbar("obst line min", obstacleConfig, &obstacleLinePixelsMin, 32);
	createTrackbar("obst ball win", obstacleConfig, &obstacleBallWindowSize, 32);
	createTrackbar("obst ball min", obstacleConfig, &obstacleBallPixelsMin, 32);
	createTrackbar("obst trans max", obstacleConfig, &obstacleTransferPixelsMax, 32);
	createTrackbar("obst window", obstacleConfig, &obstacleWindowSize, 100);
	createTrackbar("obst minimal", obstacleConfig, &obstaclePixelsMin, 100);

	cameraConfig = "camera";
	namedWindow(cameraConfig, CV_WINDOW_NORMAL);
	// createTrackbar("red", cameraConfig, &red, 255); // fixed in zynqGrab to save cycles
	createTrackbar("green", cameraConfig, &green, 255);
	createTrackbar("blue", cameraConfig, &blue, 255);
	createTrackbar("test pattern", cameraConfig, &testPattern, 9);
	createTrackbar("ana gain", cameraConfig, &analogGain, 255);
	createTrackbar("shutter", cameraConfig, &shutter, 1024);

	// number of effective pixels 3296 x 2480
	// number of active pixels 3280 x 2464
	// camera configured for x4 binning : 3280/4 = 820, 2464/4 = 616
	int lines = 2728; // 0x0160-0x0161 FRM_LENGTH_A : how does this relate to 2480/2454
	int pixels = 3448 + 16; // 0x0162-0x0163 LINE_LENGTH_A : 3280 (active pixels) + 168 (blanking), added 16 to reduce sync errors, TODO: investigate why this occurs
	int xStart = 0; // 0x0164-0x0165 X_ADD_STA_A : X-address on the top left corner of the visible pixel data, default 0
	int xEnd = 3279; // 0x0166-0x0167 X_ADD_END_A : X-address on the bottom right corner of the visible pixel data, default 3279
	int xSize = 1280 + 16; // 0x016c-0x016d : X_OUTPUT_SIZE_A : width of output image 1280 + 16 ?? size required by FPGA CS-2 decoder ?
	int yStart = 0; // 0x0168-0x0169 Y_ADD_STA_A : Y-address on the top left left corner of the visible pixel data, default 0
	int yEnd = 2463; // 0x016a-0x016b Y_ADD_END_A : Y-address on the bottom right corner of the visible pixel data : default 2463
	int ySize = 720 + 16; // 0x016e-0x016f Y_OUTPUT_SIZE_A : height of output image, 720 + 16 ?? size required by FPGA CS-2 decoder ?
	sizeConfig = "sizes";
	namedWindow(sizeConfig, CV_WINDOW_NORMAL);
	createTrackbar("lines", sizeConfig, &lines, 3000);
	createTrackbar("pixels", sizeConfig, &pixels, 4000);
	createTrackbar("x start", sizeConfig, &xStart, 1000);
	createTrackbar("x end", sizeConfig, &xEnd, 4000);
	createTrackbar("x size", sizeConfig, &xSize, 4000);
	createTrackbar("y start", sizeConfig, &yStart, 2000);
	createTrackbar("y end", sizeConfig, &yEnd, 3000);
	createTrackbar("y size", sizeConfig, &ySize, 2000);

	int EXCK_FREQ = 0x1800; // 0x012a-0x012b : 24.00MHz = 24MHz + 0.00MHz = 0x18 + 0x00 = 0x1800 = 6144
	int VTPXCK_DIV = 5; // 0x0301
	int VTSYCK_DIV = 1; // 0x0303
	int PREPLLCK_VT_DIV = 2; // 0x0304
	int PREPLLCK_OP_DIV = 2; // 0x0305
	int PLL_VT_MPY = 0x0017; // 0x0306-0x0307 :  PLL video timing system multiplier value, works in range 0x10 to 0x19
	int OPPXCK_DIV = 0x0a; // 0x0309
	int OPSYCK_DIV = 0x01; // 0x030b
	int PLL_OP_MPY = 0x002e; // 0x030c-0x030d : PLL output system multiplier value, works from 0x002a to 0x005c
	string clockConfig = "clock";
	namedWindow(clockConfig, CV_WINDOW_NORMAL);
	createTrackbar("reset", clockConfig, &cameraReset, 1); // 0x012a, 0x012b
	createTrackbar("EXCK_FREQ", clockConfig, &EXCK_FREQ, 0x8000); // 0x012a, 0x012b
	createTrackbar("VTPXCK_DIV", clockConfig, &VTPXCK_DIV, 31); // 0x0301
	createTrackbar("VTSYCK_DIV", clockConfig, &VTSYCK_DIV, 3); // 0x03O3
	createTrackbar("PREPLLCK_VT_DIV", clockConfig, &PREPLLCK_VT_DIV, 255); // 0x0304
	createTrackbar("PREPLLCK_OP_DIV", clockConfig, &PREPLLCK_OP_DIV, 255); // 0x0305
	createTrackbar("PLL_VT_MPY", clockConfig, &PLL_VT_MPY, 2047); // 0x0306, 0x0307
	createTrackbar("OPPXCK_DIV", clockConfig, &OPPXCK_DIV, 31); // 0x0309
	createTrackbar("OPSYCK_DIV", clockConfig, &OPSYCK_DIV, 3); // 0x030b
	createTrackbar("PLL_OP_MPY", clockConfig, &PLL_OP_MPY, 2047); // 0x030c, 0x030d

	while (true) {
		waitKey(20);
		if (lineValMin != lineValMinPrev) {
			send_message(10, lineValMin);
			lineValMinPrev = lineValMin;
		}
		if (lineSatMax != lineSatMaxPrev) {
			send_message(11, lineSatMax);
			lineSatMaxPrev = lineSatMax;
		}
		if (ballValMin != ballValMinPrev) {
			send_message(12, ballValMin);
			ballValMinPrev = ballValMin;
		}
		if (ballSatMin != ballSatMinPrev) {
			send_message(13, ballSatMin);
			ballSatMinPrev = ballSatMin;
		}
		if (ballHueMin != ballHueMinPrev) {
			send_message(14, ballHueMin);
			ballHueMinPrev = ballHueMin;
		}
		if (ballHueMax != ballHueMaxPrev) {
			send_message(15, ballHueMax);
			ballHueMaxPrev = ballHueMax;
		}
		if (obstacleValMax != obstacleValMaxPrev) {
			send_message(16, obstacleValMax);
			obstacleValMaxPrev = obstacleValMax;
		}
		if (obstacleSatMax != obstacleSatMaxPrev) {
			send_message(17, obstacleSatMax);
			obstacleSatMaxPrev = obstacleSatMax;
		}
		if (red != redPrev) {
			send_message(18, red);
			redPrev = red;
		}
		if (green != greenPrev) {
			send_message(19, green);
			greenPrev = green;
		}
		if (blue != bluePrev) {
			send_message(20, blue);
			bluePrev = blue;
		}
		if (testPattern != testPatternPrev) {
			send_message(21, testPattern);
			testPatternPrev = testPattern;
		}
		if (analogGain != analogGainPrev) {
			send_message(22, analogGain);
			analogGainPrev = analogGain;
		}
		if (lines != linesPrev) {
			send_message(23, lines);
			linesPrev = lines;
		}
		if (pixels != pixelsPrev) {
			send_message(24, pixels);
			pixelsPrev = pixels;
		}
		if (xStart != xStartPrev) {
			send_message(25, xStart);
			xStartPrev = xStart;
		}
		if (xEnd != xEndPrev) {
			send_message(26, xEnd);
			xEndPrev = xEnd;
		}
		if (xSize != xSizePrev) {
			send_message(27, xSize);
			xSizePrev = xSize;
		}
		if (yStart != yStartPrev) {
			send_message(28, yStart);
			yStartPrev = yStart;
		}
		if (yEnd != yEndPrev) {
			send_message(29, yEnd);
			yEndPrev = yEnd;
		}
		if (ySize != ySizePrev) {
			send_message(30, ySize);
			ySizePrev = ySize;
		}
		if (EXCK_FREQ != EXCK_FREQ_PREV) {
			send_message(31, EXCK_FREQ);
			EXCK_FREQ_PREV = EXCK_FREQ;
		}
		if (VTPXCK_DIV != VTPXCK_DIV_PREV) {
			send_message(32, VTPXCK_DIV);
			VTPXCK_DIV_PREV = VTPXCK_DIV;
		}
		if (VTSYCK_DIV != VTSYCK_DIV_PREV) {
			send_message(33, VTSYCK_DIV);
			VTSYCK_DIV_PREV = VTSYCK_DIV;
		}
		if (PREPLLCK_VT_DIV != PREPLLCK_VT_DIV_PREV) {
			send_message(34, PREPLLCK_VT_DIV);
			PREPLLCK_VT_DIV_PREV = PREPLLCK_VT_DIV;
		}
		if (PREPLLCK_OP_DIV != PREPLLCK_OP_DIV_PREV) {
			send_message(35, PREPLLCK_OP_DIV);
			PREPLLCK_OP_DIV_PREV = PREPLLCK_OP_DIV;
		}
		if (PLL_VT_MPY != PLL_VT_MPY_PREV) {
			send_message(36, PLL_VT_MPY);
			PLL_VT_MPY_PREV = PLL_VT_MPY;
		}
		if (OPPXCK_DIV != OPPXCK_DIV_PREV) {
			send_message(37, OPPXCK_DIV);
			OPPXCK_DIV_PREV = OPPXCK_DIV;
		}
		if (OPSYCK_DIV != OPSYCK_DIV_PREV) {
			send_message(38, OPSYCK_DIV);
			OPSYCK_DIV_PREV = OPSYCK_DIV;
		}
		if (PLL_OP_MPY != PLL_OP_MPY_PREV) {
			send_message(39, PLL_OP_MPY);
			PLL_OP_MPY_PREV = PLL_OP_MPY;
		}
		if (floorValMin != floorValMinPrev) {
			send_message(44, floorValMin);
			floorValMinPrev = floorValMin;
		}
		if (floorSatMin != floorSatMinPrev) {
			send_message(45, floorSatMin);
			floorSatMinPrev = floorSatMin;
		}
		if (floorHueMin != floorHueMinPrev) {
			send_message(46, floorHueMin);
			floorHueMinPrev = floorHueMin;
		}
		if (floorHueMax != floorHueMaxPrev) {
			send_message(47, floorHueMax);
			floorHueMaxPrev = floorHueMax;
		}
		if (lineTransferPixelsMax != lineTransferPixelsMaxPrev) {
			send_message(48, lineTransferPixelsMax);
			lineTransferPixelsMaxPrev = lineTransferPixelsMax;
		}
		if (lineFloorWindowSize != lineFloorWindowSizePrev) {
			send_message(49, lineFloorWindowSize);
			lineFloorWindowSizePrev = lineFloorWindowSize;
		}
		if (lineFloorPixelsMin != lineFloorPixelsMinPrev) {
			send_message(50, lineFloorPixelsMin);
			lineFloorPixelsMinPrev = lineFloorPixelsMin;
		}
		if (lineWindowSize != lineWindowSizePrev) {
			send_message(51, lineWindowSize);
			lineWindowSizePrev = lineWindowSize;
		}
		if (linePixelsMin != linePixelsMinPrev) {
			send_message(52, linePixelsMin);
			linePixelsMinPrev = linePixelsMin;
		}
		if (ballWindowSize != ballWindowSizePrev) {
			send_message(53, ballWindowSize);
			ballWindowSizePrev = ballWindowSize;
		}
		if (ballPixelsMin != ballPixelsMinPrev) {
			send_message(54, ballPixelsMin);
			ballPixelsMinPrev = ballPixelsMin;
		}
		if (ballFalsePixelsMax != ballFalsePixelsMaxPrev) {
			send_message(55, ballFalsePixelsMax);
			ballFalsePixelsMaxPrev = ballFalsePixelsMax;
		}

		if (obstacleFloorWindowSize != obstacleFloorWindowSizePrev) {
			send_message(56, obstacleFloorWindowSize);
			obstacleFloorWindowSizePrev = obstacleFloorWindowSize;
		}
		if (obstacleLineWindowSize != obstacleLineWindowSizePrev) {
			send_message(57, obstacleLineWindowSize);
			obstacleLineWindowSizePrev = obstacleLineWindowSize;
		}
		if (obstacleBallWindowSize != obstacleBallWindowSizePrev) {
			send_message(58, obstacleBallWindowSize);
			obstacleBallWindowSizePrev = obstacleBallWindowSize;
		}
		if (obstacleWindowSize != obstacleWindowSizePrev) {
			send_message(59, obstacleWindowSize);
			obstacleWindowSizePrev = obstacleWindowSize;
		}
		if (obstacleTransferPixelsMax != obstacleTransferPixelsMaxPrev) {
			send_message(60, obstacleTransferPixelsMax);
			obstacleTransferPixelsMaxPrev = obstacleTransferPixelsMax;
		}
		if (obstacleFloorPixelsMin != obstacleFloorPixelsMinPrev) {
			send_message(61, obstacleFloorPixelsMin);
			obstacleFloorPixelsMinPrev = obstacleFloorPixelsMin;
		}
		if (obstacleLinePixelsMin != obstacleLinePixelsMinPrev) {
			send_message(62, obstacleLinePixelsMin);
			obstacleLinePixelsMinPrev = obstacleLinePixelsMin;
		}
		if (obstacleBallPixelsMin != obstacleBallPixelsMinPrev) {
			send_message(63, obstacleBallPixelsMin);
			obstacleBallPixelsMinPrev = obstacleBallPixelsMin;
		}
		if (obstaclePixelsMin != obstaclePixelsMinPrev) {
			send_message(64, obstaclePixelsMin);
			obstaclePixelsMinPrev = obstaclePixelsMin;
		}
		if (shutter != shutterPrev) {
			send_message(65, shutter);
			shutterPrev = shutter;
		}

		if (cameraReset == 1) {
			// before sending reset, send all current values
			send_message(10, lineValMin);
			send_message(11, lineSatMax);
			send_message(12, ballValMin);
			send_message(13, ballSatMin);
			send_message(14, ballHueMin);
			send_message(15, ballHueMax);
			send_message(16, obstacleValMax);
			send_message(17, obstacleSatMax);
			send_message(18, red);
			send_message(19, green);
			send_message(20, blue);
			send_message(21, testPattern);
			send_message(22, analogGain);
			send_message(23, lines);
			send_message(24, pixels);
			send_message(25, xStart);
			send_message(26, xEnd);
			send_message(27, xSize);
			send_message(28, yStart);
			send_message(29, yEnd);
			send_message(30, ySize);
			send_message(31, EXCK_FREQ);
			send_message(32, VTPXCK_DIV);
			send_message(33, VTSYCK_DIV);
			send_message(34, PREPLLCK_VT_DIV);
			send_message(35, PREPLLCK_OP_DIV);
			send_message(36, PLL_VT_MPY);
			send_message(37, OPPXCK_DIV);
			send_message(38, OPSYCK_DIV);
			send_message(39, PLL_OP_MPY);
			send_message(43); // reset (with auto clear)
			send_message(44, floorValMin);
			send_message(45, floorSatMin);
			send_message(46, floorHueMin);
			send_message(47, floorHueMax);
			send_message(48, lineTransferPixelsMax);
			send_message(49, lineFloorWindowSize);
			send_message(50, lineFloorPixelsMin);
			send_message(51, lineWindowSize);
			send_message(52, linePixelsMin);
			send_message(53, ballWindowSize);
			send_message(54, ballPixelsMin);
			send_message(55, ballFalsePixelsMax);
			send_message(56, obstacleFloorWindowSize);
			send_message(57, obstacleLineWindowSize);
			send_message(58, obstacleBallWindowSize);
			send_message(59, obstacleWindowSize);
			send_message(60, obstacleTransferPixelsMax);
			send_message(61, obstacleFloorPixelsMin);
			send_message(62, obstacleLinePixelsMin);
			send_message(63, obstacleBallPixelsMin);
			send_message(64, obstaclePixelsMin);
			send_message(65, shutter);
			cameraReset = 0;
		}
	}

	for (size_t ii = 0; ii < 4; ii++) {
		sock[ii]->disconnect();
	}
}

int main(int argc, char** argv) {
	int opt = 0;
	bool interfaceLocal = false;
	while ((opt = getopt(argc, argv, "e")) != -1) {
		switch (opt) {
		case 'e':
			interfaceLocal = true;
			break;
		case 'h':
			printf("INFO    : -e use local Ethernet (lo) interface\n");
			exit(EXIT_SUCCESS);
			break;
		}
	}

	zynqControl *client = new zynqControl();

	client->update(interfaceLocal);

	return 0;
}

