// Copyright 2019-2020 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "raspiControl.hpp"
#include "raspiDefaults.hpp"

using namespace cv;
using namespace std;

raspiControl::raspiControl() {
	camSysRecv = new camSysReceive();

	multFd = 0;
	memset((char *) &toAddr, 0, sizeof(toAddr));

	txPacketCnt = 0;

	struct timeval tv;
	gettimeofday(&tv, NULL);
	systemControlSendTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;

	// start thread that collects system data from multiple camera's
	camSysRecvThread = thread(&camSysReceive::receive, camSysRecv);
}

void raspiControl::multiCastSetup() {
	// create a normal UDP socket
	if ((multFd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cannot create UDP socket!, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(22222); // configuration port

	printf("INFO      : control multicast IP address %s port %u\n", inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));
}

void raspiControl::multiCastClose() {
	close(multFd);
}

void raspiControl::reboot() {
	txPacket.pl.u32[0] = 0xdead0022;

	txPacket.id = 99; // packet id for the analyzer and camera control
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 4;

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for reboot\n", actualSize, txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::poweroff() {
	txPacket.pl.u32[0] = 0xdead0011;

	txPacket.id = 98; // packet id for the analyzer and camera control
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 4;

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for reboot\n", actualSize, txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::analyzeControl() {
	size_t ii = (uint32_t) 0;
	txPacket.pl.u16[ii++] = 10;

	txPacket.id = 34; // packet id for the analyzer and camera control
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + (ii * 2); // every value is 2 bytes

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for analyzer configuration\n", actualSize, txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::grabControl() {

	size_t ii = 0;
	txPacket.pl.f32[ii++] = (float) grabberRedGain / 100.0;
	txPacket.pl.f32[ii++] = (float) grabberBlueGain / 100.0;
	txPacket.id = 32; // packet id for controlling grabber process

	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + (ii * 4); // every value is 4 bytes

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for grabber configuration\n", actualSize, txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::camControlSetup() {

	// camera clocks (imx219)
	EXCK_FREQ = EXCK_FREQ_DEFAULT; // 0x012a-0x012b
	VTPXCK_DIV = VTPXCK_DIV_DEFAULT; // 0x0301
	VTSYCK_DIV = VTSYCK_DIV_DEFAULT; // 0x0303
	PREPLLCK_VT_DIV = PREPLLCK_VT_DIV_DEFAULT; // 0x0304
	PREPLLCK_OP_DIV = PREPLLCK_OP_DIV_DEFAULT; // 0x0305
	PLL_VT_MPY = PLL_VT_MPY_DEFAULT; // 0x0306-0x0307
	OPPXCK_DIV = OPPXCK_DIV_DEFAULT; // 0x0309
	OPSYCK_DIV = OPSYCK_DIV_DEFAULT; // 0x030b
	PLL_OP_MPY = PLL_OP_MPY_DEFAULT; // 0x030c-0x030d

	string clockConfig = "camera clock";
	namedWindow(clockConfig, cv::WINDOW_NORMAL);
	createTrackbar("EXCK_FREQ", clockConfig, &EXCK_FREQ, 0x8000); // 0x012a, 0x012b
	createTrackbar("VTPXCK_DIV", clockConfig, &VTPXCK_DIV, 10); // 0x0301, 5 bits, but only values 4, 5, 8 and 10 are allowed
	createTrackbar("VTSYCK_DIV", clockConfig, &VTSYCK_DIV, 3); // 0x03O3, 2 bits, but only value 1 is allowed
	createTrackbar("PREPLLCK_VT_DIV", clockConfig, &PREPLLCK_VT_DIV, 3); // 0x0304, 8 bits, but only 1, 2 and 3 are allowed
	createTrackbar("PREPLLCK_OP_DIV", clockConfig, &PREPLLCK_OP_DIV, 3); // 0x0305, 8 bits, but only 1, 2 and 3 are allowed
	createTrackbar("PLL_VT_MPY", clockConfig, &PLL_VT_MPY, 255); // 0x0306, 0x0307, 11 bits, but PLL max output frequency is 916 MHz
	createTrackbar("OPPXCK_DIV", clockConfig, &OPPXCK_DIV, 10); // 0x0309, 5 bits, but only value 8 and 10 are allowed
	createTrackbar("OPSYCK_DIV", clockConfig, &OPSYCK_DIV, 3); // 0x030b, 2 bits, but only value 1 is allowed
	createTrackbar("PLL_OP_MPY", clockConfig, &PLL_OP_MPY, 255); // 0x030c, 0x030d, 11 bits, but PLL max output frequency is 916 MHz

	// camera control and white balance (software)
	cameraReset = CAMERA_RESET;
	cameraVerbose = CAMERA_VERBOSE;
	testPattern = TEST_PATTERN;
	analogGain = ANALOG_GAIN;
	shutter = SHUTTER;
	blackLevel = BLACK_LEVEL;

	string cameraConfig = "camera config";
	namedWindow(cameraConfig, cv::WINDOW_NORMAL);
	createTrackbar("reset", cameraConfig, &cameraReset, 1); // 0x0103
	createTrackbar("verbose", cameraConfig, &cameraVerbose, 1);
	createTrackbar("test pattern", cameraConfig, &testPattern, 9);
	createTrackbar("analog gain", cameraConfig, &analogGain, 255);
	createTrackbar("shutter", cameraConfig, &shutter, 2047);
	createTrackbar("black level", cameraConfig, &blackLevel, 1023);

	// camera dimensions
	lines = LINES; // 0x0160-0x0161
	pixels = PIXELS; // 0x0162-0x0163
	xStart = X_START; // 0x0164-0x0165
	xEnd = X_END; // 0x0166-0x0167
	xSize = X_SIZE; // 0x016c-0x016d
	yStart = Y_START; // 0x0168-0x0169
	yEnd = Y_END; // 0x016a-0x016b
	ySize = Y_SIZE; // 0x016e-0x016f
	imageOrientation = IMAGE_ORIENTATION; // 0x0172

	for (size_t ii = 0; ii < 4; ii++) {
		pixelsOffset[ii] = 16;
	}

	string sizeConfig = "dimensions";
	namedWindow(sizeConfig, cv::WINDOW_NORMAL);
	createTrackbar("lines", sizeConfig, &lines, 3000);
	createTrackbar("pixels", sizeConfig, &pixels, 4000);
	createTrackbar("p offset cam0", sizeConfig, &pixelsOffset[0], 32);
	createTrackbar("p offset cam1", sizeConfig, &pixelsOffset[1], 32);
	createTrackbar("p offset cam2", sizeConfig, &pixelsOffset[2], 32);
	createTrackbar("p offset cam3", sizeConfig, &pixelsOffset[3], 32);
	createTrackbar("x start", sizeConfig, &xStart, 1000);
	createTrackbar("x end", sizeConfig, &xEnd, 4000);
	createTrackbar("x size", sizeConfig, &xSize, 4000);
	createTrackbar("y start", sizeConfig, &yStart, 2000);
	createTrackbar("y end", sizeConfig, &yEnd, 3000);
	createTrackbar("y size", sizeConfig, &ySize, 2000);
	createTrackbar("img orient", sizeConfig, &imageOrientation, 3);

	// line point detection
	lineValMin = LINE_VAL_MIN;
	lineSatMax = LINE_SAT_MAX;
	lineTransferPixelsMax = LINE_TRANSFER_PIXELS_MAX;
	lineFloorWindowSize = LINE_FLOOR_WINDOW_SIZE;
	lineFloorPixelsMin = LINE_FLOOR_PIXELS_MIN;
	lineWindowSize = LINE_WINDOW_SIZE;
	linePixelsMin = LINE_PIXELS_MIN;

	string lineConfig = "lines";
	namedWindow(lineConfig, cv::WINDOW_NORMAL);
	createTrackbar("line value min", lineConfig, &lineValMin, 255);
	createTrackbar("line saturation max", lineConfig, &lineSatMax, 255);
	createTrackbar("line trans max", lineConfig, &lineTransferPixelsMax, 32);
	createTrackbar("line floor window", lineConfig, &lineFloorWindowSize, 32);
	createTrackbar("line pixel floor max", lineConfig, &lineFloorPixelsMin, 32);
	createTrackbar("line window max", lineConfig, &lineWindowSize, 32);
	createTrackbar("line pixels min", lineConfig, &linePixelsMin, 32);

	// ball point detection
	ballValMin = BALL_VAL_MIN;
	ballSatMin = BALL_SAT_MIN;
	ballHueMin = BALL_HUE_MIN;
	ballHueMax = BALL_HUE_MAX;
	ballWindowSize = BALL_WINDOW_SIZE;
	ballPixelsMin = BALL_PIXELS_MIN;
	ballFalsePixelsMax = BALL_FALSE_PIXELS_MAX;

	// ballFar point detection
	ballFarValMin = BALL_FAR_VAL_MIN;
	ballFarSatMin = BALL_FAR_SAT_MIN;
	ballFarHueMin = BALL_FAR_HUE_MIN;
	ballFarHueMax = BALL_FAR_HUE_MAX;
	ballFarWindowSize = BALL_FAR_WINDOW_SIZE;
	ballFarPixelsMin = BALL_FAR_PIXELS_MIN;
	ballFarFalsePixelsMax = BALL_FAR_FALSE_PIXELS_MAX;

#ifdef WHITE_BLACK_BALL_SEARCH
	ballWhiteWindowSize = BALL_WHITE_WINDOW_SIZE;
	ballWhitePixelsMin = BALL_WHITE_PIXELS_MIN;
	ballBlackWindowSize = BALL_BLACK_WINDOW_SIZE;
	ballBlackPixelsMin = BALL_BLACK_PIXELS_MIN;
	ballWhiteBlackFalsePixelsMax = BALL_WHITE_BLACK_FALSE_PIXELS_MAX;
#endif

	string ballConfig = "ballsNearby";
	namedWindow(ballConfig, cv::WINDOW_NORMAL);
	createTrackbar("ball val min", ballConfig, &ballValMin, 255);
	createTrackbar("ball sat min", ballConfig, &ballSatMin, 255);
	createTrackbar("ball hue min", ballConfig, &ballHueMin, 255);
	createTrackbar("ball hue max", ballConfig, &ballHueMax, 255);
	createTrackbar("ball window size", ballConfig, &ballWindowSize, 255);
	createTrackbar("ball pixel min", ballConfig, &ballPixelsMin, 255);
	createTrackbar("ball false pix", ballConfig, &ballFalsePixelsMax, 255);

	string ballFarConfig = "ballsFar";
	namedWindow(ballFarConfig, cv::WINDOW_NORMAL);
	createTrackbar("ballFar val min", ballFarConfig, &ballFarValMin, 255);
	createTrackbar("ballFar sat min", ballFarConfig, &ballFarSatMin, 255);
	createTrackbar("ballFar hue min", ballFarConfig, &ballFarHueMin, 255);
	createTrackbar("ballFar hue max", ballFarConfig, &ballFarHueMax, 255);
	createTrackbar("ballFar window size", ballFarConfig, &ballFarWindowSize, 255);
	createTrackbar("ballFar pixel min", ballFarConfig, &ballFarPixelsMin, 255);
	createTrackbar("ballFar false pix", ballFarConfig, &ballFarFalsePixelsMax, 255);

#ifdef WHITE_BLACK_BALL_SEARCH
	createTrackbar("ball white win size", ballConfig, &ballWhiteWindowSize, 255);
	createTrackbar("ball white pixel min", ballConfig, &ballWhitePixelsMin, 255);
	createTrackbar("ball black win size", ballConfig, &ballBlackWindowSize, 255);
	createTrackbar("ball black pixel min", ballConfig, &ballBlackPixelsMin, 255);
	createTrackbar("ball wb false pix", ballConfig, &ballWhiteBlackFalsePixelsMax, 255);
#endif

	// floor point detection
	floorValMin = FLOOR_VAL_MIN;
	floorSatMin = FLOOR_SAT_MIN;
	floorHueMin = FLOOR_HUE_MIN;
	floorHueMax = FLOOR_HUE_MAX;

	string floorConfig = "floor";
	namedWindow(floorConfig, cv::WINDOW_NORMAL);
	createTrackbar("floor val min", floorConfig, &floorValMin, 255);
	createTrackbar("floor sat min", floorConfig, &floorSatMin, 255);
	createTrackbar("floor hue min", floorConfig, &floorHueMin, 255);
	createTrackbar("floor hue max", floorConfig, &floorHueMax, 255);

	// obstacle point detection
	obstacleValMax = OBSTACLE_VAL_MAX;
	obstacleSatMax = OBSTACLE_SAT_MAX;
	obstacleFloorWindowSize = OBSTACLE_FLOOR_WINDOW_SIZE;
	obstacleFloorPixelsMin = OBSTACLE_FLOOR_PIXELS_MIN;
	obstacleLineWindowSize = OBSTACLE_LINE_WINDOW_SIZE;
	obstacleLinePixelsMin = OBSTACLE_LINE_PIXELS_MIN;
	obstacleBallWindowSize = OBSTACLE_BALL_WINDOW_SIZE;
	obstacleBallPixelsMin = OBSTACLE_BALL_PIXELS_MIN;
	obstacleTransferPixelsMax = OBSTACLE_TRANSFER_PIXELS_MAX;
	obstacleWindowSize = OBSTACLE_WINDOW_SIZE;
	obstaclePixelsMin = OBSTACLE_PIXELS_MIN;

	string obstacleConfig = "obstacle";
	namedWindow(obstacleConfig, cv::WINDOW_NORMAL);
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
}

void raspiControl::grabControlSetup() {
	// grabber configuration
	grabberRedGain = GRABBER_RED_GAIN;
	grabberBlueGain = GRABBER_BLUE_GAIN;

	string grabberConfig = "grabber";
	namedWindow(grabberConfig, cv::WINDOW_NORMAL);
	createTrackbar("red gain", grabberConfig, &grabberRedGain, 300);
	createTrackbar("blue gain", grabberConfig, &grabberBlueGain, 300);
}

void raspiControl::camControl() {

	size_t ii = (uint16_t) 0;
	txPacket.pl.u16[ii++] = (uint16_t) lineValMin;
	txPacket.pl.u16[ii++] = (uint16_t) lineSatMax;
	txPacket.pl.u16[ii++] = (uint16_t) lineTransferPixelsMax;
	txPacket.pl.u16[ii++] = (uint16_t) lineFloorWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) lineFloorPixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) lineWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) linePixelsMin;

	txPacket.pl.u16[ii++] = (uint16_t) ballValMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballSatMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballHueMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballHueMax;
	txPacket.pl.u16[ii++] = (uint16_t) ballWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) ballPixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballFalsePixelsMax;

	txPacket.pl.u16[ii++] = (uint16_t) ballFarValMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballFarSatMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballFarHueMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballFarHueMax;
	txPacket.pl.u16[ii++] = (uint16_t) ballFarWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) ballFarPixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballFarFalsePixelsMax;

#ifdef WHITE_BLACK_BALL_SEARCH
	txPacket.pl.u16[ii++] = (uint16_t) ballWhiteWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) ballWhitePixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballBlackWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) ballBlackPixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) ballWhiteBlackFalsePixelsMax;
#endif

	txPacket.pl.u16[ii++] = (uint16_t) floorValMin;
	txPacket.pl.u16[ii++] = (uint16_t) floorSatMin;
	txPacket.pl.u16[ii++] = (uint16_t) floorHueMin;
	txPacket.pl.u16[ii++] = (uint16_t) floorHueMax;

	txPacket.pl.u16[ii++] = (uint16_t) obstacleValMax;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleSatMax;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleFloorWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleFloorPixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleLineWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleLinePixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleBallWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleBallPixelsMin;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleTransferPixelsMax;
	txPacket.pl.u16[ii++] = (uint16_t) obstacleWindowSize;
	txPacket.pl.u16[ii++] = (uint16_t) obstaclePixelsMin;

	txPacket.id = 31; // packet id for the analyzer and camera control
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + (ii * 2); // every value is 2 bytes

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for analyze configuration\n", actualSize, txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::systemControl() {

	// by now we should have received the control acknowledge from the raspi
	// calculate the UDP round trip by using the moment the previous control packet was send
	static size_t printCounter = 0;
	static double roundTripAvg[4] = { 0, 0, 0, 0 };
	static size_t roundTripAvgCount[4] = { 0, 0, 0, 0 };
	double roundTrip[4];
	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		roundTrip[camIndex] = camSysRecv->getCamSystem(camIndex).configAcknowledgeTime - systemControlSendTime;

		if (roundTrip[camIndex] > 0.00001 && roundTrip[camIndex] < 0.001) { // between 10us and 1ms
			roundTripAvg[camIndex] += roundTrip[camIndex];
			roundTripAvgCount[camIndex]++;
		}
	}
	if (printCounter % 16 == 8 || true) {
		printf("INFO      : round trip cam0 %7.1f (%7.1f) us cam1 %7.1f (%7.1f) us cam2 %7.1f (%7.1f) us cam3 %7.1f (%7.1f) us\n",
				1000000.0 * roundTrip[0], 1000000.0 * roundTripAvg[0] / roundTripAvgCount[0],
				1000000.0 * roundTrip[1], 1000000.0 * roundTripAvg[1] / roundTripAvgCount[1],
				1000000.0 * roundTrip[2], 1000000.0 * roundTripAvg[2] / roundTripAvgCount[2],
				1000000.0 * roundTrip[3], 1000000.0 * roundTripAvg[3] / roundTripAvgCount[3] );
	}
	printCounter++;

	size_t ii = (uint16_t) 0;
	txPacket.pl.u16[ii++] = (uint16_t) EXCK_FREQ;
	txPacket.pl.u16[ii++] = (uint16_t) VTPXCK_DIV;
	txPacket.pl.u16[ii++] = (uint16_t) VTSYCK_DIV;
	txPacket.pl.u16[ii++] = (uint16_t) PREPLLCK_VT_DIV;
	txPacket.pl.u16[ii++] = (uint16_t) PREPLLCK_OP_DIV;
	txPacket.pl.u16[ii++] = (uint16_t) PLL_VT_MPY;
	txPacket.pl.u16[ii++] = (uint16_t) OPPXCK_DIV;
	txPacket.pl.u16[ii++] = (uint16_t) OPSYCK_DIV;
	txPacket.pl.u16[ii++] = (uint16_t) PLL_OP_MPY;
	txPacket.pl.u16[ii++] = (uint16_t) cameraReset;
	txPacket.pl.u16[ii++] = (uint16_t) cameraVerbose;
	txPacket.pl.u16[ii++] = (uint16_t) testPattern;
	txPacket.pl.u16[ii++] = (uint16_t) analogGain;
	txPacket.pl.u16[ii++] = (uint16_t) shutter;
	txPacket.pl.u16[ii++] = (uint16_t) blackLevel;
	txPacket.pl.u16[ii++] = (uint16_t) lines;
	txPacket.pl.u16[ii++] = (uint16_t) pixels;
	txPacket.pl.u16[ii++] = (uint16_t) xStart;
	txPacket.pl.u16[ii++] = (uint16_t) xEnd;
	txPacket.pl.u16[ii++] = (uint16_t) xSize;
	txPacket.pl.u16[ii++] = (uint16_t) yStart;
	txPacket.pl.u16[ii++] = (uint16_t) yEnd;
	txPacket.pl.u16[ii++] = (uint16_t) ySize;
	txPacket.pl.u16[ii++] = (uint16_t) imageOrientation;

	txPacket.id = 64; // packet id for the raspiSystem camera control
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + (ii * 2); // every value is 2 bytes

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
        printf("ERROR   : send on %zd instead of %u bytes for raspiSystem camera control\n", actualSize, txPacket.size);
		fflush(stdout);
	}

	// store the time when the current control packet was send, used in the next cycle to calculate the UDP round trip
	struct timeval tv;
	gettimeofday(&tv, NULL);
	systemControlSendTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
}

void raspiControl::cam0FrameCntSend(uint32_t cam0FrameCounter) {
	txPacket.id = 69; // packet to synchronize frame counter of cam1, cam2 and cam3 with frame counter of cam0
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 4; // 4 bytes payload

	txPacket.pl.u32[0] = cam0FrameCounter; // send the frame counter from cam0 to all cameras

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for raspiSystem frame counter return\n", actualSize,
				txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::systemControlPixelsOffset() {
	for (size_t ii = 0; ii < 4; ii++) {
		int pixelOffset = pixelsOffset[ii] - 16;
		txPacket.pl.s8[ii] = (int8_t) pixelOffset;
	}

	txPacket.id = 67; // packet id for the raspiSystem camera control pixel offset list
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 4; // the for each camere 1 byte

	ssize_t actualSize = sendto(multFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR     : cannot send message, message %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (actualSize != txPacket.size) {
		printf("ERROR     : send on %zd instead of %u bytes for raspiSystem camera control\n", actualSize,
				txPacket.size);
		fflush(stdout);
	}
}

void raspiControl::updateSystemControlPixelsOffset(int pixelsOffset[4]) {
	for (size_t ii = 0; ii < 4; ii++) {
		this->pixelsOffset[ii] = pixelsOffset[ii];
	}
	// send the new values to the 4 raspi camera's
	systemControlPixelsOffset();
}

void raspiControl::update() {

	camControlSetup();
	grabControlSetup();

	bool keepGoing = true;

	while (keepGoing) {
		systemControl();
		systemControlPixelsOffset();
		camControl();
		grabControl();
		analyzeControl();

		int key = waitKey(1000); // send multicast packet only once a second
		if (key == 27 || key == 'q') { // escape or q
			keepGoing = false;
		}
	}
}
