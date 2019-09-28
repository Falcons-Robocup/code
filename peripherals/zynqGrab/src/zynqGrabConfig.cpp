// Copyright 2017 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "zynqGrabConfig.hpp"

using namespace std;

zynqGrabConfig::zynqGrabConfig() {
	cameraReset = false;
	lineValMin = 169;
	lineSatMax = 52;
	lineTransferPixelsMax = 32; // needs to large for close by white lines
	lineFloorWindowSize = 15;
	lineFloorPixelsMin = 10;
	lineWindowSize = 31; // need to be larger then 15, because there might be non white pixels on the line
	linePixelsMin = 0; // small to be able to catch far away pixels

	// ball color detection
	ballValMin = 125;
	ballSatMin = 50;
	ballHueMin = 15; // yellow is 60 degrees => 60/2=30
	ballHueMax = 35;
	ballWindowSize = 4;
	ballPixelsMin = 2;
	ballFalsePixelsMax = 4;

	// floor color detection
	floorValMin = 55;
	floorSatMin = 20; // should be pretty low, to block light spots which might be recognized as lines
	floorHueMin = 55; // floor is 200 degrees => 200/2=100
	floorHueMax = 95;
	// obstacle color detection
	obstacleValMax = 70;
	obstacleSatMax = 60;
	obstacleFloorWindowSize = 20;
	obstacleFloorPixelsMin = 10;
	obstacleLineWindowSize = 10;
	obstacleLinePixelsMin = 5;
	obstacleBallWindowSize = 10;
	obstacleBallPixelsMin = 5;
	obstacleTransferPixelsMax = 20;
	obstacleWindowSize = 30;
	obstaclePixelsMin = 20;

	red = 128;
	green = 97;
	blue = 176;

	testPattern = 0;
	analogGain = 190; // 0x0157
	shutter = 209; // use this one to compensate for the 50Hz mains frequency;

	// number of effective pixels 3296 x 2480
	// number of active pixels 3280 x 2464
	// camera configured for x4 binning : 3280/4 = 820, 2464/4 = 616
	lines = 2728; // 0x0160-0x0161 FRM_LENGTH_A : how does this relate to 2480/2454
	pixels = 3448 + 16; // 0x0162-0x0163 LINE_LENGTH_A : 3280 (active pixels) + 168 (blanking), added 16 to reduce sync errors, TODO: investigate why this occurs
	xStart = 0; // 0x0164-0x0165 X_ADD_STA_A : X-address on the top left corner of the visible pixel data, default 0
	xEnd = 3279; // 0x0166-0x0167 X_ADD_END_A : X-address on the bottom right corner of the visible pixel data, default 3279
	xSize = 1280 + 16; // 0x016c-0x016d : X_OUTPUT_SIZE_A : width of output image 1280 + 16 ?? size required by FPGA CS-2 decoder ?
	yStart = 0; // 0x0168-0x0169 Y_ADD_STA_A : Y-address on the top left left corner of the visible pixel data, default 0
	yEnd = 2463; // 0x016a-0x016b Y_ADD_END_A : Y-address on the bottom right corner of the visible pixel data : default 2463
	ySize = 720 + 16; // 0x016e-0x016f Y_OUTPUT_SIZE_A : height of output image, 720 + 16 ?? size required by FPGA CS-2 decoder ?

	EXCK_FREQ = 0x1800;  // 0x012a-0x012b : 24.00MHz = 24MHz + 0.00MHz = 0x18 + 0x00 = 0x1800 = 6144
	VTPXCK_DIV = 5;      // 0x0301
	VTSYCK_DIV = 1;      // 0x0303
	PREPLLCK_VT_DIV = 2; // 0x0304
	PREPLLCK_OP_DIV = 2; // 0x0305
	PLL_VT_MPY = 0x0017; // 0x0306-0x0307 :  PLL video timing system multiplier value, works in range 0x10 to 0x19
	OPPXCK_DIV = 0x0a;   // 0x0309
	OPSYCK_DIV = 0x01;   // 0x030b
	PLL_OP_MPY = 0x002e; // 0x030c-0x030d : PLL output system multiplier value, works from 0x002a to 0x005c
}
