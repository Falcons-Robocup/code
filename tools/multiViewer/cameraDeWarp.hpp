// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERA_DE_WARP_HPP
#define CAMERA_DE_WARP_HPP

#include <stdint.h>


class cameraDeWarp {
private:
	uint16_t deWarpLookup[DEWARP_WIDTH][DEWARP_HEIGHT]; // TODO: try if different types (e.g. uint16_t) results in higher performance (e.g. fits better in cache)

#define ANGLES_NUM 7
	double angleList[ANGLES_NUM];
#define MARKER_MAX 29
	uint16_t marker[ANGLES_NUM][MARKER_MAX];
#define PIXELS_MAX 800
	int deWarp[ANGLES_NUM][PIXELS_MAX]; // pixels to millimeters (assume for now we use only 400 pixels for the line detection

	uint16_t deWarpMax; // max value, used for range checking

	void fillLookupTable();

public:
	cameraDeWarp();
	// convert pixel x,y to distance in millimeters
	uint16_t getLut(size_t xx, size_t yy);

	uint16_t getMarker(size_t angle, size_t index);

	double getAngle(size_t index);
};

#endif
