// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <math.h> // round
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "cameraDeWarp.hpp"

cameraDeWarp::cameraDeWarp() {

	// initialize lookup table with invalid value to verify if all locations are calculated
	for (size_t xx = 0; xx < DEWARP_WIDTH; xx++) {
		for (size_t yy = 0; yy < DEWARP_HEIGHT; yy++) {
			deWarpLookup[xx][yy] = 0xffff; // this deWarp value is higher then the calculated values
		}
	}

	// show the angles on which the markers can be calibrated
	for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
		angleList[angle] = 90.0 * angle / (ANGLES_NUM - 1);
		printf("angle index %zu is %.1f degrees\n", angle, angleList[angle]);
	}

// #define MARKER_MEASURED
#ifdef MARKER_MEASURED
	marker[0][0] = 0; // should be around 0 mm (e.g. correct for camera to robot cen
	marker[0][1] = 33;// 25 cm
	marker[0][2] = 65;// 50 cm
	marker[0][3] = 104;// 75 cm
	marker[0][4] = 136;// 100 cm
	marker[0][5] = 160;// 125 cm
	marker[0][6] = 180;// 150 cm
	marker[0][7] = 202;// 175 cm
	marker[0][8] = 218;// 200 cm
	marker[0][9] = 230;// 225 cm
	marker[0][10] = marker[0][9] + 19;// 250 cm
	marker[0][11] = marker[0][10] + 17;// 275 cm
	marker[0][12] = marker[0][11] + 15;// 300 cm
	marker[0][13] = marker[0][12] + 13;// 325 cm
	marker[0][14] = marker[0][13] + 12;// 350 cm
	marker[0][15] = marker[0][14] + 11;// 375 cm
	marker[0][16] = marker[0][15] + 10;// 400 cm
	marker[0][17] = marker[0][16] + 9;// 425 cm
	marker[0][18] = marker[0][17] + 8;// 450 cm
	marker[0][19] = marker[0][18] + 7;// 475 cm
	marker[0][20] = marker[0][19] + 6;// 500 cm
	marker[0][21] = marker[0][20] + 5;// 525 cm
	marker[0][22] = marker[0][21] + 5;// 550 cm
	marker[0][23] = marker[0][22] + 4;// 575 cm
	marker[0][24] = marker[0][23] + 4;// 600 cm
	marker[0][25] = marker[0][24] + 3;// 625 cm
	marker[0][26] = marker[0][25] + 3;// 650 cm
	marker[0][27] = marker[0][26] + 2;// 675 cm
	marker[0][28] = marker[0][27] + 2;// 700 cm, should be around 400
#define MARKER_MAX_VALUE 1000

#else

	// Use some made up values for the markers, these have to be measured
	// with the camera setup on a real field.
	// They should be the same for all cameras on all robots
	// They are used as the base to fill the deWarp lookup table

	// The markers are on fixed location (e.g. 0.25 meter spacing)
	// So this table is a conversion from meters to pixels
	// The goal is is to use this table as base for the reverse conversion
	// from pixels to meters

	// the image is 1280 wide, about 2/5 of the image will be used for the line detection
	// 1280/(2/5) = 512
	// 512/28 = 18.3
	// so for the simplicity let's assume at pixel 400 we are at distance 700 cm
	// in the end a few pixels is already quite some cm
	marker[0][0] = 0; // should be around 0 mm (e.g. correct for camera to robot centre)
	marker[0][1] = marker[0][0] + 60; // 25 cm
	marker[0][2] = marker[0][1] + 54; // 50 cm
	marker[0][3] = marker[0][2] + 48; // 75 cm
	marker[0][4] = marker[0][3] + 43; // 100 cm
	marker[0][5] = marker[0][4] + 38; // 125 cm
	marker[0][6] = marker[0][5] + 34; // 150 cm
	marker[0][7] = marker[0][6] + 30; // 175 cm
	marker[0][8] = marker[0][7] + 26; // 200 cm
	marker[0][9] = marker[0][8] + 22; // 225 cm
	marker[0][10] = marker[0][9] + 19; // 250 cm
	marker[0][11] = marker[0][10] + 17; // 275 cm
	marker[0][12] = marker[0][11] + 15; // 300 cm
	marker[0][13] = marker[0][12] + 13; // 325 cm
	marker[0][14] = marker[0][13] + 12; // 350 cm
	marker[0][15] = marker[0][14] + 11; // 375 cm
	marker[0][16] = marker[0][15] + 10; // 400 cm
	marker[0][17] = marker[0][16] + 9; // 425 cm
	marker[0][18] = marker[0][17] + 8; // 450 cm
	marker[0][19] = marker[0][18] + 7; // 475 cm
	marker[0][20] = marker[0][19] + 6; // 500 cm
	marker[0][21] = marker[0][20] + 5; // 525 cm
	marker[0][22] = marker[0][21] + 5; // 550 cm
	marker[0][23] = marker[0][22] + 4; // 575 cm
	marker[0][24] = marker[0][23] + 4; // 600 cm
	marker[0][25] = marker[0][24] + 3; // 625 cm
	marker[0][26] = marker[0][25] + 3; // 650 cm
	marker[0][27] = marker[0][26] + 2; // 675 cm
	marker[0][28] = marker[0][27] + 2; // 700 cm, should be around 400
#define MARKER_MAX_VALUE 1000

#endif

	// show the markers created for testing (in the real world they need to be measured)
	for (size_t ii = 0; ii < MARKER_MAX; ii++) {
		printf("INFO     : marker index %2zu distance %3zu cm relates to %3d pixels\n", ii, ii * 25, marker[0][ii]);
	}

	// create the markers for the other angles, normally these need to be measured to
	// there is distortion in the lens, meaning pixels at 0 degrees will have a different
	// distance then pixels at e.g. 60 degrees
	// assume that e.g. 100 pixels at 60 degrees is more meters than 100 pixels at 0 degrees
	// so the meters to pixel conversion decrease when the angle increases
	for (size_t angle = 1; angle < ANGLES_NUM; angle++) {
		for (size_t jj = 0; jj < MARKER_MAX; jj++) {
			marker[angle][jj] = round(1.0 * marker[0][jj] * (1.0 - 0.5 * angle / ( ANGLES_NUM - 1)));
		}
	}

	// verify if the markers are in range
	for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
		for (size_t jj = 0; jj < MARKER_MAX; jj++) {
			if ((jj == 0) && (marker[angle][jj] != 0)) {
				printf("ERROR    : marker at angle %.1f marker index 0 should be 0 but is %d\n", angleList[angle],
						marker[angle][jj]);
				exit(EXIT_FAILURE);

			} else if (marker[angle][jj] > MARKER_MAX_VALUE) {
				printf("ERROR    : marker at angle %.1f marker index %zu of %d out of range %d\n", angleList[angle], jj,
						marker[angle][jj], MARKER_MAX_VALUE);
				exit(EXIT_FAILURE);
			}
		}
	}

	// verify if markers decrease when angle increases
	for (size_t angle = 0; angle < ANGLES_NUM - 1; angle++) {
		for (size_t jj = 0; jj < MARKER_MAX; jj++) {
			if (marker[angle + 1][jj] > marker[angle][jj]) {
				printf("ERROR    : marker at angle %zu of %d shall not be higher then at angle %zu of %d\n", angle + 1,
						marker[angle + 1][jj], angle, marker[angle][jj]);
				exit(EXIT_FAILURE);
			}
		}
	}

	fillLookupTable();
}

void cameraDeWarp::fillLookupTable() {

// set all deWarp pixels to 0
	for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
		for (size_t pixel = 0; pixel < PIXELS_MAX; pixel++) {
			deWarp[angle][pixel] = -1;
		}
	}

// center of the lookupTable is at 0, HIEGHT/2
// at that location the pixel to meter ratio is 0
// on the center x-axis the 0 degree correction can be used
// assume each marker is 500mm
// then marker at pixel 86 is 500mm and
// maker at pixel 270 is 4500mm
// inter-extrapolate one calibration vector
// Note: this is the most lousy job you can do, but expected to be good enough for our use
// the first marker is at 0 (because 0 pixels is 0 meter)
// let's assume each remap pixels is 1cm
// then with a view of 7 meter we need a counter to 700
// marker[0] = 0 = 0
// marker[1] = 15 pixels = 25 cm, then calculate linear the pixels 1 to 14
// deltaPixel = maker[1] - marker[0];
// deltaMeter = 0.25 meter = 250mm
// pixel = meter[0] + 0.25*delta/deltaPixel;
// meter = (marker[1] - marker[0] ) / meter[1] - meter[0]
// marker[2] = 86 pixels = 50 cm
// marker[3] = 141 pixels = 75 cm5
// marker[4] = 179 pixels = 100 cm
	for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
		int markerIndex = 0;
		int counter = 0;
		int markerDelta = marker[angle][markerIndex + 1] - marker[angle][markerIndex]; // 15 - 0 = 15
		if (markerDelta == 0) {
			printf("new marker delta %d cm at pixel %d which relates from %d to %d mm xxxx\n", markerDelta, 0,
					marker[angle][markerIndex], marker[angle][markerIndex + 1]);
			printf("ERROR marker delta is 0 for pixel %d at angle %.1f, marker index low %d high %d xxxx\n", 0,
					angleList[angle], markerIndex, markerIndex + 1);
			exit(EXIT_FAILURE);
		}

//		printf(
//			"new marker delta %d cm at pixel %d which relates from %d to %d mm\n",
//		markerDelta, 0, marker[angle][markerIndex],
//	marker[angle][markerIndex + 1]);
// next one 86 - 15 = 71
// next one 141 - 86 = 55
// next one 179 - 141 = 38
		for (size_t pixel = 0; pixel < PIXELS_MAX; pixel++) {
			// deWarp[0] = 0 + 250 * ( 0 / 15 )
			// deWarp[1] = 0 + 250 * ( 1 / 15 )
			// deWarp[2] = 0 + 250 * ( 2 / 15 )
			// deWarp[15] = 0 + 250 * ( 15 / 15 )
			// deWarp[16] = 250 + 250 * ( 1 / 71 )
			// deWarp[81] = 250 + 250 * ( 71 / 71 )
			// deWarp[82] = 500 + 250 * ( 1 / 55 )
			// deWarp[141] = 500 + 250 * ( 55 / 55 )
			// deWarp[142] = 750 + 250 * ( 1 / 38 )
			// deWarp[179] = 750 + 250 * ( 38 / 38 )
			int value = 250 * markerIndex + round(250.0 * counter / markerDelta);
			if ((angle == (ANGLES_NUM - 1)) && (pixel == (PIXELS_MAX - 1))) {
				deWarpMax = value;
				printf("maximal distance for pixel %zu is %d mm\n", pixel, value);
			}
			int valueMaximal = 250 * (markerIndex + 1);
			if (((value <= 0) && (pixel > 0)) || (value > valueMaximal)) {
				printf("ERROR: marker out of range %d for markerIndex %d counter %d pixel %zu angle %zu\n", value,
						markerIndex, counter, pixel, angle);
				exit(EXIT_FAILURE);
			}
			deWarp[angle][pixel] = value;
			if (counter == markerDelta) {
				// done with interpolation between to two markers
				// restart with interpolation with the next marker
				if (markerIndex < ( MARKER_MAX - 3)) {
					markerIndex++;
				}
				markerDelta = marker[angle][markerIndex + 1] - marker[angle][markerIndex];
				if (markerDelta == 0) {
					printf("new marker delta %d cm at pixel %zu which relates from %d to %d mm\n", markerDelta, pixel,
							marker[angle][markerIndex], marker[angle][markerIndex + 1]);
					printf("ERROR marker delta is 0 for pixel %zu at angle %.1f, marker index low %d high %d\n", pixel,
							angleList[angle], markerIndex, markerIndex + 1);
					exit(EXIT_FAILURE);
				}
				counter = 1;
			} else {
				counter++;
			}
		}
	}

// check if calculated values are in range
	for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
		for (size_t pixel = 0; pixel < PIXELS_MAX; pixel++) {
			if (deWarp[angle][pixel] < 0 || deWarp[angle][pixel] > 8000) {
				printf("ERROR pixel %zu deWarp %d for angle %.1f out of range\n", pixel, deWarp[angle][pixel],
						angleList[angle]);
				exit( EXIT_FAILURE);

			}
		}
	}

// check if the interpolated pixel deWarp becomes lower when the angle increases
// (the ratio is highest straight for the camera)
	int errorCnt = 0;
	for (size_t angle = 0; angle < ANGLES_NUM - 1; angle++) {
		for (size_t pixel = 0; pixel < PIXELS_MAX; pixel++) {
			// WARNING: the check does not work on the end of the pixels because one might not go as far as the other
			if ((deWarp[angle][pixel] < 6000) && (deWarp[angle + 1][pixel] < deWarp[angle][pixel])) {
				printf("ERROR pixel %zu deWarp %d for angle %.1f should be larger then %d for angle %.1f\n", pixel,
						deWarp[angle + 1][pixel], angleList[angle + 1], deWarp[angle][pixel], angleList[angle]);
				if (errorCnt > 5) {
					exit( EXIT_FAILURE);
				}
				errorCnt++;

			}
		}
	}

// now try to calculate the values for the other pixels
// for each pixel get the radius from the 0 point (x = 0, y = HEIGHT/2)
// get the angle line left of the pixel, lookup the deWarp factor
// get the angle line right of the pixel, loocup the deWarp factor
// determine the distance from the pixel to the left angle
// determine the distance from the pixel to the right angle
// determine the delta between the deWarp factor
// calculate ratio
	for (int x = 0; x < DEWARP_WIDTH / 2; x++) {
		for (int y = 0; y < DEWARP_HEIGHT / 2; y++) {
			double anglePixel = 360.0 * atan2(y, x) / (2 * M_PI);
			int radius = round(sqrt(pow(x, 2) + pow(y, 2)));

			if (radius >= PIXELS_MAX) {
				radius = PIXELS_MAX - 1;
			}

			int angleLow = 0;
			int angleHigh = ANGLES_NUM - 1;
			for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
				if (anglePixel > angleList[angle]) {
					angleLow = angle;
				}
				int angleInverse = ANGLES_NUM - angle - 1;
				if (anglePixel < angleList[angleInverse]) {
					angleHigh = angleInverse;
				}
			}

			int radiusDeWarpForLowAngle = deWarp[angleLow][radius]; // in mm
			int radiusDeWarpForHighAngle = deWarp[angleHigh][radius]; // in mm
			// TODO: do something with the maximal length limitation of 6000mm
			if ((radiusDeWarpForLowAngle < 6000) & (radiusDeWarpForLowAngle > radiusDeWarpForHighAngle)) {
				printf(
						"ERROR radiusWarpForLowAngle %d mm should be lower then radiusWarpForHighAngle %d mm for angle %.1f pixel %d\n",
						radiusDeWarpForLowAngle, radiusDeWarpForHighAngle, anglePixel, radius);
				exit(EXIT_FAILURE);
			}

			int radiusDeWarpDelta = radiusDeWarpForLowAngle - radiusDeWarpForHighAngle; // in mm

			double angleDeltaLow = anglePixel - angleList[angleLow];
			double angleDeltaHigh = angleList[angleHigh] - anglePixel;
			double angleDelta = angleList[angleHigh] - angleList[angleLow];
			if (angleDelta < 1.0) {
				printf("ERROR: angleDelta of %.1f is to small\n", angleDelta);
				exit(EXIT_FAILURE);
			}

			int radiusDeWarpPixel = radiusDeWarpForHighAngle + round(radiusDeWarpDelta * (angleDeltaHigh / angleDelta));

			int radiusDeWarpPixelMaximal = radiusDeWarpForLowAngle;
			int radiusDeWarpPixelMinimal = radiusDeWarpForHighAngle;
			if (radiusDeWarpForHighAngle > radiusDeWarpForLowAngle) {
				radiusDeWarpPixelMaximal = radiusDeWarpForHighAngle;
				radiusDeWarpPixelMinimal = radiusDeWarpForLowAngle;
			}
			if (radiusDeWarpPixel > deWarpMax) {
				printf("ERROR: radiusDeWarpPixel of %d should be lower then max de warp %d\n", radiusDeWarpPixel,
						deWarpMax);
				exit(EXIT_FAILURE);
			}

			if (radiusDeWarpPixel > radiusDeWarpPixelMaximal) {
				printf("ERROR: radiusDeWarpPixel of %d should be larger then %d\n", radiusDeWarpPixel,
						radiusDeWarpPixelMaximal);
				exit(EXIT_FAILURE);
			}
			if (radiusDeWarpPixel < radiusDeWarpPixelMinimal) {
				printf("ERROR: radiusDeWarpPixel of %d should be smaller then %d\n", radiusDeWarpPixel,
						radiusDeWarpPixelMinimal);
				exit(EXIT_FAILURE);
			}

			if ((radiusDeWarpForLowAngle < 6000)
					&& ((radiusDeWarpPixel < radiusDeWarpForLowAngle) || (radiusDeWarpPixel > radiusDeWarpForHighAngle))) {
				printf("angle pixel %.1f degrees, radius %d pixels, angle low index %d angle high index %d\n",
						anglePixel, radius, angleLow, angleHigh);
				printf("angleDeltaLow %.1f degrees, angleDeltaHigh %.1f degrees, angleDelta %.1f\n", angleDeltaLow,
						angleDeltaHigh, angleDelta);
				printf("angle low %.1f angle high %.1f\n", angleList[angleLow], angleList[angleHigh]);
				printf(
						"radiusWarpForLowAngle %d mm, radiusWarpForHighAngle %d mm, radiusDeWarpDelta %d mm, rediusDeWarpPixel %d mm\n",
						radiusDeWarpForLowAngle, radiusDeWarpForHighAngle, radiusDeWarpDelta, radiusDeWarpPixel);
				printf("ERROR: calculation of radiusDeWarpPixel out of range\n");
				exit(EXIT_FAILURE);
			}
			deWarpLookup[x][y + DEWARP_HEIGHT / 2] = radiusDeWarpPixel; // 720 /2 = 360, yy = [0:359] -> 360 to 719
			deWarpLookup[x][-y - 1 + DEWARP_HEIGHT / 2] = radiusDeWarpPixel; // -359 + 360 = 1 to 360
			// copy the last line because the center (at 360) is the same in both lines and the deWarp shall work for the complete y
			if (y == ( DEWARP_HEIGHT / 2 - 1)) {
				deWarpLookup[x][DEWARP_HEIGHT - 1] = radiusDeWarpPixel; // -359 + 360 = 1 to 360

			}
		}
	}

	// check if calculated value not out of range (and if default values overwritten)
	errorCnt = 0;
	for (size_t xx = 0; xx < DEWARP_WIDTH / 2; xx++) {
		for (size_t yy = 0; yy < DEWARP_HEIGHT; yy++) {
			if (deWarpLookup[xx][yy] > deWarpMax) {
				printf("ERROR    : value %d higher then %d for x %zu and y %zu\n", deWarpLookup[xx][yy], deWarpMax, xx,
						yy);
				if (errorCnt >= 10) {
					exit(EXIT_FAILURE);
				}
				errorCnt++;
			}
		}
	}

	// draw the deWarp for all 10 angles over the already created lookup table, to make visible if a calculation error was made
	for (size_t angle = 0; angle < ANGLES_NUM; angle++) {
		for (size_t pixel = 0; pixel < PIXELS_MAX; pixel++) {
			// calculate x and y when tilt of 15 degrees
			int x = round(pixel * cos(angleList[angle] * 2 * M_PI / 360.0));
			int y = round(pixel * sin(angleList[angle] * 2 * M_PI / 360.0));
			if (y < ( DEWARP_HEIGHT / 2)) {
				deWarpLookup[x][y + DEWARP_HEIGHT / 2] = 2 * deWarp[angle][pixel];
				deWarpLookup[x][-y + DEWARP_HEIGHT / 2] = 2 * deWarp[angle][pixel];
			}
		}
	}

}

uint16_t cameraDeWarp::getLut(size_t xx, size_t yy) {
	if (xx >= DEWARP_WIDTH) {
		printf("ERROR    : deWarp lut x index of %zu out range %d\n", xx, DEWARP_WIDTH);
		exit(EXIT_FAILURE);
	}
	if (yy >= DEWARP_HEIGHT) {
		printf("ERROR    : deWarp lut y index of %zu out range %d\n", yy, DEWARP_HEIGHT);
		exit(EXIT_FAILURE);
	}
	return deWarpLookup[xx][yy];
}

uint16_t cameraDeWarp::getMarker(size_t angle, size_t index) {
	if (angle >= ANGLES_NUM) {
		printf("ERROR    : deWarp marker angle of %zu out range %d\n", angle, ANGLES_NUM);
		exit(EXIT_FAILURE);
	}
	if (index >= MARKER_MAX) {
		printf("ERROR    : deWarp marker index value of %zu out range %d\n", index, MARKER_MAX);
		exit(EXIT_FAILURE);
	}
	return marker[angle][index];
}

double cameraDeWarp::getAngle(size_t index) {
	if (index >= ANGLES_NUM) {
		printf("ERROR    : deWarp angle index value of %zu out range %d\n", index, ANGLES_NUM);
		exit(EXIT_FAILURE);
	}
	return angleList[index];
}
