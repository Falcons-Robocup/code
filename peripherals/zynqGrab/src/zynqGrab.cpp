// Copyright 2017 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// #define GRAB_STATIC_IMAGE


#if defined(__arm__)
#include <arm_neon.h>
#endif
#include <cerrno>
#include <fcntl.h> // for O_RDWR
#include <linux/fb.h> // for fb_
#include <stdint.h> // for uint32_t
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h> // for ioctl
#include <vector>
#include <unistd.h>
#include <arpa/inet.h>
#include <ctime>
#include <iostream>
#include <net/if.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <sys/ioctl.h>
#include <unistd.h>

#if defined(__arm__)
#include <linux/i2c-dev.h>
#endif

#include "zynqGrabConfig.hpp"
#include "tcp_socket.hpp"

using namespace std;

#define CAMERA_V2_1_IIC_ADDRESS		0x10

#define FRAME_GRABBER_WIDTH 1280
#define FRAME_GRABBER_HEIGHT 720

// line color detection
uint8_t line_val_min = 170;
uint8_t line_sat_max = 120;
size_t lineTransferPixelsMax = 0;
size_t lineFloorWindowSize = 0;
size_t lineFloorPixelsMin = 0;
size_t lineWindowSize = 0;
size_t linePixelsMin = 0;

// ball color detection
uint8_t ball_val_min = 0;
uint8_t ball_sat_min = 0;
uint8_t ball_hue_min = 0;
uint8_t ball_hue_max = 0;
uint16_t ballWindowSize = 0;
uint16_t ballPixelsMin = 0;
uint16_t ballFalsePixelsMax = 0;

// floor color detection
uint8_t floor_val_min = 0;
uint8_t floor_sat_min = 0;
uint8_t floor_hue_min = 0;
uint8_t floor_hue_max = 0;

// obstacle color detection
uint8_t obstacle_val_max = 60;
uint8_t obstacle_sat_max = 60;
uint32_t obstacleFloorWindowSize = 0;
uint32_t obstacleLineWindowSize = 0;
uint32_t obstacleBallWindowSize = 0;
uint32_t obstacleWindowSize = 0;
uint32_t obstacleTransferPixelsMax = 0;
uint32_t obstacleFloorPixelsMin = 0;
uint32_t obstacleLinePixelsMin = 0;
uint32_t obstacleBallPixelsMin = 0;
uint32_t obstaclePixelsMin = 0;

// cyan color detection
uint8_t cyan_val_min = 80;
uint8_t cyan_sat_min = 50;
uint8_t cyan_hue_min = 90 - 15; // cyan is 180 degrees
uint8_t cyan_hue_max = 90 + 15;

uint8_t redMult = 127;
uint8_t greenMult = 127;
uint8_t blueMult = 127;

typedef struct {
	uint16_t xBegin;
	uint16_t xEnd;
	uint16_t yBegin;
	uint16_t yEnd;
	uint16_t size;
} linePointSt;

typedef enum detectionEnum {
	DET_DEFAULT = 0, DET_LINE = 1, DET_BALL = 2, DET_OBSTACLE = 4, DET_CYAN = 8, DET_FLOOR = 16, DET_ZZ_LAST = 128
} detectionT;

typedef enum lineDetSetateEnum {
	LINE_DET_INIT = 0,
	LINE_DET_FLOOR_BEFORE,
	LINE_DET_TRANSFER_BEFORE,
	LINE_DET_ON_LINE,
	LINE_DET_TRANSFER_AFTER,
	LINE_DET_FLOOR_AFTER
} lineDetStateT;

typedef enum ballDetSetateEnum {
	BALL_DET_SEARCH = 0, BALL_DET_FOUND, BALL_DET_GOOD_ENOUGH
} ballDetStateT;

typedef enum obstacleDetSetateEnum {
	OBST_DET_INIT = 0, OBST_DET_OBSTACLE_PENDING, OBST_DET_OBSTACLE_FINALIZE, OBST_DET_TRANSFER,
} obstacleDetStateT;

#define GRAB_FLOAT
// #define DEBUG

// the FLOOR_WIDTH is the depth of the field which is used for floor, obstacle and line point detection
// #define FLOOR_WIDTH ( ROI_WIDTH / 2 )
#define FLOOR_WIDTH 400

// global variables
uint32_t buffer[FRAME_GRABBER_WIDTH * FRAME_GRABBER_HEIGHT];
uint8_t send_image_buffer[3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT];
bool send_image_buffer_done;
vector<linePointSt> foundPoints;
vector<linePointSt> floorPoints;
vector<linePointSt> ballPoints;
vector<linePointSt> obstaclePoints;
vector<linePointSt> filtered;
packetT txBuffer;
struct sockaddr_in toAddr;
int fd;
int camIndex;
int i2c_file;
zynqGrabConfig *config;

uint8_t testPatternPrev;
uint8_t analogGainPrev;
uint16_t shutterPrev;
uint16_t linesPrev;
uint16_t pixelsPrev;
uint16_t xStartPrev;
uint16_t xEndPrev;
uint16_t xSizePrev;
uint16_t yStartPrev;
uint16_t yEndPrev;
uint16_t ySizePrev;

uint32_t EXCK_FREQ_PREV;
uint32_t VTPXCK_DIV_PREV;
uint32_t VTSYCK_DIV_PREV;
uint32_t PREPLLCK_VT_DIV_PREV;
uint32_t PREPLLCK_OP_DIV_PREV;
uint32_t PLL_VT_MPY_PREV;
uint32_t OPPXCK_DIV_PREV;
uint32_t OPSYCK_DIV_PREV;
uint32_t PLL_OP_MPY_PREV;

inline void bgrToHsvFloat(const uint32_t data, uint8_t &hue, uint8_t &saturation, uint8_t &value) {

	// from https://docs.opencv.org/master/de/d25/imgproc_color_conversions.html#color_convert_rgb_hsv

#ifndef NONO
	// scale blue green and red from 0.0 to 1.0
	float blue = ((float) (data & 0x00ff0000)) / (65536 * 255);
	float green = ((float) (data & 0x0000ff00)) / (256 * 255);
	float red = ((float) (data & 0x000000ff)) / 255;

	blue = (blue * (256.0 + blueMult - 127)) / 256.0;
	if (blue > 1.0) {
		blue = 1.0;
	}
	green = (green * (256.0 + greenMult - 127)) / 256.0;
	if (green > 1.0) {
		green = 1.0;
	}

#else
//	uint8_t *val;
//	val = (uint8_t *) &data;

	float bl = (float) (data & 0x00ff0000);
	float gr = (float) (data & 0x0000ff00);
	float rd = (float) (data & 0xff);
//	val++;
//	float bl = (float) *val;
//	val++;
//	float gr = (float) *val;
//	val++;
//	float rd = (float) *val;
//	val++;

//	float32x4_t v1 = { (float) *val++, (float) *val++, (float) *val++, 0 };

	float32x4_t v1 = {bl, gr, rd, 0};
	float bluetmp = (256.0 + blueMult - 127) / 256.0;
	float greentmp = (256.0 + greenMult - 127) / 256.0;
	float32x4_t v2 = {bluetmp / (65536.0 * 255.0), greentmp / (256.0 * 255.0), 1.0 / 255.0, 0};

	float32x4_t prod = vmulq_f32(v1, v2);

	float blue = prod[0];
	float green = prod[1];
	float red = prod[2];

	if (blue > 1.0) {
		blue = 1.0;
	}
	if (green > 1.0) {
		green = 1.0;
	}

#endif

	// determine the maximal value of blue green and red
	float vmax = red;
	uint8_t vmax_type = 0; // red
	if (green > vmax) {
		vmax = green;
		vmax_type = 1; // green
	}
	if (blue > vmax) {
		vmax = blue;
		vmax_type = 2; // blue
	}

	if (vmax > 1.0) {
		printf("ERROR   : cam %d vmax is %f but should not exceed 1.0\n", camIndex, vmax);
	}
	value = (uint8_t) (vmax * 255);

	// deterime the minimal value of blue green and red
	float vmin = red;
	if (blue < vmin) {
		vmin = blue;
	}
	if (green < vmin) {
		vmin = green;
	}

	float vdelta = vmax - vmin;

	float s;
	if (vmax == 0) {
		s = 0;
	} else {
		s = vdelta / vmax;
	}
	s = s * 255;
	if (s > 255) {
		printf("ERROR   : cam %d saturation of %f should not happen\n", camIndex, s);
		saturation = 255;
	} else {
		saturation = (uint8_t) s;
	}

	// 0 to 360 degrees is represented as 0 to 180 degrees*2 so it fits in uint8_t
	float h;
	if (vdelta == 0) {
		h = 0;
	} else if (vmax_type == 0) {
		h = 30 * (green - blue) / vdelta; // if Vmax = R,  -60 to 60 degrees -> -30 to 30 degrees*2
	} else if (vmax_type == 1) {
		h = 60 + 30 * (blue - red) / vdelta; // if Vmax = G,  60 to 180 degrees -> 30 to 90 degrees*2
	} else {
		h = 120 + 30 * (red - green) / vdelta; // if Vmax = B, ( 180 to 300 degrees -> 80 to 150 degrees*2
	}
	if (h < 0) {
		h += 180;
	}
	hue = (uint8_t) h;
}

inline void bgrToHsvInt32(const uint32_t data, uint8_t &hue, uint8_t &saturation, uint8_t &value) {

	uint32_t blue = (data & 0x00ff0000) >> 16;
	uint32_t green = (data & 0x0000ff00) >> 8;
	uint32_t red = (data & 0x000000ff) >> 0;

	// todo combine with above shift and "and"
	blue = (blue * (256 + blueMult - 127)) / 256;
	if (blue > 255) {
		blue = 255;
	}
	green = (green * (256 + greenMult - 127)) / 256;
	if (green > 255) {
		green = 255;
	}
	// red = (red * (256 + redMult - 127)) / 256;
	//if (red > 255) {
	//	red = 255;
	//}

// determine maximal value for red, green and blue
	uint32_t vmax = red;
	uint8_t vmax_type = 0;
	if (green > vmax) {
		vmax = green;
		vmax_type = 1; // green
	}
	if (blue > vmax) {
		vmax = blue;
		vmax_type = 2; // blue
	}

// determine minimal value for red, green and blue
	uint32_t vmin = red;
	if (blue < vmin) {
		vmin = blue;
	}
	if (green < vmin) {
		vmin = green;
	}

// determine the maximal color difference
	uint32_t vdelta = vmax - vmin;

// determine saturation
	uint32_t s;
	if (vmax == 0) {
		s = 0;
	} else {
		s = (255 * vdelta) / vmax;
	}

// determine hue
// 0 to 360 degrees is represented as 0 to 180 degrees*2 so it fits in uint8_t
	int h;
	if (vdelta == 0) {
		h = 0;
	} else if (vmax_type == 0) {
		// cast to int32_t because green - blue can be negative and negative / uint32_t will cast the negative value to a +429496xxx
		h = (30 * ((int32_t) green - (int32_t) blue)) / (int32_t) vdelta; // if Vmax = R,  -60 to 60 degrees -> -30 to 30 degrees*2
	} else if (vmax_type == 1) {
		// green
		h = 60 + (30 * ((int32_t) blue - (int32_t) red)) / (int32_t) vdelta; // if Vmax = G,  60 to 180 degrees -> 30 to 90 degrees*2
	} else {
		// blue
		h = 120 + (30 * ((int32_t) red - (int32_t) green)) / (int32_t) vdelta; // if Vmax = B, ( 180 to 300 degrees -> 80 to 150 degrees*2
	}
	if (h < 0) {
		h += 180;
	}
	hue = (uint8_t) h;

	value = (uint8_t) vmax;
	saturation = (uint8_t) s;

}

inline void colorDetection(const uint8_t hue, const uint8_t saturation, const uint8_t value, detectionT &detection) {

	if ((value >= floor_val_min) && (saturation >= floor_sat_min) && (hue >= floor_hue_min) && (hue <= floor_hue_max)) {
		detection = DET_FLOOR;
	} else if ((value >= line_val_min) && (saturation <= line_sat_max)) {
		detection = DET_LINE;
	} else if ((value >= ball_val_min) && (saturation >= ball_sat_min) && (hue >= ball_hue_min)
			&& (hue <= ball_hue_max)) {
		detection = DET_BALL;
	} else if ((value <= obstacle_val_max) && (saturation <= obstacle_sat_max)) {
		detection = DET_OBSTACLE;
	} else if ((value >= cyan_val_min) && (saturation >= cyan_sat_min) && (hue >= cyan_hue_min)
			&& (hue <= cyan_hue_max)) {
		detection = DET_CYAN;
	} else {
		detection = DET_DEFAULT;
	}
}

// try to find a white line on the green floor and reject other white objects like the goal and goal net
// a line is white
// a line is located on the floor
// the floor is green (floor pixel)
// so a line is valid when first green, then white, and again green
// unfortunately there might be non white or non green pixels between the green and white transfer
// because of noise it is possible that not all pixels on the floor are marked is a floor (green)
// because of noise it is possible that not all pixels on the line are market as a line (white)
// so let's make the following assumption
// at least 5 out of 10 (green) floor pixels
// then maximal 10 transfer pixels of a non (green) floor or non (white) line pixel
// at least 2 of 6 (white) line pixels
// then maximal 10 transfer pixels of a non (green) floor or non (white) line pixel
// at least 5 of 10 (green) floor pixels
// when above condition is met,
// - send the start point of the white line and the end point of the white line
// - search for the next line
// Note: the exported "point" exists of 2 points (xBegin, XEnd) on the same line (y)
inline void lineDetection(const bool init, const detectionT detection, const uint16_t xx, const uint16_t yy) {
	static size_t floorPixels = 0;
	static size_t linePixels = 0;
	static size_t transferPixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static lineDetStateT state = LINE_DET_INIT;

	if (init) {
		// new line, start with new search
		state = LINE_DET_INIT;
		floorPixels = 0;
		linePixels = 0;
		transferPixels = 0;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if (detection == DET_FLOOR) {
		// increase the floor pixels and decrease the other pixel counters
		if (floorPixels < lineFloorWindowSize) {
			floorPixels++;
		}
		if (linePixels > 0) {
			linePixels--;
		}
		if (transferPixels > 0) {
			transferPixels--;
		}
	} else if (detection == DET_LINE) {
		// increase the line pixels and decrease the other pixel counters
		if (floorPixels > 0) {
			floorPixels--;
		}
		if (linePixels < lineWindowSize) {
			linePixels++;
		}
		if (transferPixels > 0) {
			transferPixels--;
		}
	} else {
		// if it is not a floor pixel or line pixel it is classified as transfer pixel
		// increase the transfer pixel and decrease the other pixel counters
		if (floorPixels > 0) {
			floorPixels--;
		}
		if (linePixels > 0) {
			linePixels--;
		}
		if (transferPixels < lineTransferPixelsMax) {
			transferPixels++;
		}
	}

	// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch (state) {
	case LINE_DET_INIT:
		// we are nowhere, restart the search
		if (detection == DET_FLOOR) {
			state = LINE_DET_FLOOR_BEFORE;
		}
		break;
	case LINE_DET_FLOOR_BEFORE:
		if (detection == DET_FLOOR) {
			// floor pixel, keep this state
		} else if (detection == DET_LINE) {
			// the line is only valid if there are enough floor pixels
			if (floorPixels >= lineFloorPixelsMin) {
				// enough floor pixels to change state to the line detection state
				state = LINE_DET_ON_LINE;
				xBegin = xx;
			} else {
				if (floorPixels == 0) {
					// not enough floor pixels detected, so this cannot be classified as a line, restart search
					state = LINE_DET_INIT;
				}
			}
		} else {
			// not a floor or line pixel, assume this is a transfer pixel
			if (floorPixels >= lineFloorPixelsMin) {
				// enough floor pixels to change state to the transfer state
				state = LINE_DET_TRANSFER_BEFORE;
			} else {
				if (floorPixels == 0) {
					// not enough floor pixels detected, so this cannot be classified as a transfer, restart search
					state = LINE_DET_INIT;
				}
			}
		}
		break;
	case LINE_DET_TRANSFER_BEFORE:
		if (detection == DET_FLOOR) {
			// it was likely a noise pixel, so we are still on the floor
			state = LINE_DET_FLOOR_BEFORE; // go back one state
		} else if (detection == DET_LINE) {
			// line pixels found, go to the line detection state
			state = LINE_DET_ON_LINE;
			xBegin = xx;
		} else {
			// no line or floor pixel detected
			if (transferPixels >= lineTransferPixelsMax) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			}
		}
		break;
	case LINE_DET_ON_LINE:
		if (detection == DET_FLOOR) {
			// a floor pixel detected, this can be a floor pixel before the line or a floor pixel after the line
			if (linePixels >= linePixelsMin) {
				state = LINE_DET_FLOOR_AFTER; // proceed two states
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				state = LINE_DET_FLOOR_BEFORE; // go back two states
			}
		} else if (detection == DET_LINE) {
			// line pixels, keep this state
		} else {
			// no line or floor pixel detected, this can be noise, the transfer area before the line or transfer area after the line
			if (linePixels == 0) {
				// not a lot of line pixels, go back to the transfer area before the line
				state = LINE_DET_TRANSFER_BEFORE; // go back one state
			} else if (linePixels >= linePixelsMin) {
				// enough line pixels, go to the transfer area after the line
				state = LINE_DET_TRANSFER_AFTER; // proceed one state
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				// likely a noise pixel, keep this state
			}
		}
		break;
	case LINE_DET_TRANSFER_AFTER:
		if (detection == DET_FLOOR) {
			// reached the floor after the line
			state = LINE_DET_FLOOR_AFTER; // proceed one state
		} else if (detection == DET_LINE) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back one state
		} else {
			// no line or floor pixel detected, check if the transfer area is not to wide
			if (transferPixels >= lineTransferPixelsMax) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			} else {
				// keep this state, and wait until we reach the floor or determine we are still on the line
			}
		}
		break;

		break;
	case LINE_DET_FLOOR_AFTER:
		if (detection == DET_FLOOR) {
			// floor pixel, check if we are done
			if (floorPixels >= lineFloorPixelsMin) {
				// store the line pixel
				if (xBegin > xEnd) {
					printf("ERROR   : cam %d x begin line pixel %u larger then x end line pixel %u for line %u\n",
							camIndex, xBegin, xEnd, yy);
				} else {
					linePointSt point;
					point.xBegin = xBegin;
					point.xEnd = xEnd;
					point.yBegin = yy;
					point.yEnd = yy;
					point.size = 0; // TODO remove from struct and transmission
					foundPoints.push_back(point);
				}

				state = LINE_DET_INIT; // restart search for next line
			} // else keep this state
		} else if (detection == DET_LINE) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back two states
		} else {
			// likely it was a noise pixel and we are in the transfer area
			state = LINE_DET_TRANSFER_AFTER; // go back two states
		}
		break;
	}
}

inline void storeFloorPoint(const uint16_t xBegin, const uint16_t xEnd, const uint16_t yy, const uint16_t size) {

	if (xBegin > xEnd) {
		printf("ERROR   : cam %d x begin pixel %u larger then x end pixel %u for floor point %u\n", camIndex, xBegin,
				xEnd, yy);
	} else {
		uint16_t length = xEnd - xBegin + 1;
		if ((size >= 5) && (length >= 10)) {
			// store floor point
			linePointSt point;
			point.xBegin = xBegin;
			point.xEnd = xEnd;
			point.yBegin = yy;
			point.yEnd = yy;
			point.size = size;
			floorPoints.push_back(point);
		}
	}
}

inline void floorDetection(const detectionT detection, const uint16_t xx, const uint16_t yy) {

	const uint16_t lost_max = 3;

	static bool foundPixel = false;
	static uint16_t size;
	static uint16_t xBegin;
	static uint16_t xEnd;
	static uint16_t lost;

	if (xx == 0) {
		// new line, start with new search
		foundPixel = false;
	}

	if (foundPixel) {
		// find out when we did not see a green pixel for a while
		if (detection == DET_FLOOR) {
			// so we still on the green floor
			lost = 0;
			size++;
			// update the last known position
			xEnd = xx;
		} else {
			if (lost > lost_max) {
				// so we did not see a green pixel for a while, so we are not on the green floor anymore
				// figure out if the collected data is worth storing
				storeFloorPoint(xBegin, xEnd, yy, size);
				foundPixel = false;
			}
			lost++; // we missed the green pixel
		}
	} else {
		// search for green pixel
		if (detection == DET_FLOOR) {
			foundPixel = true;
			// remember where the green pixels started
			// TODO: add to list
			xBegin = xx;
			xEnd = xx;
			// keep track how long ago we saw a green pixel (we do not want to bail out after the first noise pixel)
			lost = 0;
			size = 1;
		}

	}

	if ((xx == ( FLOOR_WIDTH - 1)) && (foundPixel)) {
		// finalize a found pixel in case we are at the end of a line
		storeFloorPoint(xBegin, xEnd, yy, size);
		foundPixel = false;
		// printf("INFO    : cam %d x begin pixel %u end pixel %u yy %u size %u for floor point\n", camIndex, xBegin, xEnd,
		//		yy, size);
	}
}

// the ball can be (partly) on the (green) floor, (white) line, before (black) obstacle or
// before an arbitrary color when flying through they air
// so for ball detection we cannot use any of the other colors to reject wrong ball pixels
// search for a (yellow) ball pixels and continue until the last (yellow) ball pixel has been found
// there might be pixels in the ball area that are not classified as (yellow) ball pixels
// so we need to combine ball pixels when there are some non ball pixels in between
inline void ballDetection(const detectionT detection, const uint16_t xx, const uint16_t yy) {
	static uint16_t ballPixels = 0;
	static uint16_t ballFalsePixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static ballDetStateT state = BALL_DET_SEARCH;

	if (xx == 0) {
		// new line, start with new search
		ballPixels = 0;
		ballFalsePixels = 0;
		state = BALL_DET_SEARCH;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if (detection == DET_BALL) {
		if (ballPixels < ballWindowSize) {
			ballPixels++;
		}
		if (ballFalsePixels > 0) {
			ballFalsePixels--;
		}
	} else {
		if (ballPixels > 0) {
			ballPixels--;
		}
		if (ballFalsePixels < ballFalsePixelsMax) {
			ballFalsePixels++;
		}
	}

	// first search for ball pixel, then keep on going until no ball pixel and finally send the ball pixels
	switch (state) {
	case BALL_DET_SEARCH:
		// we are nowhere, (re)start the search
		if (detection == DET_BALL) {
			state = BALL_DET_FOUND;
			xBegin = xx; // store the location where the ball is seen first
		}
		break;
	case BALL_DET_FOUND:
		if (detection == DET_BALL) {
			// still on the ball
			if (ballPixels >= ballPixelsMin) {
				// found enough ball pixels, now Classified as ball
				state = BALL_DET_GOOD_ENOUGH;
				xEnd = xx; // store the location where the ball is seen last
			}
		} else {
			// noise pixel, or noise on the ball, or lost the ball (beyond the ball)
			if (ballPixels == 0) {
				// not enough ball pixels, so this was likely was just a noise pixel
				state = BALL_DET_SEARCH;
			}
		}
		break;
	case BALL_DET_GOOD_ENOUGH:
		if (detection == DET_BALL) {
			// still on the ball
			xEnd = xx; // store the location where the ball is seen last
		} else {
			// noise on the ball, or lost the ball (beyond the ball)
			if (ballFalsePixels == ballFalsePixelsMax) {
				// not on the ball anymore, we are done with this ball
				linePointSt point;
				point.xBegin = xBegin;
				point.xEnd = xEnd;
				point.yBegin = yy;
				point.yEnd = yy;
				point.size = 0; // TODO remove from struct and transmission
				ballPoints.push_back(point);

				state = BALL_DET_SEARCH; // restart search for next ball
			} else {
				// wait for a number of non ball pixels (to be sure we are beyond the ball)
			}
		}
		break;
	}
}

// try to find obstacles on the floor
// obstacles are always in the floor
// so before the obstacle one of the following colors should be visible
//  - floor (green pixel)
//  - ball (yellow pixel)
//  - line ( white pixel)
// because the transfer from one of the above colors to the obstacle color (black) might
// not occur in directly, but there might be an intermediate color (transfer color)
// so make the following assumption
// at least 5 out of 10 floor pixels
// or at least 2 out of 4 ball pixels
// or at least 3 out of 6 line pixels
// then maximal 6 transfer pixels and then at least 30 obstacle pixels
// and then at least 20 out of 30 obstacle pixels
// then it is classified as obstacle
// because there might be noise in the obstacle pixels there might
// when above condition is met,
// - send the start point of the obstacle and the end point of the obstacle
// Note: the exported "point" exists of 2 points (xBegin, XEnd) on the same line (y)
// Note: for obstacles that are not black, but e.g. red, just decrease
// the thresholds of the obstacle filter (increase the maximal obstacle saturation and and value)
// NOte: it still will be difficult to distinguish between black borders and obstacles
inline void obstacleDetection(const detectionT detection, const uint16_t xx, const uint16_t yy) {
	static size_t floorPixels = 0;
	static size_t linePixels = 0;
	static size_t ballPixels = 0;
	static size_t obstaclePixels = 0;
	static size_t transferPixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static obstacleDetStateT state = OBST_DET_INIT;

	if (xx == 0) {
		// new line, start with new search
		state = OBST_DET_INIT;
		floorPixels = 0;
		linePixels = 0;
		ballPixels = 0;
		obstaclePixels = 0;
		transferPixels = 0;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if (detection == DET_FLOOR) {
		// increase the floor pixels
		if (floorPixels < obstacleFloorWindowSize) {
			floorPixels++;
		}
	} else {
		if (floorPixels > 0) {
			floorPixels--;
		}
	}

	if (detection == DET_LINE) {
		// increase the line pixels
		if (linePixels < obstacleLineWindowSize) {
			linePixels++;
		}
	} else {
		if (linePixels > 0) {
			linePixels--;
		}
	}

	if (detection == DET_BALL) {
		// increase the ball pixels
		if (ballPixels < obstacleBallWindowSize) {
			ballPixels++;
		}
	} else {
		if (ballPixels > 0) {
			ballPixels--;
		}
	}
	if (detection == DET_OBSTACLE) {
		// increase the obstacle pixels
		if (obstaclePixels < obstacleWindowSize) {
			obstaclePixels++;
		}
	} else {
		if (obstaclePixels > 0) {
			obstaclePixels--;
		}
	}

	if (detection == DET_DEFAULT) {
		// if it is not a floor pixel, line pixel, ball pixel or obstacle pixel it
		// is classified as transfer pixel
		// increase the transfer pixel
		if (transferPixels < obstacleTransferPixelsMax) {
			transferPixels++;
		}
	} else {
		if (transferPixels > 0) {
			transferPixels--;
		}
	}

	// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch (state) {
	case OBST_DET_INIT:
		if (detection == DET_OBSTACLE) {
			// found an obstacle pixel, first check enough floor, line or ball pixels have been seen
			if ((floorPixels >= obstacleFloorPixelsMin) || (linePixels >= obstacleLinePixelsMin)
					|| (ballPixels >= obstacleBallPixelsMin)) {
				// we are on the floor, line or ball, so now we could expect an obstacle
				state = OBST_DET_OBSTACLE_PENDING;
				xBegin = xx; // keep track of the first found obstacle pixel
			}
		} else if (detection == DET_DEFAULT) {
			// no valid color, but this might be intermediate color between a known color and obstacle
			if ((floorPixels >= obstacleFloorPixelsMin) || (linePixels >= obstacleLinePixelsMin)
					|| (ballPixels >= obstacleBallPixelsMin)) {
				state = OBST_DET_TRANSFER;
			}
		}
		break;
	case OBST_DET_TRANSFER:
		// we are in the transfer zone, this zone should not be to long, otherwise we have to go back to the init state
		if (detection == DET_OBSTACLE) {
			// we just got lucky and straight found a obstacle, now go to the obstacle search state
			state = OBST_DET_OBSTACLE_PENDING;
			xBegin = xx; // keep track of the first found obstacle pixel
		} else {
			// still waiting for a valid obstacle pixel, which we need to jump to the obstacle search state
			if ((transferPixels == 0) || (transferPixels >= obstacleTransferPixelsMax)) {
				// to bad, no transfer pixel for a while or to many transfer pixels, in both cases start all over again
				state = OBST_DET_INIT;
			}
		}
		break;
	case OBST_DET_OBSTACLE_PENDING:
		// determine if we have enough obstacle pixels, to be able to classify as obstacle
		if (detection == DET_OBSTACLE) {
			if (obstaclePixels >= obstaclePixelsMin) {
				// printf( "obst pixels %3d obstaclePixelsMin %3d, obstaclePixels window %3d\n", obstaclePixels, obstaclePixelsMin,obstacleWindowSize );
				// found enough pixels, classify as obstacle
				state = OBST_DET_OBSTACLE_FINALIZE;
				xEnd = xx; // keep track of the last found obstacle pixel
			}
		} else {
			// no obstacle pixel anymore
			if (obstaclePixels == 0) {
				// did not see an obstacle pixel for a long while, start all over again
				state = OBST_DET_INIT;
			}
		}
		break;
	case OBST_DET_OBSTACLE_FINALIZE:
		// enough pixels to qualify as obstacle, now search for the end
		if (detection == DET_OBSTACLE) {
			// still recognized as obstacle, keep state
			xEnd = xx; // keep track of the last found obstacle pixel
		} else {
			// did not see an obstacle pixel for a while, so we left the obstacle, restart the search
			if (obstaclePixels == 0) {
				state = OBST_DET_INIT;
			}
		}

		if ((xx == ( FLOOR_WIDTH - 1)) || (obstaclePixels == 0)) {
			// reached the end of the line or did not see an obstacle for a while
			// now we also know for sure what the xEnd pixel was, store the pixel for transmission
			linePointSt point;
			point.xBegin = xBegin;
			point.xEnd = xEnd;
			point.yBegin = yy;
			point.yEnd = yy;
			point.size = 0; // TODO remove from struct and transmission
			obstaclePoints.push_back(point);
		}
		break;
	}
}

inline void writeToBuffer(ssize_t index, uint32_t data) {
	if ((index < 0) || (index > (ssize_t) sizeof(buffer))) {
		printf("ERROR   : cam %d buffer index of %zd out of range\n", camIndex, index);
	} else {
		buffer[index] = data; // heu, saturation and value
	}
}

inline void filterLinePoints() {
	filtered.clear();
// printf("random: ");
	uint32_t found = foundPoints.size();
	uint32_t maxAmount = 100;
	if (maxAmount > found) {
		maxAmount = found;
	}
	for (uint32_t ii = 0; ii < maxAmount; ii++) {
		int index = rand() % found;
		// printf(" %d", index);
		filtered.push_back(foundPoints.at(index));
	}
}

inline void printPixelValue(bool direction) {
	if (direction) {
		printf("INFO    : cam %d out ", camIndex);
	} else {
		printf("INFO    : cam %d in  ", camIndex);
	}
	for (int jj = ROI_WIDTH * 100 + 700; jj < ROI_WIDTH * 100 + 700 + 4; jj++) {
		printf(" 0x%08x", buffer[jj]); // prints a series of bytes
	}
	printf("\n");
}

// send the floor line points
inline void sendLinePoints() {
// one line point is : xBegin, xEnd, yBegin, yEnd and size (which are 2 bytes each)
	if ((sizeof(linePointSt) * filtered.size() + HEADER_SIZE) > sizeof(txBuffer)) {
		printf("ERROR   : cam %d send line points of %zu bytes does not fit in tx buffer of %zu bytes\n", camIndex,
				sizeof(linePointSt) * filtered.size() + HEADER_SIZE, sizeof(txBuffer));
		exit(EXIT_FAILURE);
	}

	for (size_t ii = 0; ii < filtered.size(); ii++) {
		linePointSt point = filtered.at(ii);
		txBuffer.pl.u16[5 * ii] = point.xBegin;
		txBuffer.pl.u16[5 * ii + 1] = point.xEnd;
		txBuffer.pl.u16[5 * ii + 2] = point.yBegin;
		txBuffer.pl.u16[5 * ii + 3] = point.yEnd;
		txBuffer.pl.u16[5 * ii + 4] = point.size;
	}

	txBuffer.id = camIndex * 16 + 2;
	txBuffer.cnt++; // wrap around at 255
	txBuffer.size = HEADER_SIZE + sizeof(linePointSt) * filtered.size();

	ssize_t actualSize = sendto(fd, &txBuffer, txBuffer.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR   : cam %d cannot send message, message %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	} else if (actualSize != txBuffer.size) {
		printf("ERROR   : cam %d send on %zd instead of %u bytes of the line points\n", camIndex, actualSize,
				txBuffer.size);
	}

// printf("INFO    : send %zd bytes (%d points) to %s port %u\n", actualSize, (actualSize - HEADER_SIZE)/4,
//	inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));
}

// send the ball points
inline void sendBallPoints() {
	// one ball point is : xBegin, xEnd, y and size (which are 2 bytes each)
	if ((sizeof(linePointSt) * ballPoints.size() + HEADER_SIZE) > sizeof(txBuffer)) {
		printf("ERROR   : cam %d send ball points of %zu bytes does not fit in tx buffer of %zu bytes\n", camIndex,
				sizeof(linePointSt) * ballPoints.size() + HEADER_SIZE, sizeof(txBuffer));
		exit(EXIT_FAILURE);
	}

	for (size_t ii = 0; ii < ballPoints.size(); ii++) {
		linePointSt point = ballPoints.at(ii);
		txBuffer.pl.u16[5 * ii] = point.xBegin;
		txBuffer.pl.u16[5 * ii + 1] = point.xEnd;
		txBuffer.pl.u16[5 * ii + 2] = point.yBegin;
		txBuffer.pl.u16[5 * ii + 3] = point.yEnd;
		txBuffer.pl.u16[5 * ii + 4] = point.size;
	}

	txBuffer.id = camIndex * 16 + 3;
	txBuffer.cnt++; // wrap around at 255
	txBuffer.size = HEADER_SIZE + sizeof(linePointSt) * ballPoints.size();

	if (txBuffer.size > 1000) {
		// printf("INFO    : cam %d ############## buffer size %u ##########\n", camIndex, txBuffer.size);
	}
	ssize_t actualSize = sendto(fd, &txBuffer, txBuffer.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR   : cam %d cannot send message when sending ball points, message %s\n", camIndex,
				strerror(errno));
		exit(EXIT_FAILURE);
	} else if (actualSize != txBuffer.size) {
		printf("ERROR   : cam %d send on %zd instead of %u bytes of the ball points\n", camIndex, actualSize,
				txBuffer.size);
	}
}

// send the floor points
inline void sendFloorPoints() {
// one floor point is : xBegin, xEnd, y and size (which are 2 bytes each)
	if ((sizeof(linePointSt) * floorPoints.size() + HEADER_SIZE) > sizeof(txBuffer)) {
		printf("ERROR   : cam %d send floor points of %zu bytes does not fit in tx buffer of %zu bytes\n", camIndex,
				sizeof(linePointSt) * floorPoints.size() + HEADER_SIZE, sizeof(txBuffer));
		exit(EXIT_FAILURE);
	}

	for (size_t ii = 0; ii < floorPoints.size(); ii++) {
		linePointSt point = floorPoints.at(ii);
		txBuffer.pl.u16[5 * ii] = point.xBegin;
		txBuffer.pl.u16[5 * ii + 1] = point.xEnd;
		txBuffer.pl.u16[5 * ii + 2] = point.yBegin;
		txBuffer.pl.u16[5 * ii + 3] = point.yEnd;
		txBuffer.pl.u16[5 * ii + 4] = point.size;
	}

	txBuffer.id = camIndex * 16 + 5;
	txBuffer.cnt++; // wrap around at 255
	txBuffer.size = HEADER_SIZE + sizeof(linePointSt) * floorPoints.size();

	ssize_t actualSize = sendto(fd, &txBuffer, txBuffer.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR   : cam %d cannot send message when sending floor points, message %s\n", camIndex,
				strerror(errno));
		exit(EXIT_FAILURE);
	} else if (actualSize != txBuffer.size) {
		printf("ERROR   : cam %d send on %zd instead of %u bytes of the floor points\n", camIndex, actualSize,
				txBuffer.size);
	}
}

// send the obstacle points
inline void sendObstaclePoints() {
// one obstacle point is : xBegin, xEnd, y and size (which are 2 bytes each)
	if ((sizeof(linePointSt) * obstaclePoints.size() + HEADER_SIZE) > sizeof(txBuffer)) {
		printf("ERROR   : cam %d send obstacle points of %zu bytes does not fit in tx buffer of %zu bytes\n", camIndex,
				sizeof(linePointSt) * obstaclePoints.size() + HEADER_SIZE, sizeof(txBuffer));
		exit(EXIT_FAILURE);
	}

	for (size_t ii = 0; ii < obstaclePoints.size(); ii++) {
		linePointSt point = obstaclePoints.at(ii);
		txBuffer.pl.u16[5 * ii] = point.xBegin;
		txBuffer.pl.u16[5 * ii + 1] = point.xEnd;
		txBuffer.pl.u16[5 * ii + 2] = point.yBegin;
		txBuffer.pl.u16[5 * ii + 3] = point.yEnd;
		txBuffer.pl.u16[5 * ii + 4] = point.size;
	}

	txBuffer.id = camIndex * 16 + 4;
	txBuffer.cnt++; // wrap around at 255
	txBuffer.size = HEADER_SIZE + sizeof(linePointSt) * obstaclePoints.size();

	if (txBuffer.size > 1000) {
//		printf("INFO    : cam %d ############## required size %u buffer size %u object size %u ##########\n", camIndex, txBuffer.size,
		//			sizeof(txBuffer), sizeof(linePointSt));
	}

	ssize_t actualSize = sendto(fd, &txBuffer, txBuffer.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (actualSize < 0) {
		printf("ERROR   : cam %d cannot send message when sending obstacle points, message %s\n", camIndex,
				strerror(errno));
		exit(EXIT_FAILURE);
	} else if (actualSize != txBuffer.size) {
		printf("ERROR   : cam %d send on %zd instead of %u bytes of the obstacle points\n", camIndex, actualSize,
				txBuffer.size);
	}
}

// send a scaled down image to the CPU box
inline void sendImage(const uint32_t size) {
// size in amount of bytes
// we send the image in 4 quadrants to the CPU box
	if (((size / 4) + HEADER_SIZE) > sizeof(txBuffer)) {
		printf("ERROR   : cam %d send image of %u bytes does not fit in tx buffer of %zu bytes\n", camIndex,
				size / 4 + HEADER_SIZE, sizeof(txBuffer));
		exit(EXIT_FAILURE);
	}

	for (size_t quadrant = 0; quadrant < 4; quadrant++) {
		memcpy(&txBuffer.pl, &send_image_buffer[quadrant * size / 4], size / 4);
		txBuffer.id = camIndex * 16 + 8 + quadrant;
		txBuffer.cnt++; // wrap around at 255
		txBuffer.size = HEADER_SIZE + size / 4; // one pixel is 4 bytes

		ssize_t actualSize = sendto(fd, &txBuffer, txBuffer.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
		if (actualSize < 0) {
			printf("ERROR   : cam %d cannot send message, message %s\n", camIndex, strerror(errno));
			exit(EXIT_FAILURE);
		}
		// printf("INFO    : send %zd bytes (%d points) to %s port %u\n", actualSize, (actualSize - HEADER_SIZE)/4,
		//	inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));
	}
}

unsigned char i2c_reg_read(int dev_file, unsigned char dev_addr, const unsigned short reg_addr) {
#if defined(__arm__)
	__u8 inbuf[2];
	__u8 outbuf[2];
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];

	/*
	 * In order to read a register, we first do a "dummy write" by writing
	 * 0 bytes to the register we want to read from.  This is similar to
	 * the packet in set_i2c_register, except it's 1 byte rather than 2.
	 */
	outbuf[0] = reg_addr >> 8;
	outbuf[1] = reg_addr & 0xFF;
	messages[0].addr = dev_addr;
	messages[0].flags = 0;
	messages[0].len = 2,			//sizeof(outbuf);
	messages[0].buf = outbuf;
	/* The data will get returned in this structure */
	messages[1].addr = dev_addr;
	messages[1].flags = I2C_M_RD; /* | I2C_M_NOSTART*/
	messages[1].len = 1,			//sizeof(inbuf);
	messages[1].buf = inbuf;

	/* Send the request to the kernel and get the result back */
	packets.msgs = messages;
	packets.nmsgs = 2;
	if (ioctl(dev_file, I2C_RDWR, &packets) < 0) {
		printf("ERROR   : cam %d unable to read i2c, message %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}
	return inbuf[0];
#else
	// prevent message about unused variables by returning as fake data
	return dev_file | dev_addr | reg_addr;
#endif
}

int i2c_reg_write(int dev_file, unsigned char dev_addr, unsigned short reg_addr, unsigned char reg_data) {
#if defined(__arm__)
	unsigned char outbuf[3];
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[1];

	messages[0].addr = dev_addr;
	messages[0].flags = 0;
	messages[0].len = sizeof(outbuf);
	messages[0].buf = outbuf;
	/* The first byte indicates which register we'll write */
	outbuf[0] = reg_addr >> 8;
	outbuf[1] = reg_addr & 0xFF;
	outbuf[2] = reg_data;
	/* Transfer the i2c packets to the kernel and verify it worked */
	packets.msgs = messages;
	packets.nmsgs = 1;
	if (ioctl(dev_file, I2C_RDWR, &packets) < 0) {
		printf("ERROR   : cam %d unable to write i2c, message %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

// unsigned char readData = i2c_reg_read( dev_file, dev_addr, reg_addr );
// printf("INFO    : cam %d i2c 0x%04x : write 0x%02x read 0x%02x\n", camIndex, reg_addr, reg_data, readData);
	return 0;
#else
	// prevent message about unused variables by returning as fake data
	return dev_file | dev_addr | reg_addr | reg_data;
#endif
}

inline void cameraConfigure(bool verbose) {
	uint8_t testPattern = config->getTestPattern();
	if (testPattern != testPatternPrev) {
		if (verbose) {
			printf("INFO    : cam %d set test pattern 0x0601 to 0x%02x (%d)\n", camIndex, testPattern, testPattern);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0601, testPattern);
		testPatternPrev = testPattern;
	}
	uint8_t analogGain = config->getAnalogGain();
	if (analogGain != analogGainPrev) {
		if (verbose) {
			printf("INFO    : cam %d set analog gain 0x0157 to 0x%02x (%d)\n", camIndex, analogGain, analogGain);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0157, analogGain);
		analogGainPrev = analogGain;
	}
	uint16_t shutter = config->getShutter();
	if (shutter != shutterPrev) {
		if (verbose) {
			printf("INFO    : cam %d set shutter 0x015a to 0x%02x (%d), 0x015b to 0x%02x (%d)\n", camIndex,
					shutter >> 8, shutter >> 8, shutter & 0xff, shutter & 0xff);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x015a, shutter >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x015b, shutter & 0xff);
		shutterPrev = shutter;
	}
	uint16_t lines = config->getLines();
	if (lines != linesPrev) {
		if (verbose) {
			printf("INFO    : cam %d set amount of lines 0x0160 to 0x%04x (%d)\n", camIndex, lines, lines);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0160, lines >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0161, lines & 0xff);
		linesPrev = lines;
	}
	uint16_t pixels = config->getPixels();
	if (pixels != pixelsPrev) {
		if (verbose) {
			printf("INFO    : cam %d set amount of pixels 0x0162 to 0x%04x (%d)\n", camIndex, pixels, pixels);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0162, pixels >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0163, pixels & 0xff);
		pixelsPrev = pixels;
	}
	uint16_t xStart = config->getXStart();
	if (xStart != xStartPrev) {
		if (verbose) {
			printf("INFO    : cam %d set x start position 0x0164 to 0x%04x (%d)\n", camIndex, xStart, xStart);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0164, xStart >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0165, xStart & 0xff);
		xStartPrev = xStart;
	}
	uint16_t xEnd = config->getXEnd();
	if (xEnd != xEndPrev) {
		if (verbose) {
			printf("INFO    : cam %d set x end position 0x0166 to 0x%04x (%d)\n", camIndex, xEnd, xEnd);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0166, xEnd >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0167, xEnd & 0xff);
		xEndPrev = xEnd;
	}
	uint16_t xSize = config->getXSize();
	if (xSize != xSizePrev) {
		if (verbose) {
			printf("INFO    : cam %d set x size 0x016c to 0x%04x (%d)\n", camIndex, xSize, xSize);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x016c, xSize >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x016d, xSize & 0xff);
		xSizePrev = xSize;
	}
	uint16_t yStart = config->getYStart();
	if (yStart != yStartPrev) {
		if (verbose) {
			printf("INFO    : cam %d set y start position 0x0168 to 0x%04x (%d)\n", camIndex, yStart, yStart);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0168, yStart >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0169, yStart & 0xff);
		yStartPrev = yStart;
	}
	uint16_t yEnd = config->getYEnd();
	if (yEnd != yEndPrev) {
		if (verbose) {
			printf("INFO    : cam %d set y end position 0x016a to 0x%04x (%d)\n", camIndex, yEnd, yEnd);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x016a, yEnd >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x016b, yEnd & 0xff);
		yEndPrev = yEnd;
	}
	uint16_t ySize = config->getYSize();
	if (ySize != ySizePrev) {
		if (verbose) {
			printf("INFO    : cam %d set y size 0x016e to 0x%04x (%d)\n", camIndex, ySize, ySize);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x016e, ySize >> 8);
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x016f, ySize & 0xff);
		ySizePrev = ySize;
	}

	uint16_t EXCK_FREQ = config->getEXCK_FREQ();
	if (EXCK_FREQ != EXCK_FREQ_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set EXCK_FREQ 0x012a to 0x%08x (%d)\n", camIndex, EXCK_FREQ, EXCK_FREQ);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x012a, EXCK_FREQ >> 8); // total 16 bits
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x012b, EXCK_FREQ & 0xff);
		EXCK_FREQ_PREV = EXCK_FREQ;
	}
	uint16_t VTPXCK_DIV = config->getVTPXCK_DIV();
	if (VTPXCK_DIV != VTPXCK_DIV_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set VTPXCK_DIV 0x0301 to 0x%08x (%d)\n", camIndex, VTPXCK_DIV, VTPXCK_DIV);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0301, VTPXCK_DIV); // 5 bits
		VTPXCK_DIV_PREV = VTPXCK_DIV;
	}
	uint16_t VTSYCK_DIV = config->getVTSYCK_DIV();
	if (VTSYCK_DIV != VTSYCK_DIV_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set VTSYCK_DIV 0x303 to 0x%08x (%d)\n", camIndex, VTSYCK_DIV, VTSYCK_DIV);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0303, VTSYCK_DIV); // 2 bits
		VTSYCK_DIV_PREV = VTSYCK_DIV;
	}
	uint16_t PREPLLCK_VT_DIV = config->getPREPLLCK_VT_DIV();
	if (PREPLLCK_VT_DIV != PREPLLCK_VT_DIV_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set PREPLLCK_VT_DIV 0x0304 to 0x%08x (%d)\n", camIndex, PREPLLCK_VT_DIV,
					PREPLLCK_VT_DIV);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0304, PREPLLCK_VT_DIV); // 8 bits
		PREPLLCK_VT_DIV_PREV = PREPLLCK_VT_DIV;
	}
	uint16_t PREPLLCK_OP_DIV = config->getPREPLLCK_OP_DIV();
	if (PREPLLCK_OP_DIV != PREPLLCK_OP_DIV_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set PREPLLCK_OP_DIV 0x0305 to 0x%08x (%d)\n", camIndex, PREPLLCK_OP_DIV,
					PREPLLCK_OP_DIV);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0305, PREPLLCK_OP_DIV); // 8 bits
		PREPLLCK_OP_DIV_PREV = PREPLLCK_OP_DIV;
	}
	uint16_t PLL_VT_MPY = config->getPLL_VT_MPY();
	if (PLL_VT_MPY != PLL_VT_MPY_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set PLL_VT_MPY 0x0306 to 0x%08x (%d)\n", camIndex, PLL_VT_MPY, PLL_VT_MPY);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0306, PLL_VT_MPY >> 8); // total 11 bits
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0307, PLL_VT_MPY & 0xff);
		PLL_VT_MPY_PREV = PLL_VT_MPY;
	}
	uint16_t OPPXCK_DIV = config->getOPPXCK_DIV();
	if (OPPXCK_DIV != OPPXCK_DIV_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set OPPXCK_DIV 0x0309 to 0x%08x (%d)\n", camIndex, OPPXCK_DIV, OPPXCK_DIV);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0309, OPPXCK_DIV); // 5 bits
		OPPXCK_DIV_PREV = OPPXCK_DIV;
	}
	uint16_t OPSYCK_DIV = config->getOPSYCK_DIV();
	if (OPSYCK_DIV != OPSYCK_DIV_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set OPSYCK_DIV 0x030b to 0x%08x (%d)\n", camIndex, OPSYCK_DIV, OPSYCK_DIV);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x030b, OPSYCK_DIV); // 2 bits
		OPSYCK_DIV_PREV = OPSYCK_DIV;
	}
	uint16_t PLL_OP_MPY = config->getPLL_OP_MPY();
	if (PLL_OP_MPY != PLL_OP_MPY_PREV) {
		if (verbose) {
			printf("INFO    : cam %d set PLL_OP_MPY 0x030c to 0x%08x (%d)\n", camIndex, PLL_OP_MPY, PLL_OP_MPY);
		}
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x030c, PLL_OP_MPY >> 8); // total 11 bits
		i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x030d, PLL_OP_MPY & 0xff);
		PLL_OP_MPY_PREV = PLL_OP_MPY;
	}
}

void cameraReset(bool verbose) {
// reset the "previous" value, so these values will be set to the correct value from the main loop
	testPatternPrev = 255;
	analogGainPrev = 255;
	shutterPrev = 0xffff;
	linesPrev = 0xffff;
	pixelsPrev = 0xffff;
	xStartPrev = 0xffff;
	xEndPrev = 0xffff;
	xSizePrev = 0xffff;
	yStartPrev = 0xffff;
	yEndPrev = 0xffff;
	ySizePrev = 0xffff;

	EXCK_FREQ_PREV = 0xffffffff;
	VTPXCK_DIV_PREV = 0xffffffff;
	VTSYCK_DIV_PREV = 0xffffffff;
	PREPLLCK_VT_DIV_PREV = 0xffffffff;
	PREPLLCK_OP_DIV_PREV = 0xffffffff;
	PLL_VT_MPY_PREV = 0xffffffff;
	OPPXCK_DIV_PREV = 0xffffffff;
	OPSYCK_DIV_PREV = 0xffffffff;
	PLL_OP_MPY_PREV = 0xffffffff;

// reset camera through software reset
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0103, 0x01); // software reset

// acquire access to the camera manufacturer specific registers of the camera (of which only the black level is used)
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x30eb, 0x05);
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x30eb, 0x0c);
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x300a, 0xff);
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x300b, 0xff);
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x30eb, 0x05);
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x30eb, 0x09);

	if (verbose) {
		// get the camera model, lot id, waver number and chip number (likely from camera NVM)
		int addr = 0;
		uint32_t model = i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr) << 8;
		addr = 1;
		model |= i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr);
		addr = 4;
		uint32_t lotId = i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr) << 16;
		addr = 5;
		lotId |= i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr) << 8;
		addr = 6;
		lotId |= i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr);
		addr = 7;
		uint32_t waferNum = i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr);
		addr = 0x0d;
		uint32_t chipNum = i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr) << 8;
		addr = 0x0e;
		chipNum |= i2c_reg_read(i2c_file, CAMERA_V2_1_IIC_ADDRESS, addr);
		printf("INFO    : cam %d model 0x%04x lot ID 0x%06x, wafer 0x%02x chip 0x%04x\n", camIndex, model, lotId,
				waferNum, chipNum);
	}

// configure the fixed parameters of the camera, the other parameters are set through the zynqGrabConfig
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0114, 0x01); // CSI_LANE_MODE 2 lanes
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0128, 0x00); // DPHY_CTRL : MIPI global timing setting auto mode
	cameraConfigure(false);
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x015a, 0x00); // course integration time (electronic shutter) [15:8]
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x015b, 0xe8); // course integration time (electronic shutter) [7:0]
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0174, 0x02); // BINNING_MODE_H_A [1:0] : H-direction x4-binning
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0175, 0x02); // BINNING_MODE_V_A [1:0] : V-direction x4-binning
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0176, 0x01); // BINNING_CAL_MODE_H_A [0] : H-direction sum mode binning (instead of average)
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0177, 0x01); // BINNING_CAL_MODE_V_A [0] : V-direction sum mode binning (instead of average)
// the PLL_VT_MPY and PLL_OP_MPY are also set through the zynqGrabConfig, but apparently they also need to be set at this location (likely before activation streaming mode)
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0306, 0x00); // PLL_VT_MPY [10:8] : PLL video timing system multiplier value
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0307, 0x17); // PLL_VT_MPY [7:0] : 0x0017 = 23, works in range 0x10 to 0x19
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x030c, 0x00); // PLL_OP_MPY [10:8] : PLL output system multiplier value
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x030d, 0x2e); // PLL_OP_MPY [7:0] : 0x002e = 46 ,works from 0x002a to 0x005c
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0xd1ea, 0x00); // DT_PEDESTAL [9:8] : black level
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0xd1eb, 0x10); // DT_PEDESTAL [7:0] : default is 0x40
	i2c_reg_write(i2c_file, CAMERA_V2_1_IIC_ADDRESS, 0x0100, 0x01); // streaming mode
}

int main(int argc, char** argv) {

	string ip = "10.0.0.1";
	string imageFile = "imageUndefined.bin";
	uint16_t port = 12345; // used for as well UDP as TCP
	camIndex = 0;
	int opt = 0;
	bool grab = false;
	bool fixed = false;
	uint32_t bufferPrev[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	while ((opt = getopt(argc, argv, "fgi:l:n:p:")) != -1) {
		switch (opt) {
		case 'f':
			fixed = true;
			break;
		case 'g':
			grab = true;
			break;
		case 'i':
			ip.assign(optarg);
			break;
		case 'l':
			camIndex = atoi(optarg);
			break;
		case 'n':
			imageFile.assign(optarg);
			break;
		case 'p':
			port = (uint16_t) atoi(optarg);
			break;
		}
	}
	config = new zynqGrabConfig();

// create a normal UDP socket
	fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		printf("ERROR   : cam %d cannot create UDP socket!, message %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

// fill in server_addr struct
	memset((char *) &toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // IP protocol family
	toAddr.sin_port = htons(port);
	if (inet_aton(ip.c_str(), &toAddr.sin_addr) == 0) {
		perror("ERROR   : inet_aton() failed, message");
		exit(EXIT_FAILURE);
	}
	printf("INFO    : cam %d setup UDP send connection, destination IP %s port %u\n", camIndex,
			inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));
	fflush(stdout);

	txBuffer.cnt = 0; // initialize at 0
// ## network send done ##

// ## network receive (server) ##
	tcp_socket *sock;
	sock = new tcp_socket(camIndex, config);
	sock->bind(port);

	thread sockThread = thread(&tcp_socket::serverUpdate, sock);

	send_image_buffer_done = true;

// ## end network receive (server) ##

	printf("INFO    : cam %d tx packet size %zu can hold up to points of linePoint types per packet %.0f\n", camIndex,
			sizeof(txBuffer), 1.0 * (sizeof(txBuffer) - HEADER_SIZE) / sizeof(linePointSt));

#if defined(__arm__)
// ## open i2c to camera ##
	if ((i2c_file = open("/dev/i2c-5", O_RDWR)) < 0) {
		printf("ERROR   : cam %d cannot open /dev/i2c-5, message %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

	for (size_t ii = 0; ii < 5; ii++) {
		cameraReset(false);
	}
	cameraReset(false);
#endif

	FILE *readPtr;
	if (fixed) {
		readPtr = fopen(imageFile.c_str(), "rb");  // r for read, b for binary
		if (readPtr == NULL) {
			printf("ERROR   : cam %d %s when opening file %s for reading\n", camIndex, strerror(errno),
					imageFile.c_str());
			exit( EXIT_FAILURE);
		} else {
			printf("INFO    : cam %d using file %s as camera input\n", camIndex, imageFile.c_str());
		}
	} else {
#if defined(__arm__)
		readPtr = fopen("/dev/fb1", "rb");  // r for read, b for binary
		if (readPtr == NULL) {
			printf("ERROR   : %s when opening /dev/fb1 for reading\n", strerror(errno));
			exit( EXIT_FAILURE);
		}
#endif
	}

#if defined(__arm__)
	FILE *hdmiPtr = fopen("/dev/fb0", "wb");  // w for read, b for binary
	if (hdmiPtr == NULL) {
		printf("ERROR   : %s when opening /dev/fb0 for writing\n", strerror(errno));
		exit( EXIT_FAILURE);
	}
#endif
	uint8_t hue, saturation, value;

	size_t read_size = 0;
	size_t forCount = 0xffffffff;
	if (grab) {
		forCount = 1;
	}

	for (size_t ii = 0; ii < forCount; ii++) {
		line_val_min = config->getLineValMin();
		line_sat_max = config->getLineSatMax();
		lineTransferPixelsMax = config->getLineTransferPixelsMax();
		lineFloorWindowSize = config->getLineFloorWindowSize();
		lineFloorPixelsMin = config->getLineFloorPixelsMin();
		lineWindowSize = config->getLineWindowSize();
		linePixelsMin = config->getLinePixelsMin();
		ball_val_min = config->getBallValMin();
		ball_sat_min = config->getBallSatMin();
		ball_hue_min = config->getBallHueMin();
		ball_hue_max = config->getBallHueMax();
		ballWindowSize = config->getBallWindowSize();
		ballPixelsMin = config->getBallPixelsMin();
		ballFalsePixelsMax = config->getBallFalsePixelsMax();
		floor_val_min = config->getFloorValMin();
		floor_sat_min = config->getFloorSatMin();
		floor_hue_min = config->getFloorHueMin();
		floor_hue_max = config->getFloorHueMax();
		obstacle_val_max = config->getObstacleValMax();
		obstacle_sat_max = config->getObstacleSatMax();
		obstacleFloorWindowSize = config->getObstacleFloorWindowSize();
		obstacleLineWindowSize = config->getObstacleLineWindowSize();
		obstacleBallWindowSize = config->getObstacleBallWindowSize();
		obstacleWindowSize = config->getObstacleWindowSize();
		obstacleTransferPixelsMax = config->getObstacleTransferPixelsMax();
		obstacleFloorPixelsMin = config->getObstacleFloorPixelsMin();
		obstacleLinePixelsMin = config->getObstacleLinePixelsMin();
		obstacleBallPixelsMin = config->getObstacleBallPixelsMin();
		obstaclePixelsMin = config->getObstaclePixelsMin();
		redMult = config->getRed();
		greenMult = config->getGreen();
		blueMult = config->getBlue();

#if defined(__arm__)
		if (config->getCameraReset()) {
			printf("INFO    : cam %d camera reset\n", camIndex);
			cameraReset(false);
		}
		cameraConfigure(false);
#endif

		read_size = fread(buffer, sizeof(buffer), 1, readPtr);
		if (read_size == 0) {
			printf("ERROR   : cam %d read error from /dev/fb1, message %s\n", camIndex, strerror(errno));
			exit(EXIT_FAILURE);
		}

		uint32_t *buffer_ptr = buffer;
// #define DRAW_WHITE_CENTER_CROSS
#ifdef DRAW_WHITE_CENTER_CROSS
		for (uint16_t yy = 0; yy < FRAME_GRABBER_HEIGHT; yy++) {
			// process one line
			for (uint16_t xx = 0; xx < FRAME_GRABBER_WIDTH; xx++) {
				// store modified data to buffer (overwrite read data)
				if (yy == ROI_HEIGHT / 2) {
					*buffer_ptr = 0xffffffff;
				}
				if (xx == 820 / 2) { // ROI_WIDTH has been increased so it can be divided by 8, so ROI_WIDTH/2 is not at the center
					*buffer_ptr = 0xffffffff;
				}
				if (xx == ( ROI_WIDTH - 1)) {
					buffer_ptr += FRAME_GRABBER_WIDTH - ROI_WIDTH + 1;
					xx = FRAME_GRABBER_WIDTH;
				} else {
					buffer_ptr++;
				}
			}
			if (yy == ( ROI_HEIGHT - 1)) {
				yy = FRAME_GRABBER_HEIGHT;
			}
		} // for all pixels
#endif

#if defined(__arm__)
		size_t hdmiSize = fwrite(buffer, sizeof(buffer), 1, hdmiPtr);
		if (hdmiSize == 0) {
			printf("ERROR   : cam %d write error to /dev/fb0, message %s\n", camIndex, strerror(errno));
			exit(EXIT_FAILURE);
		}
#endif
		// printPixelValue(false);

		uint8_t *send_image_buffer_ptr = send_image_buffer;
		foundPoints.clear(); // clear the list of line points
		floorPoints.clear(); // clear the list of floor points
		ballPoints.clear(); // clear the list of ball points
		obstaclePoints.clear(); // clear the list of obstacle points

		// process one frame 720*1280 = 921600 pixels
		// both CPUs are 32 bits @ 666MHz and let's assume one pixel is also 32 bits and we want to achieve 20fps
		// then 666/(921600*20) = 36 CPU cycles per pixel for one CPU

		// first scan in "lines" from top to bottom instead from left to right, to be able to find floor lines
		// parallel with the camera scan lines
		// this is a quite expensive operation because the read pointer is jumping through the memory instead
		// of a linear read
		// do this before the normal scanning so there is still room left in the line points buffer
		// the floor lines are typically at the center of the camera image, so we do not need to scan the full
		// vertical line
		// the effective field width is 400, so let's go for about 350, and jumps of 25 gives enough points
		for (uint16_t xx = 25; xx < 350; xx += 25) { // full range is 0:823
			// process one line from top to bottom
			// scan next vertical line
			buffer_ptr = buffer + xx; // reset pointer to pixel xx on the first line
			for (uint16_t yy = 0; yy < ROI_HEIGHT; yy++) { // 0:615 range
				if ((sizeof(linePointSt) * (foundPoints.size() + 1)) < MAX_PAYLOAD_SIZE) {
					hue = 0;
					saturation = 0;
					value = 0;

					bgrToHsvInt32(*buffer_ptr, hue, saturation, value);

					detectionT detection = DET_DEFAULT;
					colorDetection(hue, saturation, value, detection);
					lineDetection(yy == 0, detection, yy, xx); // swapped xx and yy to be able to re-use the lineDetection function, corrected after this loop
					buffer_ptr += FRAME_GRABBER_WIDTH; // + 1280
				}
			}
		}
		for (size_t ii = 0; ii < foundPoints.size(); ii++) {
			// swap xx and yy because we scanned in the other direction
			// Note: because we are scanning on the x-axis the x begin and x end are the same
			uint16_t tmp = foundPoints[ii].xBegin;
			foundPoints[ii].xBegin = foundPoints[ii].yBegin;
			foundPoints[ii].yBegin = tmp;
			tmp = foundPoints[ii].xEnd;
			foundPoints[ii].xEnd = foundPoints[ii].yEnd;
			foundPoints[ii].yEnd = tmp;
			int width = foundPoints[ii].yEnd - foundPoints[ii].yBegin;
			if (width < 0) {
				printf("ERROR    : cam %d y width is is %d but should be 0 or larger\n", camIndex, width);
			}
		}
#ifdef NONO
		if (foundPoints.size() != 0) {
			printf("INFO     : cam %d line points %zu, first xBegin %d xEnd %d yBegin %d yEnd %d\n", camIndex,
					foundPoints.size(), foundPoints[0].xBegin, foundPoints[0].xEnd, foundPoints[0].yBegin,
					foundPoints[0].yEnd);

		}
#endif

		// initialize to start processing new frame
		buffer_ptr = buffer;
		bool same = true;
#ifndef GRAB_STATIC_IMAGE
#define HALF_LINES
#ifdef HALF_LINES
		for (uint16_t yy = 0; yy < FRAME_GRABBER_HEIGHT; yy += 2) {
#else
			for (uint16_t yy = 0; yy < FRAME_GRABBER_HEIGHT; yy++) {
#endif
			// process one line
			for (uint16_t xx = 0; xx < FRAME_GRABBER_WIDTH; xx++) {
				// TODO: skip upper part of the field
				hue = 0;
				saturation = 0;
				value = 0;
				if (xx < FLOOR_WIDTH) { // ROI_WIDTH / 2 = 824 / 2 = 412
					// check if the data was updated compared with the previous line
					if (!fixed) {
						if (yy == 200) {
							if (xx == 50) {
								if (bufferPrev[0] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[0] = *buffer_ptr;
							} else if (xx == 150) {
								if (bufferPrev[1] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[1] = *buffer_ptr;
							} else if (xx == 250) {
								if (bufferPrev[2] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[2] = *buffer_ptr;
							} else if (xx == 350) {
								if (bufferPrev[3] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[3] = *buffer_ptr;
							} // if (xx ==
						} else if (yy == 400) {
							if (xx == 70) {
								if (bufferPrev[4] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[4] = *buffer_ptr;
							} else if (xx == 170) {
								if (bufferPrev[5] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[5] = *buffer_ptr;
							} else if (xx == 270) {
								if (bufferPrev[6] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[6] = *buffer_ptr;
							} else if (xx == 370) {
								if (bufferPrev[7] != *buffer_ptr) {
									same = false;
								}
								bufferPrev[7] = *buffer_ptr;
								if (same) {
									printf(
											"WARNING : cam %d same data 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x as previous frame\n",
											camIndex, bufferPrev[0], bufferPrev[1], bufferPrev[2], bufferPrev[3],
											bufferPrev[4], bufferPrev[5], bufferPrev[6], bufferPrev[7]);
								} // if (same ==
							} // if (xx ==
						} // if (yy ==
					} // if (fixed ==
					  // choose to use integer or floating point bgrToHsv conversion
					  // bgrToHsvFloat(*buffer_ptr, hue, saturation, value);
					bgrToHsvInt32(*buffer_ptr, hue, saturation, value);

					detectionT detection = DET_DEFAULT;
					colorDetection(hue, saturation, value, detection);
					// do not scan on every camera line for a field line to prevent that the ones scanned in the y direction
					// are suppressed (there is a limit on the amount of line points used by the solver)
					if ((yy % 16) == 8) {
						// line point detection, floor points and obstacles only on left part of image (= floor)
						// only continue searching for line points if there is at least space for one more line point in the send buffer
						// if ((sizeof(linePointSt) * (foundPoints.size() + 1) + HEADER_SIZE) < sizeof(txBuffer)) {
						if ((sizeof(linePointSt) * (foundPoints.size() + 1)) < MAX_PAYLOAD_SIZE) {
							lineDetection(xx == 0, detection, xx, yy);
						}
					}
					// only continue searching for floor points if there is at least space for one more floor point in the send buffer
					// if ((sizeof(linePointSt) * (floorPoints.size() + 1) + HEADER_SIZE) < sizeof(txBuffer)) {
					if ((sizeof(linePointSt) * (floorPoints.size() + 1)) < MAX_PAYLOAD_SIZE) {
						floorDetection(detection, xx, yy);
					}

					// only continue searching for obstacles if there is at least space for one more obstacle in the send buffer
					// if ((sizeof(linePointSt) * (obstaclePoints.size() + 1) + HEADER_SIZE) < sizeof(txBuffer)) {
					if ((sizeof(linePointSt) * (obstaclePoints.size() + 1)) < MAX_PAYLOAD_SIZE) {
						obstacleDetection(detection, xx, yy);
					}

					// only continue searching for balls if there is at least space for one more ball in the send buffer
					//if ((sizeof(linePointSt) * (ballPoints.size() + 1) + HEADER_SIZE) < sizeof(txBuffer)) {
					if ((sizeof(linePointSt) * (ballPoints.size() + 1)) < MAX_PAYLOAD_SIZE) {
						ballDetection(detection, xx, yy);
					}

					// store modified data to buffer (overwrite read data)
					*buffer_ptr = (detection << 24) | (hue << 16) | (saturation << 8) | value;
				}
				// create a resized HSV image that can be send to the CPU box
				// 760/4 = 190, 616/4 = 154, which is 29260 bytes
				if (send_image_buffer_done && ((yy % SEND_IMAGE_HEIGHT_FACTOR) == (SEND_IMAGE_HEIGHT_FACTOR / 2))
						&& ((xx % SEND_IMAGE_WIDTH_FACTOR) == (SEND_IMAGE_WIDTH_FACTOR / 2))) {
					*send_image_buffer_ptr++ = hue;
					*send_image_buffer_ptr++ = saturation;
					*send_image_buffer_ptr++ = value;
				}

				if (xx == ( ROI_WIDTH - 1)) {
					buffer_ptr += FRAME_GRABBER_WIDTH - ROI_WIDTH + 1;
#ifdef HALF_LINES
					buffer_ptr += FRAME_GRABBER_WIDTH;
#endif
					xx = FRAME_GRABBER_WIDTH;
				} else {
					buffer_ptr++;
				}
			}
#ifdef HALF_LINES
			if (yy >= ( ROI_HEIGHT - 2)) {
#else
				if (yy >= ( ROI_HEIGHT - 1)) {
#endif
				yy = FRAME_GRABBER_HEIGHT;
			}
		} // for all pixels
		send_image_buffer_done = false;

		sendBallPoints();
		sendFloorPoints();
		sendObstaclePoints();
		filterLinePoints();
		sendLinePoints();
		send_image_buffer_done = true;
		sendImage(3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT); // amount of bytes
		// sendMessage();

		// printPixelValue(true);

		if (fseek(readPtr, 0L, SEEK_SET) != 0) {
			printf("ERROR   : cam %d cannot set file pointer back for /dev/fb1, message %s\n", camIndex,
					strerror(errno));
			exit(EXIT_FAILURE);
		}

#if defined(__arm__)
		// TODO: checkout how we software can wait until new data is available from the camera (IRQ)
		usleep(60000);// 60 ms, if we read to faster, we read the same data as the previous frame

		// there is probably a better way to update write pointer, but for now this works
		if (fseek(hdmiPtr, 0L, SEEK_SET) != 0) {
			printf("ERROR   : cam %d cannot set file pointer back for /dev/fb0, message %s\n", camIndex,
					strerror(errno));
			exit(EXIT_FAILURE);
		}
#else
		usleep(100000); // 100ms latency to emulate nearly 10 FPS
#endif
		// TODO: checkout if fflush is not slowing down performance
		fflush(stdout);
#endif // ifndef GRAB_STATIC_IMAGE

	} // number of frames to process

	FILE *writePtr;
	writePtr = fopen("zynqGrab.bin", "wb"); // w for write, b for binary
	fwrite(buffer, sizeof(buffer), 1, writePtr); // write the buffer to file

// TODO: implement the correct way to stop the receive thread (used to change config of camera and filters)
#ifdef NONO
	printf("stop thread\n");
	fflush(stdout);

	sock->stop();
	printf("wait for join\n");
	fflush(stdout);

	std::terminate();
//   sockThread.join();
#endif

	fclose(writePtr);
	fclose(readPtr);
	close(fd);
#if defined(__arm__)
	fclose(hdmiPtr);
	close(i2c_file);
#endif

	printf("INFO    : cam %d all done\n", camIndex);

	return 0;
}
// main
