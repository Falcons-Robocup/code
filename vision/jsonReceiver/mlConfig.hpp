// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef ML_CONFIG_HPP
#define ML_CONFIG_HPP

#include <cstdio>

struct mlObjectSt {
    float azimuth; // -M_PI/2..M_PI/2 (-90..90 degrees), by definition 0 degrees is nort and 90 degrees is east
	size_t classId; // 0..4
	float confidence; // 0..1 (range does not include 1.0 itself)
	float elevation; // -M_PI..M_PI
	float height; // 0..1 (range does not include 1.0 itself)
	float radius; // 0..MAX_FLT in meters
	bool valid;
	float width; // 0..1 (range does not include 1.0 itself)
	float xCenter; // 0..1, 0 = left (range does not include 1.0 itself)
	float yCenter; // 0..1, 0 = top (range does not include 1.0 itself)
};

// WARNING: the cam width and heigth shall match the size of the dewarp lookup table
// hexdump /home/robocup/falcons/data/internal/vision/multiCam/calibration/20200627_r4_assyF_cam0.bin -n 8 -x
// 0000000    0320    0000    0260    0000
// 0x320 = 800
// 0x260 = 608
// genius f100 usb webcam cropped from 1280x720 to 960 to 720
// hexdump /home/robocup/falcons/data/internal/vision/multiCam/calibration/20210625_155231_r10_cam0.bin -n 8 -x
// 0000000    03c0    0000    02d0    0000
// 0x3c0 = 960
// 0x2d0 = 720
// hexdump /home/robocup/falcons/data/internal/vision/multiCam/calibration/20210625_181243_r10_cam0.bin -n 8 -x
// 0000000    0260    0000    0320    0000
// 0x260 = 608
// 0x320 = 800

#define CAM_WIDTH 608
#define CAM_HEIGHT 800
// cropped genius f100 usb webcam, note: 960 is barely covers 90 degrees
// WARNING: the genius f100 is not rotated, so width and height should be swapped
// UPDATE: resize genius f100 images to the same size as raspi image size
// #define CAM_WIDTH 960
// #define CAM_HEIGHT 720
// don't why i defined the dewarp width and height, but apparently they are not used
// #define DEWARP_WIDTH 977
// #define DEWARP_HEIGHT 701

#define CLASS_ID_MAX 4
#define RADIUS_MAX 20.0 // everything beyond 20.0 meters is irrelevant

#define ROBOT_INDEX_LAST 10

// colors: blue, green, red
#define BALL_COLOR cv::Scalar( 0, 255, 255 )
#define HUMAN_COLOR cv::Scalar( 0, 128, 255 )
#define OBSTACLE_COLOR cv::Scalar ( 255, 0, 255 )
#define OUTSIDE_COLOR cv::Scalar ( 255, 0, 0 )
#define GOALPOST_COLOR cv::Scalar ( 255, 255, 255 );

// frame counter should not exceed 2 days @ 40 FPS (typically it only runs a few hours)
#define FRAME_ID_MAX 2*24*60*60*40

#endif // ML_CONFIG_HPP
