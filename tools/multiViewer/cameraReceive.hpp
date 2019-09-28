// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERA_RECEIVE_HPP
#define CAMERA_RECEIVE_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <thread>

#include <arpa/inet.h>
#include <mutex>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// resolution is 820x616, but 820 does not divide by 8
#define ROI_WIDTH 824
#define ROI_HEIGHT 616

#define SEND_IMAGE_WIDTH_FACTOR 4
#define SEND_IMAGE_HEIGHT_FACTOR 4

#define SEND_IMAGE_WIDTH (ROI_WIDTH/SEND_IMAGE_WIDTH_FACTOR)
#define SEND_IMAGE_HEIGHT (ROI_HEIGHT/SEND_IMAGE_HEIGHT_FACTOR)

typedef struct {
	uint16_t x;
	uint16_t y;
} camLinePointT;

// maximal camera packet size 3*160*90 = 43200 bytes
typedef struct {
	uint8_t id;
	uint8_t cnt;
	uint16_t size; // camera packet size including header in bytes
	union {
		// 2^16 = 64 KiB
		uint8_t u8[1 << 16]; // unsigned payload
		uint16_t u16[1 << 15];
		uint32_t u32[1 << 14];
		uint64_t u64[1 << 13];
		int8_t s8[1 << 16]; // signed payload
		int16_t s16[1 << 15];
		int32_t s32[1 << 14];
		int64_t s64[1 << 13];
		float f32[1 << 14];
		double d64[1 << 13];
	} pl;
}__attribute__((packed)) camPacketT;

#define CAM_HEADER_SIZE 4

#ifdef NONO
enum camIdList {
	invalid=0,
	cam0LinePoint=2,
	cam0Image0=3,
	cam0Image1,
	cam0Image2,
	cam0Image3,
	cam1LinePoint,
	cam2LinePoint,
	cam3LinePoint,
	cam1Image,
	cam2Image,
	cam3Image
};
#endif

class cameraReceive {
private:
	int fd;
	camPacketT rxPacket;
	uint8_t rxPacketCntExpected[4];
	bool rxPacketCntFirstCheck[4];
	std::vector<camLinePointT> linePointList;
	std::mutex linePointListExportMutex, cameraFrameExportMutex;
	bool waitForNew;
	bool newCameraFrameReceived;
	bool newLinePointsReceived;
	cv::Mat cameraFrame;

	uint8_t receive_image_buffer[3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT];


public:
	cameraReceive();
	void receive( );
	std::vector<camLinePointT> getLinePoints();
	cv::Mat getCameraFrame();
	bool getWaitForNew() { return this->waitForNew; }
	bool receivedNewLinePoints() {
		if( newLinePointsReceived ) {
			newLinePointsReceived = false;
			return true;
		} else {
			return false;
		}
	}
	bool receivedNewCameraFrame() {
		if( newCameraFrameReceived ) {
			newCameraFrameReceived = false;
			return true;
		} else {
			return false;
		}
	}
};

#endif
