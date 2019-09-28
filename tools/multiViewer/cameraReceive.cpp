// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "cameraReceive.hpp"

using namespace cv;
using namespace std;

cameraReceive::cameraReceive() {

	// ## network ##
	uint16_t port = 44444;

	// create a normal UDP socket
	fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		perror("ERROR   : cannot create UDP socket!, message");
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port
	int reuse = 1;
	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse))
			< 0) {
		perror(
				"ERROR     : cannot configure port for multiple UTP sockets, message");
		exit(EXIT_FAILURE);
	}

	// fill in the local address struct
	struct sockaddr_in localAddr;
	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(port); // port
	// bind to the local address / port
	if (::bind(fd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		perror("ERROR     : cannot bind to UDP socket, message");
		exit(EXIT_FAILURE);
	}
	printf("INFO    : setup UDP send connection, listen on IP %s port %u\n",
			inet_ntoa(localAddr.sin_addr), ntohs(localAddr.sin_port));
	fflush(stdout);
	// ## network done ##

	for (size_t ii = 0; ii < 3; ii++) {
		rxPacketCntExpected[ii] = 0; // initialize at 0
		rxPacketCntFirstCheck[ii] = true;
	}
	linePointListExportMutex.unlock(); // probably not needed
	cameraFrameExportMutex.unlock();
	waitForNew = true;
	cameraFrame = Mat::zeros(SEND_IMAGE_HEIGHT, SEND_IMAGE_WIDTH, CV_8UC3);

	newCameraFrameReceived = false;
	newLinePointsReceived = false;
}

void cameraReceive::receive() {

	int location = 0;

	while (true) {
		// block until data received
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		ssize_t nBytes = recvfrom(fd, &rxPacket, sizeof(camPacketT), 0,
				(struct sockaddr *) &fromAddr, &fromAddrlen);
		if (nBytes < 0) {
			perror("ERROR   : reading from socket, message");
			close(fd);
			exit(EXIT_FAILURE);
		}

		bool valid = true;
		if (rxPacket.size != nBytes) {
			printf("ERROR   : received %zd bytes, but expected %u bytes\n",
					nBytes, rxPacket.size);
			valid = false;
		} else {
			// printf("INFO    : received %u bytes from sender %s:%u\n",
			//		rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//		ntohs(fromAddr.sin_port));
		}

		size_t camIndex = rxPacket.id >> 4; // highest 4 bits contain the localization of the camera
		if (valid && (camIndex > 3)) {
			valid = false;
			// the single test camera is using index 4, so do not use, but also no error
			if( camIndex > 4 ) {
				printf("ERROR   : camera location %zu out of range for rx packet id %u\n", camIndex, rxPacket.id);
			}
		}

		if (valid && (rxPacket.cnt != rxPacketCntExpected[camIndex])) {
			if (rxPacketCntFirstCheck[camIndex]) {
				rxPacketCntFirstCheck[camIndex] = false;
				rxPacketCntExpected[camIndex] = rxPacket.cnt + 1; // set the expected for the next received camera packet
			} else {
				printf("ERROR     : for camera %zu received camera packet counter %u, but expected packet counter %u\n",
						camIndex, rxPacket.cnt, rxPacketCntExpected[camIndex]);
				rxPacketCntExpected[camIndex] = rxPacket.cnt + 1;
				valid = false;
			}
		} else {
			rxPacketCntExpected[camIndex]++;
		}

		if (valid) {
			if (rxPacket.id == (location * 16 + 2) ) {
				size_t points = (rxPacket.size - CAM_HEADER_SIZE) / 4; // one point contains 2 values of uint16_t
				linePointListExportMutex.lock();
				linePointList.clear();
				for (size_t ii = 0; ii < points; ii++) {
					camLinePointT linePoint;
					linePoint.x = rxPacket.pl.u16[2 * ii];
					linePoint.y = rxPacket.pl.u16[2 * ii + 1];
					linePointList.push_back(linePoint);
				}
				newLinePointsReceived = true;
				linePointListExportMutex.unlock();
			} else if ( (rxPacket.id >= (location * 16 + 8) ) && (rxPacket.id <= (location * 16 + 8 + 3) ) ) {
				size_t size = rxPacket.size - CAM_HEADER_SIZE;
				if (size != ( 3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT / 4)) {
					printf(
							"ERROR   : received %zd bytes, but expected %u bytes for quarter image\n",
							size, ( 3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT / 4));
					exit(EXIT_FAILURE);
				}
				memcpy(&receive_image_buffer[(rxPacket.id - (location * 16 + 8) ) * size], &rxPacket.pl.u8[0],
						size);

				if (rxPacket.id == (location * 16 + 8 + 3)) {
				size_t pixels = 4 * size / 3; // one pixel is 3 bytes
				if (pixels > (SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT)) {
					pixels = SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT;
				}
				size_t xx = 0;
				size_t yy = 0;
				cameraFrameExportMutex.lock();
				for (size_t ii = 0; ii < pixels; ii++) {
					cameraFrame.at<Vec3b>(yy, xx)[0] = receive_image_buffer[3 * ii]; // blue
					cameraFrame.at<Vec3b>(yy, xx)[1] =
							receive_image_buffer[3 * ii + 1]; // green
					cameraFrame.at<Vec3b>(yy, xx)[2] =
							receive_image_buffer[3 * ii + 2]; // red
					xx++;
					if (xx == SEND_IMAGE_WIDTH) {
						xx = 0;
						yy++;
					}
				}
				newCameraFrameReceived = true;
				cameraFrameExportMutex.unlock();
				}
			} else {
				// printf("WARNING : unknown camera packet id %u\n", rxPacket.id);
			}
		}
		waitForNew = false;
	}
}

vector<camLinePointT> cameraReceive::getLinePoints() {
	linePointListExportMutex.lock();
	vector<camLinePointT> retVal = linePointList;
	waitForNew = true;
	linePointListExportMutex.unlock();
	return retVal;
}

Mat cameraReceive::getCameraFrame() {
	cameraFrameExportMutex.lock();
	Mat retVal = cameraFrame;
	waitForNew = true;
	cameraFrameExportMutex.unlock();
	return retVal;
}
