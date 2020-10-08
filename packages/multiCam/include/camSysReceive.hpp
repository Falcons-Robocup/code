 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAM_SYS_RECEIVE_HPP
#define CAM_SYS_RECEIVE_HPP

#include "camPacket.hpp"
#include <mutex>
#include <stdlib.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// TODO: get from central place
#define SEND_IMAGE_WIDTH (32*25) // 800
#define SEND_IMAGE_HEIGHT (32*19) // 608

struct camSystemSt {
	bool updated;

	uint16_t sysApplUptime; // system application uptime in seconds (wrap around at 18.2 hours)
	int8_t cmosTemp; // imx219 camera sensor temperature in Celsius
	uint8_t cpuLoad; // warning: fixed point, divide by 32.0 to get correct value, note: there are 4 physical cores on the Raspberry pi 3b+
	uint8_t cpuTemp; // A53 CPU temperature in Celsius
	uint16_t cpuUptime; // CPU uptime in seconds (wrap around at 18.2 hours)
	uint8_t gpuTemp; // GPU temperature in Celsius

	uint8_t cpuStatus; // not decoded: underVoltage, underVoltageOccured, frequencyCapped, frequencyCappedOccured, throttling and throttlingOccured
	bool underVoltage;
	bool underVoltageOccured;

	bool frequencyCapped;
	bool frequencyCappedOccured;

	bool throttling;
	bool throttlingOccured;

	bool softTempLimit;
	bool softTempLimitOccured;

	uint32_t md5sumPart0; // used to verify if raspi is using the correct files
	uint32_t md5sumPart1;
	uint32_t md5sumPart2;
	uint32_t md5sumPart3;

	double rebootTime; // local time when going to reboot message received from raspi
	double configAcknowledgeTime; // local time when received config acknowledge packet from raspi
};

struct hhMmSsSt {
	uint32_t hours;
	uint32_t minutes;
	uint32_t seconds;
};

class camSysReceive {
private:
	char *directory; // workspace directory of multiCam executable
	bool storeImages;
	bool imagePartValid[4][256] = {{false}};
	char dateCode[64];
	std::string imageGrabPath;
	int multReceiveFd;
	camPacketT rxPacket;
	uint8_t rxPacketCntExpected[4];
	bool rxPacketCntFirstCheck[4];
	camSystemSt camSys[4];
	size_t latestCamIndex;
	std::mutex camSysExportMutex;
	bool md5sumCheckEnable;
	uint32_t md5sumPart0Local;
	uint32_t md5sumPart1Local;
	uint32_t md5sumPart2Local;
	uint32_t md5sumPart3Local;

	uint32_t copyBuildRaspiAge[4];

	std::mutex camImageExportMutex;
	uint8_t camImage[4][3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT]; // max allocation for uncompressed image, typical only few % used (compressed)

	void md5sumCheck(size_t camIndex, uint32_t md5sumPart0Camera, uint32_t md5sumPart1Camera,
			uint32_t md5sumPart2Camera, uint32_t md5sumPart3Camera);

	void md5sumPart0Check(size_t camIndex, uint32_t md5sumPart0Camera);
	void storeImage(size_t camIndex, size_t receivedAlready);

public:

	camSysReceive();
	void getLocalMd5sum();
	void setMd5sumCheck(bool value) {
		md5sumCheckEnable = value;
	}
	void setImageGrabPath(std::string imageGrabPath);
	void receive(); // blocking call, run in separate thread
	camSystemSt getCamSystem(size_t camIndex);
	cv::Mat getCameraFrame(size_t camIndex);
	size_t getLatestCamIndex() {
		return latestCamIndex;
	}
	hhMmSsSt secondsToHhMmSs(uint16_t input);
};

#endif
