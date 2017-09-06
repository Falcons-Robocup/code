 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014, 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef DYNAMICCALIBRATION_HPP
#define DYNAMICCALIBRATION_HPP

#include "configurator.hpp"
#include "preprocessor.hpp"

#include <iostream>
#include <fstream>

class dynamicCalibration
{

private:
	configurator *conf;
	preprocessor *prep;
	cv::Scalar lineColor, floorColor, ballColor;
	int get(std::string setName, int id);
	void set(std::string setName, int id, int value);
	void nameSet(std::string setName, int id, int prevVal, int newVal, bool verbose );
	cameraSt cameraPrev;
	int floorPixelsPrev;
	int gainModeFrames;
	bool whiteBalanceTemperatureCalibration;
	bool whiteBalanceTemperatureUp;
	int whiteBalanceTemperatureBest;
	int lineSatMaxDefault;
	int fd; // file descriptor to camera, set once through initialize
	size_t powerLineSetFreq;
	std::ofstream statsFile;
	char currentDate[64], currentTime[64], fileName[64], fileNameBest[64], fileNameCSV[64];
	int pixelsBest;
	cv::Mat imageBest;

public:
	dynamicCalibration(configurator *conf, preprocessor *prep);
	virtual ~dynamicCalibration();
	void initialize(int fd);
	void update();
	cv::Scalar getLineColor() { return lineColor; }
	cv::Scalar getFloorColor() { return floorColor; }
	cv::Scalar getBallColor() { return ballColor; }
	int getWhiteBalanceTemperature();
	int getGain();
	void captureStats(uint64 frameCounter);
	void sweep(int frameCounter);

};

#endif
