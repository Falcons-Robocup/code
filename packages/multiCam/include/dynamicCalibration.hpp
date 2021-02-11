// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

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
