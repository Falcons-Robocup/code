// Copyright 2014-2022 Andre Pool and Geraldo Santiago
// SPDX-License-Identifier: Apache-2.0

#ifndef DETERMINEPOSITION_HPP
#define DETERMINEPOSITION_HPP

#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "fieldLut.hpp"
#include "configurator.hpp"
#include "linePointDetection.hpp"
#include "robotFloor.hpp"

class determinePosition {

private:

	struct th {
		pthread_t thread;
		determinePosition *classContext;
		detPosSt determinedPosition;
		detPosSt start;
		detPosSt found;
		int id;
		int ret;
		double scoreThreshHold;
		cv::Ptr<cv::optim::DownhillSolver> solverLUT;
		fieldLut *costTable;
	};

	// pointers for access to other classes
	configurator *conf;
	preprocessor *prep;
	linePointDetection *linePoint;
	robotFloor *rFloor;

	fieldLut *LUT_ptr;
	double scoreThreshHold;
	double sameLocRangePow2; // when locations are within this range it is expected that is the same range, pow2 to save an sqrt for each check
	std::vector<detPosSt> locList, locListExport;
	struct th threads[4]; // we have 4 cpu's available for: 1 good position + 2 possible positions (which toggle sometimes with the good position) + 1 random position
	unsigned int numThreads;
	std::mutex exportMutex;

	bool locConfident; // active when found in recent past
	int lostLoc; // keep track when we lost lock when we recently where confident we had the localization position
	detPosSt goodEnoughLoc; // containing the last known position

	positionStDbl createRandomPosition();
	detPosSt optimizePosition(positionStDbl startPos, bool localSearch, cv::Ptr<cv::optim::DownhillSolver> solverLUT);
	static void* processOneLocation(void *id);
	void goodEnough(); // determine the best position (if any)

public:
	determinePosition(configurator *conf, linePointDetection *linePoint, preprocessor *prep, robotFloor *rFloor);
	~determinePosition();
	void pointsToPosition();
	std::vector<detPosSt> getLocList();

	detPosSt getGoodEnoughLoc(); // get the last know position or none
	detPosSt getGoodEnoughLocExport();
	detPosSt getGoodEnoughLocExportRos();
};

#endif
