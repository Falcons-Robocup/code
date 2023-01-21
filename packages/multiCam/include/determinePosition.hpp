// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include "observer.hpp"

// used as intermediate to export positions to ros
#define NUMPOSITION 10

typedef struct {
	float x[NUMPOSITION]; // range -(9+1) to (9+1) meter, center point is 0
	float y[NUMPOSITION]; // range -(6+1) to (6+1) meter, center point is 0
	float rz[NUMPOSITION]; // range 0 to 360 degrees, 0 is on x axis, anti clock wise
	float score[NUMPOSITION]; // range 0 (good) to 1 (bad), roughly below 0.2 is probably location lock
	float fps[NUMPOSITION]; // APOX TODO: if available move fps to general ros WorldsSensing api
	float age[NUMPOSITION];
	float lastActive[NUMPOSITION];
	int linePoints[NUMPOSITION];
	size_t numPositions;
} detPosStRosVect;

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
	std::vector<observer*> vecObservers;
	struct th threads[5]; // we have 4 cpu's (8 with hyper threading) available for: 1 good position + 3 possible positions (which toggle sometimes with the good position) + 1 random position
	unsigned int numThreads;
	std::mutex exportMutex;

	bool locConfident; // active when found in recent past
	int lostLoc; // keep track when we lost lock when we recently where confident we had the localization position
	detPosSt goodEnoughLoc; // containing the last known position

	positionStDbl createRandomPosition(positionStDbl previousPos, bool useRecentPos);
	detPosSt optimizePosition(positionStDbl startPos, bool localSearch, cv::Ptr<cv::optim::DownhillSolver> solverLUT);
	static void* processOneLocation(void *id);
	void goodEnough(); // determine the best position (if any)
	void notifyNewPos();

public:
	determinePosition(configurator *conf, linePointDetection *linePoint, preprocessor *prep, robotFloor *rFloor);
	~determinePosition();
	void pointsToPosition();
	std::vector<detPosSt> getLocList();

	detPosSt getGoodEnoughLoc(); // get the last know position or none
	detPosSt getGoodEnoughLocExport();
	detPosSt getGoodEnoughLocExportRos();

	// only for ROS
	void attach(observer *observer);
	void detach(observer *observer);
	detPosStRosVect getFLoorLocationsRos();
};

#endif
