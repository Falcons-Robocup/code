 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2017 Andre Pool and Geraldo Santiago
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef DETERMINEPOSITION_HPP
#define DETERMINEPOSITION_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "fieldLut.hpp"
#include <iostream>
#include "configurator.hpp"
#include "linePointDetection.hpp"
#include "robotFloor.hpp"
#include "observer.hpp"

#include <mutex>

// used as intermediate to export positions to ros
#define NUMPOSITION 10

typedef struct{
	float x[NUMPOSITION]; // range -(9+1) to (9+1) meter, center point is 0
	float y[NUMPOSITION]; // range -(6+1) to (6+1) meter, center point is 0
	float rz[NUMPOSITION]; // range 0 to 360 degrees, 0 is on x axis, anti clock wise
	float score[NUMPOSITION]; // range 0 (good) to 1 (bad), roughly below 0.2 is probably location lock
	float fps[NUMPOSITION]; // APOX todo: if available move fps to general ros WorldsSensing api
	float age[NUMPOSITION];
	float lastActive[NUMPOSITION];
	int linePoints[NUMPOSITION];
	size_t numPositions;
} detPosStRosVect;

class determinePosition
{

private:

	struct th {
		pthread_t thread;
		determinePosition * classContext;
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
	struct th threads[4]; // we have 4 cpu's available for: 1 good position + 2 possible positions (which toggle sometimes with the good position) + 1 random position
	unsigned int numThreads;
	std::mutex exportMutex;

	bool locConfident; // active when found in recent past
	int lostLoc; // keep track when we lost lock when we recently where confident we had the localization position
	detPosSt goodEnoughLoc; // containing the last known position

	positionStDbl createRandomPosition(positionStDbl previousPos, bool useRecentPos);
	detPosSt optimizePosition(positionStDbl startPos, bool localSearch, cv::Ptr<cv::optim::DownhillSolver> solverLUT);
	static void* processOneLocation( void *id );
	void goodEnough(); // determine the best position (if any)
	void notifyNewPos();

public:
	determinePosition(configurator *conf, linePointDetection *linePoint, preprocessor *prep, robotFloor *rFloor);
	~determinePosition();
	void pointsToPosition();
	std::vector<detPosSt> getLocList();

	detPosSt getGoodEnoughLoc(); // get the last know position or none
	detPosSt getGoodEnoughLocExport( );
	detPosSt getGoodEnoughLocExportRos( );

	// only for ros
	void attach(observer *observer);
	void detach(observer *observer);
	detPosStRosVect getFLoorLocationsRos();
};

#endif
