 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef OBSTACLEDETECTION_HPP
#define OBSTACLEDETECTION_HPP

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "configurator.hpp"
#include "preprocessor.hpp"
#include "observer.hpp"

#include <mutex>

typedef struct
{
	int xLeft;
	int xRight;
	int yTop;
	int yBottom;
	int size;
	double radiusTan;
	double radiusLin;
	double angle;
} obstacleSt;

class obstacleDetection
{

private:
	// pointers for access to other classes
	preprocessor *prep;
	configurator *conf;

	std::vector<obstacleSt> positions, positionsExport;
	cv::Mat blackFloor, obstacleFrame, obstacleErodeFrame, obstacleDilateFrame, obstacleMask;

	int cameraRadiusObstacleMinPrevious;
	int cameraRadiusObstacleMaxPrevious;
	cv::Point centerPointPrevios;
	std::mutex exportMutex;
	bool busy;

	// List of observers for Obstacle position notifications
	std::vector<observer*> vecObservers; // only for Ros
	void notifyNewPos(); // only for Ros

public:
	obstacleDetection(configurator *conf, preprocessor *prep );
	void update( );
	std::vector<obstacleSt> getPositions();
	std::vector<obstacleSt> getPositionsExport();

	cv::Mat getObstacle();
	cv::Mat getObstacleErode();
	cv::Mat getObstacleDilate();

	bool getBusy() { return busy; }
	void setBusy() { busy = true; }

	// only for Ros
	void attach(observer *observer); // only for Ros
	void detach(observer *observer); // only for Ros
};
#endif
