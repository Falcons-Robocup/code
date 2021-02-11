// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef FIELDLUT_HPP
#define FIELDLUT_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "optim.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "configurator.hpp"
#include "linePointDetection.hpp"
#include "robotFloor.hpp"

class fieldLut: public cv::optim::Solver::Function
{

private:
	// pointers for access to other classes
	configurator *conf;
	linePointDetection *linePoint;
	robotFloor *rFloor;

	int pixelThreshold;

	void initializeFrameCost();

public:
	fieldLut(	configurator *conf,	linePointDetection *linePoint, robotFloor *rFloor);
	double calc(const double* x) const;
	scoreStruct calcSimple( positionStDbl pos, double threshold );
};

#endif
