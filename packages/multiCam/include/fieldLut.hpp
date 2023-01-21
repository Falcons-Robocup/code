// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef FIELDLUT_HPP
#define FIELDLUT_HPP

#include <opencv2/opencv.hpp>

#include "configurator.hpp"
#include "linePointDetection.hpp"
#include "optim.hpp"
#include "robotFloor.hpp"

class fieldLut: public cv::optim::Solver::Function {

private:
	// pointers for access to other classes
	configurator *conf;
	linePointDetection *linePoint;
	robotFloor *rFloor;

	int pixelThreshold;

	void initializeFrameCost();

public:
	fieldLut(configurator *conf, linePointDetection *linePoint, robotFloor *rFloor);
	double calc(const double *x) const;
	scoreStruct calcSimple(positionStDbl pos, double threshold);
};

#endif
