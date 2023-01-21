// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// TODO: rename to calcConfidence

#ifndef FIELDLUT_HPP
#define FIELDLUT_HPP

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "linePoints.hpp"
#include "optim.hpp"
#include "robotFloor.hpp"

class fieldLut: public cv::optim::Solver::Function {

public:
   fieldLut(linePoints *lPoints, robotFloor *rFloor);
   double calc(const double *x) const;
   scoreStruct calcSimple(const float *x);

   // for maximal performance, direct access to the following variables by the search algorithm
   float xLeft;
   float xRight;
   float xScale;

   float yBottom;
   float yScale;
   float yTop;

private:
   // pointers for access to other classes
   linePoints *lPoints = nullptr;
   robotFloor *rFloor = nullptr;
};

#endif
