// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * linalgcv.hpp
 *
 * Gathering the core cv::Mat linear algebra of ballTracking+obstacleTracking library.
 *
 *  Created on: Sep 20, 2016
 *      Author: Jan Feitsma
 */

#ifndef LINALGCV_HPP_
#define LINALGCV_HPP_

#include <string>
#include "opencv2/core/core.hpp"
#include "falconsCommon.hpp"


// 32bits single channel floats, should be enough
// although, Matlab uses 64 bit, so we might have to live with a small numerical difference
#define CVFLOAT (CV_32F)


void traceMat(cv::Mat const &M, std::string name);
cv::Mat fct_translate(float tx, float ty, float tz);
cv::Mat fct_xrotate(float angle);
cv::Mat fct_zrotate(float angle);
cv::Mat fct_mat_fcs2rcs(float x, float y, float phi);
cv::Mat constructD(float cx, float cy, float cz, float cphi, float az, float el);
cv::Mat weightMatrix(size_t N, float w);
Vector3D object2fcs(float cx, float cy, float cz, float cphi, float az, float el, float r);

#endif /* LINALGCV_HPP_ */
