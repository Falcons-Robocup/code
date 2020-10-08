 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
