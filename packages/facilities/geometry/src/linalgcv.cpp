// Copyright 2017-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * linalgcv.cpp
 *
 *  Created on: Sep 20, 2016
 *      Author: Jan Feitsma
 */

#include "linalgcv.hpp"
#include "tracing.hpp"


void traceMat(cv::Mat const &M, std::string name)
{
    TRACE("sizeof(%s) = (%d, %d)", name.c_str(), M.rows, M.cols);
    //return; // comment this line for detailed tracing
    size_t const bufsize = M.cols * 10;
    for (int irow = 0; irow < M.rows; ++irow)
    {
        char buf[bufsize];
        char *p = buf;
        for (int icol = 0; icol < M.cols; ++icol)
        {
            float v = M.at<float>(irow, icol);
            int n = sprintf(p, "%8.3f ", v);
            p += n;
        }
        TRACE("%s", buf);
    }
}

cv::Mat fct_translate(float tx, float ty, float tz)
{
    cv::Mat result = cv::Mat::eye(4, 4, CVFLOAT);
    result.at<float>(0, 3) = tx;
    result.at<float>(1, 3) = ty;
    result.at<float>(2, 3) = tz;
    return result;
}

cv::Mat fct_xrotate(float angle)
{
    cv::Mat result = cv::Mat::eye(4, 4, CVFLOAT);
    float ca = cos(angle);
    float sa = sin(angle);
    result.at<float>(1, 1) = ca;
    result.at<float>(1, 2) = -sa;
    result.at<float>(2, 1) = sa;
    result.at<float>(2, 2) = ca;
    return result;
}

cv::Mat fct_zrotate(float angle)
{
    cv::Mat result = cv::Mat::eye(4, 4, CVFLOAT);
    float ca = cos(angle);
    float sa = sin(angle);
    result.at<float>(0, 0) = ca;
    result.at<float>(0, 1) = -sa;
    result.at<float>(1, 0) = sa;
    result.at<float>(1, 1) = ca;
    return result;
}

cv::Mat fct_mat_fcs2rcs(float x, float y, float phi)
{
    return fct_zrotate(-phi + 0.5*M_PI) * fct_translate(-x, -y, 0);
}

cv::Mat fct_mat_rcs2fcs(float x, float y, float phi)
{
    return fct_translate(x, y, 0) * fct_zrotate(phi - 0.5*M_PI);
}

cv::Mat constructD(float cx, float cy, float cz, float cphi, float az, float el)
{
    // using a notation (auxiliary functions) consistent with Matlab toolbox
    // solution is in FCS -> convert to RCS
    cv::Mat D1 = fct_mat_fcs2rcs(cx, cy, cphi);
    // convert RCS to CCS by taking camera mounting height into account
    D1 = D1 * fct_translate(0, 0, -cz);
    // rotate away both measurement angles
    cv::Mat D2 = fct_xrotate(-el) * fct_zrotate(-az);
    return (D2 * D1);
}

cv::Mat weightMatrix(size_t N, float w)
{
    cv::Mat W = cv::Mat::eye(4*N, 4*N, CVFLOAT);
    for (size_t imeas = 0; imeas < N; ++imeas)
    {
        // leave weights at 1, except every 2nd element of 4 (associated with measurement distance)
        size_t idx = 4*imeas + 1;
        W.at<float>(idx, idx) = w;
    }
    return W;
}

Vector3D sph2cart(float az, float el, float r)
{
    return Vector3D(r * cos(el) * cos(az), r * cos(el) * sin(az), r * sin(el));
}

Vector3D object2fcs(float cx, float cy, float cz, float cphi, float az, float el, float r)
{
    // spherical to RCS
    Vector3D result = sph2cart(az + M_PI*0.5, el, r);
    //TRACE("cartesian %6.2f %6.2f %6.2f", result.x, result.y, result.z);
    // convert to FCS
    cv::Mat M = fct_mat_rcs2fcs(cx, cy, cphi);
    cv::Mat tmp = cv::Mat::zeros(4, 1, CVFLOAT);
    tmp.at<float>(0, 0) = result.x;
    tmp.at<float>(1, 0) = result.y;
    tmp.at<float>(2, 0) = result.z;
    tmp.at<float>(3, 0) = 1.0;
    tmp = M * tmp; // do the transformation
    result.x = tmp.at<float>(0, 0);
    result.y = tmp.at<float>(1, 0);
    result.z = tmp.at<float>(2, 0);
    //TRACE("objectFCS x=%6.2f y=%6.2f z=%6.2f", result.x, result.y, result.z);
    // fix camera mounting height
    result.z += cz;
    return result;
}

