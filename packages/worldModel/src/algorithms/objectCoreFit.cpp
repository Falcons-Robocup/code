 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectCoreFit.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Jan Feitsma
 */

#include "int/algorithms/objectCoreFit.hpp"
#include "FalconsCommon.h" // from package common
#include "linalgcv.hpp" // from package geometry
#include "cDiagnostics.hpp" // from package diagnostics



void designMatrixTriang(
    // inputs
    std::vector<objectMeasurementCache> const &measurements,
    double t, 
    bool withSpeed,
    // outputs
    cv::Mat &A
    )
{
    A.release();
    for (size_t imeas = 0; imeas < measurements.size(); ++imeas)
    {
        cv::Mat row, age, D;
        // construct time-independent part of the matrix
        objectMeasurement bm = measurements[imeas].getObjectMeasurement();
        D = measurements[imeas].getCvMatrix();
        // add first order terms
        if (withSpeed)
        {
            age = cv::Mat::eye(4, 4, CVFLOAT) * (bm.timestamp - t);
            // TODO: maybe add 2nd order term for z... ? it is needed for precision of vertical bounce 
            // detection because it matches directly with gravity and without that term, we get
            // significant systematic latency. however we do not want to get more noise
            // sensitivity in x,y, so perhaps we can add counter with some hard constraints 
            // to the linear system? ==> simulate/prototype first
            hconcat(D, D * age, row);
        }
        else
        {
            // offset only
            row = D;
        }
        // stack row in the output matrix A, but treat first iteration as special case
        // because unlike Matlab, vconcat cannot handle zero-size dimensions
        if (imeas)
        {
            vconcat(A, row, A);
        }
        else
        {
            A = row;
        }
    }
}

void dataVectorTriang(
    // inputs
    std::vector<objectMeasurementCache> const &measurements,
    // outputs
    cv::Mat &b
    )
{
    // initialize
    b = cv::Mat::zeros(4*measurements.size(), 1, CVFLOAT);
    // construct
    for (size_t imeas = 0; imeas < measurements.size(); ++imeas)
    {
        // set azimuth and elevation to zero because these have been transformed away in the design matrix
        b.at<float>(imeas*4 + 0, 0) = 0.0;
        b.at<float>(imeas*4 + 2, 0) = 0.0;
        // in RCS, y is the distance, which is the second component
        b.at<float>(imeas*4 + 1, 0) = measurements[imeas].getObjectMeasurement().radius;
        // add fourth dummy data point with value 1
        b.at<float>(imeas*4 + 3, 0) = 1.0;
    }
}

void objectCoreFitTriang(
    // inputs
    std::vector<objectMeasurementCache> const &measurements,
    double t, 
    objectFitConfig config,
    // outputs
    objectResultType &objectResult,
    float &residual
    )
{
    // sanity check
    if ((int)measurements.size() > 1000)
    {
        TRACE_ERROR("objectCoreFitTriang got unexpected large amount of measurements (%d)", (int)measurements.size());
        return;
    }
    
    // construct linear system of equations A * p = b with p as parameters to be fitted

    // get configuration values
    bool withSpeed = false; // not used anymore, see trajectory fit instead
    float w = config.depthWeight;
    
    // initialization
    cv::Mat A, W, b, p, residuals;
    
    // construct design matrix A
    designMatrixTriang(measurements, t, withSpeed, A);
    
    // construct data vector b
    dataVectorTriang(measurements, b);
    
    // construct weight matrix W
    W = weightMatrix(measurements.size(), w);

    // perform the fit (need SVD for non-square system, to obtain least-squares solution)
    p = cv::Mat::zeros(A.cols, 1, CVFLOAT);
    cv::solve(W * A, W * b, p, cv::DECOMP_SVD);
    //TRACE("solver returned %d", r);
    
    // fill output
    objectResult.setTimestamp(t);
    objectResult.setCoordinates(p.at<float>(0, 0),
                                p.at<float>(1, 0),
                                p.at<float>(2, 0));
    if (withSpeed)
    {
        objectResult.setVelocities(
                p.at<float>(4, 0),
                p.at<float>(5, 0),
                p.at<float>(6, 0));
    }
    
    // calculate residual vector
    residuals = b - A * p;
    residual = 0.0;
    int N = 0;
    for (int it = 0; it < residuals.rows; it+=2)
    {
        // for now use only RCS (x,z) components (it=0,2,4,...), because distance is noisy and 4th dimension has to be ignored anyway
        // TODO make more precise?
        // see also fbt_solve_fit_quality.m
        residual += pow(residuals.at<float>(it, 0), 2);
        N++;
    }
    residual = sqrt(residual / N);
}

void designMatrixTrajectory(
    // inputs
    std::vector<double> const &timeStamps,
    double t, 
    int fitOrder,
    // output
    cv::Mat &A
    )
{
    A.release();
    for (size_t imeas = 0; imeas < timeStamps.size(); ++imeas)
    {
        cv::Mat row, age;
        // 0th order
        row = cv::Mat::eye(3, 3, CVFLOAT);
        // higher orders
        for (int iOrder = 1; iOrder <= fitOrder; ++iOrder)
        {
            age = cv::Mat::eye(3, 3, CVFLOAT) * pow(timeStamps[imeas] - t, iOrder);
            hconcat(row, age, row);
        }
        // stack row in the output matrix A, but treat first iteration as special case
        // because unlike Matlab, vconcat cannot handle zero-size dimensions
        if (imeas)
        {
            vconcat(A, row, A);
        }
        else
        {
            A = row;
        }
    }
}    

void dataVectorTrajectory(
    // input
    std::vector<Vector3D> const &positions,
    // output
    cv::Mat &b
    )
{
    // initialize
    b = cv::Mat::zeros(3*positions.size(), 1, CVFLOAT);
    // construct
    for (size_t imeas = 0; imeas < positions.size(); ++imeas)
    {
        b.at<float>(imeas*3 + 0, 0) = positions[imeas].x;
        b.at<float>(imeas*3 + 1, 0) = positions[imeas].y;
        b.at<float>(imeas*3 + 2, 0) = positions[imeas].z;
    }
}

void objectCoreFitTrajectoryIterative(
    // inputs
    std::vector<double> const &timeStamps,
    std::vector<Vector3D> const &positions,
    double t, 
    int fitOrder,
    int maxIter,
    float nSigma,
    float iterFraction,
    // outputs
    objectResultType &objectResult,
    float &residual,
    int &numRemovedTotal,
    std::vector<bool> &removedMask
    )
{
    // sanity checks
    assert(timeStamps.size() == positions.size());
    assert(timeStamps.size() > 0);
    assert(fitOrder >= 0);
    assert(fitOrder <= 1); // TODO 2 for curvature/gravity
    assert(timeStamps.size() < 1000);
    
    // construct linear system of equations A * p = b with p as parameters to be fitted

    // initialization
    size_t n = timeStamps.size();
    //TRACE("n=%d", n);
    cv::Mat A, b, p, res;
    cv::Mat mask1 = cv::Mat::eye(3*n, 3*n, CVFLOAT);
    cv::Mat mask2 = cv::Mat::ones(n, 1, CV_8U);
    
    // construct design matrix A
    designMatrixTrajectory(timeStamps, t, fitOrder, A);
    
    // construct data vector b
    dataVectorTrajectory(positions, b);
    
    // iterate
    numRemovedTotal = 0;
    int numRemaining = n;
    float stddevf = 0.0;
    int iter = 0;
    for (iter = 1; iter <= maxIter; ++iter)
    {
        
        // perform the fit (need SVD for non-square system, to obtain least-squares solution)
        p = cv::Mat::zeros(A.cols, 1, CVFLOAT);
        cv::solve(mask1 * A, mask1 * b, p, cv::DECOMP_SVD);
        
        // calculate and fold residual vector
        res = b - A * p;
        cv::Mat res2 = cv::Mat::zeros(n, 1, CVFLOAT);
        for (size_t imeas = 0; imeas < n; ++imeas)
        {
            // each position yields a signed error (dx,dy,dz)
            // outlier removal kicks out points, not components within points
            float dx = res.at<float>(imeas*3 + 0, 0);
            float dy = res.at<float>(imeas*3 + 1, 0);
            float dz = res.at<float>(imeas*3 + 2, 0);
            res2.at<float>(imeas, 0) = sqrt(dx*dx + dy*dy + dz*dz);
            //TRACE("imeas=%3d  dx=%5.1f  dy=%5.1f  res2=%6.2f", (int)imeas, dx, dy, res2.at<float>(imeas,0));
        }
        
        // analyze residuals statistics
        cv::Scalar mean = 0.0;
        cv::Scalar stddev = 0.0;
        meanStdDev(res2, mean, stddev, mask2);
        stddevf = stddev.val[0];
        //TRACE("mean=%.3f, stddev=%.3f", mean.val[0], stddevf);

        // fallback for numerically perfect data
        float stddev2 = std::max(stddevf, (float)0.01);
        
        // done already? do this after residual calculation, before outlier removal
        if (iter == maxIter)
        {
            break;
        }
        
        // identify outliers in residuals
        // sort by size, remove only X% largest ones, prevent removing good data
        std::multimap<float, size_t> removeCandidates;
        for (size_t imeas = 0; imeas < n; ++imeas)
        {
            if (mask2.at<unsigned char>(imeas, 0))
            {
                float ratio = fabs(res2.at<float>(imeas, 0)) / (nSigma * stddev2);
                if (ratio > 1.0)
                {
                    removeCandidates.insert(std::make_pair(ratio, imeas));
                }
            }
        }
        
        // now remove the outliers, but only the largest ones
        // make use of map sorting, iterate backwards
        int numRemovedIter = 0;
        for (auto it = removeCandidates.rbegin(); it != removeCandidates.rend(); ++it)
        {
            size_t imeas = it->second;
            //TRACE("imeas=%3d  ratio=%6.2f", (int)imeas, it->first);
            mask1.at<float>(imeas*3 + 0, imeas) = 0;
            mask1.at<float>(imeas*3 + 1, imeas) = 0;
            mask1.at<float>(imeas*3 + 2, imeas) = 0;
            mask2.at<unsigned char>(imeas, 0) = 0;
            
            numRemovedIter++;
            numRemaining--;
            if (numRemovedIter > n * iterFraction)
            {
                break; // enough removed, continue with next iteration
            }
        }

        // done?
        numRemovedTotal += numRemovedIter;
        if (numRemovedIter == 0)
        {
            break;
        }
        
        // outlier removal may not leave less than 2 measurements
        if (numRemaining <= 2)
        {
            break;
        }

    }
    
    // debugging
    //if (numRemovedTotal)
    //{
    //    float removedPercentage = float(100.0 * (n - (int)(cv::sum(mask2).val[0])) / n);
    //    //TRACE("removed %d out of %d measurements (%.1f%%) in %d iterations, resulting stddev=%.3f", numRemovedTotal, n, removedPercentage, iter, stddevf);
    //}
        
    // fill output
    objectResult.setTimestamp(t);
    objectResult.setCoordinates(p.at<float>(0, 0),
                                p.at<float>(1, 0),
                                p.at<float>(2, 0));
    if (fitOrder > 0)
    {
        objectResult.setVelocities(
                p.at<float>(3, 0),
                p.at<float>(4, 0),
                p.at<float>(5, 0));
    }
    
    // convert and return outlier mask
    removedMask.clear();
    for (size_t imeas = 0; imeas < n; ++imeas)
    {
        removedMask.push_back(mask2.at<unsigned char>(imeas, 0));
    }
    
    // return float residual, use stddevf to take out sensitivity to n
    residual = stddevf;
}

