 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: lucas catabriga
 * Creation: 2019-09-12
 *
 * Utility class: Represent a 2D gaussian distribution
 */

#ifndef GAUSSIAN2D_HPP
#define GAUSSIAN2D_HPP

#include "vector2d.hpp"
#include "matrix22.hpp"


class Gaussian2D
{
public:
    Gaussian2D(Vector2D mean, Matrix22 covariance)
    {
        this->mean = mean;
        this->covariance = covariance;
    }

    // The two directions must be orthogonal
    Gaussian2D(Vector2D mean, Vector2D direction1, double varianceDir1, Vector2D direction2, double varianceDir2)
    {
        Matrix22 V(direction1, direction2);
        Matrix22 L( varianceDir1, 0,
                    0, varianceDir2);

        Matrix22 covariance = V * L * V.getInverse();

        this->mean = mean;
        this->covariance = covariance;
    }

    Gaussian2D operator *(const Gaussian2D& other)
    {
        Matrix22 myCovInv = covariance.getInverse();
        Matrix22 otherCovInv = other.covariance.getInverse();

        Matrix22 newCovariance = (myCovInv + otherCovInv).getInverse();
        Vector2D newMean = newCovariance*(myCovInv*mean + otherCovInv*other.mean);

        return Gaussian2D(newMean, newCovariance);
    }

    Gaussian2D operator +(const Gaussian2D& other)
    {
        Matrix22 newCovariance = covariance + other.covariance;
        Vector2D newMean = mean + other.mean;

        return Gaussian2D(newMean, newCovariance);
    }

    double getIntersection(const Gaussian2D& other)
    {
        Vector2D meanDiff = (mean - other.mean);
        Matrix22 covarianceSum = covariance + other.covariance;
        double ZcExp = -0.5 * (covarianceSum.getInverse().pre_multiply(meanDiff) * meanDiff);
        double Zc = (1.0/sqrt(2.0*M_PI*covarianceSum.getDet())) * exp(ZcExp);
        return Zc;
    }

    Vector2D getMean() const
    {
        return mean;
    }

    Matrix22 getCovariance() const
    {
        return covariance;
    }

private:
    Vector2D mean;
    Matrix22 covariance;
};

#endif