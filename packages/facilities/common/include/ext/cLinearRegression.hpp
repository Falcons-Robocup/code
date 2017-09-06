 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*  cLinearRegression.hpp

    was:linreg.h
    Linear Regression calculation class

    by: David C. Swaim II, Ph.D.
    Jan 2016: adopted by M. Koenen, major cleanup and refactor for ASML Falcons for RoboCup purpose


    This class implements a standard linear regression on
    experimental data using a least squares fit to a straight
    line graph.  Calculates coefficients a and b of the equation:

                        y = a + b * x
                        y = yIntercept + slope *x

    for data points of x and y.  Also calculates the coefficient of
    determination, the coefficient of correlation, and standard
    error of estimate.

    The value n (number of points) must be greater than 2 to
    calculate the regression.  This is primarily because the
    standard error has a (N-2) in the denominator.

    Check haveData() to see if there is enough data in
    LinearRegression to get values.

    You can think of the x,y pairs as 2 dimensional points.
  */
#ifndef _LINREG_H_
#define _LINREG_H_
#include <iostream>
#include <cmath>
#include <limits>

class cLinearRegression
{

protected:
	long  n;           // number of data points input so far
	bool  dirty;	   // keep track if re calculation should be done
	float sumX, sumY;  // sums of x and y
	float sumXsquared; // sum of x squares
	float sumYsquared; // sum y squares
	float sumXY;       // sum of x*y

	float a, b;        // coefficients of f(x) = a + b*x
	float coefD,       // coefficient of determination
	coefC,       // coefficient of correlation
	stdError;    // standard error of estimate

	void Calculate();   // calculate coefficients

public:
	~cLinearRegression();
	cLinearRegression();

	// Constructor using arrays of x values and y values
	cLinearRegression(float *x, float *y, long size = 0);


	void addXY(const float& x, const float& y);

	// Must have at least 3 points to calculate
	// standard error of estimate.  Do we have enough data?
	int haveData() const { return (n > 2 ? 1 : 0); }
	long items() const { return n; }

	float getA(),getYIntercept();
	float getB(),getSlope();

	float getCoefDeterm();
	float getCoefCorrel();
	float getStdErrorEst();
	float estimateY(float x);
	float estimateX(float y);
};

#endif                      // end of cLinearRegression.hpp

