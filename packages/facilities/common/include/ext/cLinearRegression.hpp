// Copyright 2016 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

