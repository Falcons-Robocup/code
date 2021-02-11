// Copyright 2016 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
    file cLinearRegression.cpp
    Created on: Sep 11, 2014
    Author: David C. Swaim II, Ph.D.,
    Last Modified: Michel Koenen
*/
#include <math.h>
#include <float.h>
#include <cmath>
#include "ext/cLinearRegression.hpp"

cLinearRegression::cLinearRegression()
{
    a = b = sumX = sumY = sumXsquared = sumYsquared = sumXY = 0.0;
    coefC = coefD = stdError = 0.0;
    n = 0L;
    dirty = false;
}

cLinearRegression::~cLinearRegression()
{

}

cLinearRegression::cLinearRegression(float *x, float *y, long size) : cLinearRegression() //initialize first with default constructor
{
    long i;

    if (size > 0L) // if size greater than zero there are data arrays
        for (n = 0, i = 0L; i < size; i++)
            addXY(x[i], y[i]);
}

void cLinearRegression::addXY(const float& x, const float& y)
{
    n++;
    sumX += x;
    sumY += y;
    sumXsquared += x * x;
    sumYsquared += y * y;
    sumXY += x * y;
    dirty=true;
}

float cLinearRegression::getA()
{
	if( dirty )
		Calculate();
	return a;
}

float cLinearRegression::getYIntercept()
{
	return getA();
}

float cLinearRegression::getB()
{
	if ( dirty )
		Calculate();
	return b;
}

float cLinearRegression::getSlope()
{
	return getB();
}

float cLinearRegression::estimateY(float x)
{
	if ( dirty )
		Calculate();
	return (a + b * x);
}


float cLinearRegression::estimateX(float y)
{
	if ( b!=0 )
	{
		if ( dirty )
			Calculate();
		return ( (y-a)/b );
	} else
	{
		return std::numeric_limits<double>::quiet_NaN();
	};
}

float cLinearRegression::getCoefDeterm()
{
	if ( dirty )
		Calculate();
	return coefD;
}

float cLinearRegression::getCoefCorrel()
{
	if ( dirty )
		Calculate();
	return coefC;
}

float cLinearRegression::getStdErrorEst()
{
	if ( dirty )
		Calculate();
	return stdError;
}

void cLinearRegression::Calculate()
{
    if (haveData() && dirty==true )
    {
        if (fabs( float(n) * sumXsquared - sumX * sumX) > DBL_EPSILON)
        {
            b = ( float(n) * sumXY - sumY * sumX) /
                ( float(n) * sumXsquared - sumX * sumX);
            a = (sumY - b * sumX) / float(n);

            float sx = b * ( sumXY - sumX * sumY / float(n) );
            float sy2 = sumYsquared - sumY * sumY / float(n);
            float sy = sy2 - sx;

            coefD = sx / sy2;
            coefC = sqrt(coefD);
            stdError = sqrt(sy / float(n - 2));
        }
        else
        {
            a = b = coefD = coefC = stdError = 0.0;
        }
        dirty=false;
    }
}


