 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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


