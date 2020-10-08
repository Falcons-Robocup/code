 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * LeastSquaresInterpolator.cpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#include "ext/LeastSquaresInterpolator.hpp"
#include "ext/LinearInterpolator.hpp"

#include <libalglib/interpolation.h>
#include <libalglib/specialfunctions.h>


LeastSquaresInterpolator::LeastSquaresInterpolator(int order, float range)
{
    _order = order;
    _range = range;
}

// standard probability density function
// belonging to normal distribution
// (annoyingly enough, it was chosen to not make this a part of alglib nor stdlib ?!!)
// https://stackoverflow.com/questions/19387222/probability-density-function-using-the-standard-library
float pdf(float x)
{
    return exp(-x*x*0.5) / sqrt(2*M_PI);
}

float LeastSquaresInterpolator::evaluate(float x)
{
    // use alglib to calculate polynomial, while selecting data of interest
    // see also alglib example lsfit_d_pol.c

    // require at least 1 data point
    if (_data.size() < 1)
    {
        throw std::runtime_error("no data to fit");
    }

    // construct alglib arrays
    // for continuity, weights are chosen as function of distance (scaled with configured range)
    alglib::real_1d_array w;
    alglib::real_1d_array xa;
    alglib::real_1d_array ya;
    w.setlength(_data.size());
    xa.setlength(_data.size());
    ya.setlength(_data.size());
    size_t idx = 0;
    float totalWeight = 0.0;
    for (auto it = _data.begin(); it != _data.end(); ++it)
    {
        float xx = it->first;
        xa[idx] = xx;
        ya[idx] = it->second;
        w[idx] = pdf((xx - x) / _range);
        totalWeight += w[idx];
        idx++;
    }

    // fallback to linear interpolation
    if (totalWeight < 0.01)
    {
        LinearInterpolator fallbackInterpolator;
        fallbackInterpolator._data = _data;
        return fallbackInterpolator.evaluate(x);
    }

    // (weighted) fit polynomial
    alglib::ae_int_t m = _order + 1;
    alglib::ae_int_t info;
    alglib::barycentricinterpolant p;
    alglib::polynomialfitreport rep;
    alglib::real_1d_array xc = "[]";
    alglib::real_1d_array yc = "[]";
    alglib::integer_1d_array dc = "[]";
    polynomialfitwc(xa, ya, w, xc, yc, dc, m, info, p, rep);

    // evaluate at given x
    return barycentriccalc(p, x);
}

