 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * AbstractInterpolator.hpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#ifndef _INCLUDED_ABSTRACTINTERPOLATOR_HPP_
#define _INCLUDED_ABSTRACTINTERPOLATOR_HPP_


#include <map>
#include <string>


class AbstractInterpolator
{
public:
    AbstractInterpolator();
    virtual ~AbstractInterpolator();

    // load data from file
    void load(std::string const &filename);

    // clear data
    void clear();

    // insert a data point
    void feed(float x, float y);

    // evaluate y at given x
    virtual float evaluate(float x) = 0;

    // write data and interpolation results to file, for visual inspection
    // (see tst/plotInterpolation.py)
    void writeSampleData(std::string const &filename = "/var/tmp/interpolationSampleData.txt");
    void writeInterpolatedCurve(float xmin, float xmax, float dx, std::string const &filename = "/var/tmp/interpolationCurve.txt");

    // data (x,y) pairs
    std::map<float, float> _data;
};

#endif

