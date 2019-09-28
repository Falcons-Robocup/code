 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLinearInterpolator.cpp
 *
 *  Created on: Dec 3, 2017
 *      Author: Jan Feitsma
 */

#include "int/cLinearInterpolator.hpp"
#include "tracing.hpp"


cLinearInterpolator::cLinearInterpolator()
{
}

cLinearInterpolator::~cLinearInterpolator()
{
}

float cLinearInterpolator::interpolate(float v)
{
    TRACE_FUNCTION("");
    float result = 0.0;
    bool first = true;
    bool found = false;
    std::map<float, float>::iterator prev = _samples.begin();
    for (std::map<float, float>::iterator it = _samples.begin(); (!found) && (it != _samples.end()); ++it)
    {
        if (v <= it->first)
        {
            if (first) 
            {
                // take smallest sample
                result = it->second;
            }
            else
            {
                // linear interpolation
                float d = it->first - prev->first;
                float w1 = 1.0 - (v - prev->first) / d;
                float w2 = 1.0 - (it->first - v) / d;
                result = w1 * prev->second + w2 * it->second;
            }
            found = true;
        }
        prev = it;
        first = false;
    }
    if (!found)
    {
        // value too large -> take largest sample
        result = prev->second;
    }
    return result;
}


