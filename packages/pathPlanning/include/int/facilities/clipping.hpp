 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * clipping.hpp
 *
 *  Created on: Nov, 2019
 *      Author: Jan Feitsma
 */

#ifndef CLIPPING_HPP_
#define CLIPPING_HPP_

#include "falconsCommon.hpp"


// adapter for template clip()
inline double fabs(Vector2D const &v)
{
    return v.size();
}

// generalized clipping of a value v to an area around p with radius r
// v may be a float or vector2d
template <typename T>
bool gclip(T &v, T const p, float r, char const *label = "")
{
    r = fabs(r);
    T vo = v;
    T delta = v - p;
    if ((fabs(delta) > r) && (fabs(delta) > 0.1))
    {
        // normalize
        delta *= r / fabs(delta);
        v = p + delta;
        TRACE("clipping %s from %6.2f to %6.2f", label, fabs(vo), fabs(v));
        return true;
    }
    return false;
}

#endif

