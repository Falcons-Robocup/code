 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * $Id: area2D.hpp 1930 2015-06-28 21:58:50Z jmbm $
 *
 *  Created on: Jun 28, 2015
 *      Author: jmbm
 */

#ifndef AREA2D_HPP_
#define AREA2D_HPP_

#include "position2d.hpp"
#include "vector2d.hpp"

struct Area2D {
    Vector2D    ll; // lower left corner
    Vector2D    ur; // upper right corner

    Vector2D lr; // lower right corner
    Vector2D ul; // upper left corner

    Area2D() : ll(0.0,0.0),ur(0.0,0.0),lr(0.0,0.0),ul(0.0,0.0) {};
    Area2D(double llx,double lly,double urx,double ury) : ll(llx,lly), ur(urx,ury), lr(urx,lly), ul(llx, ury) {};
    bool IncludesVector(const Vector2D &v) { return v.x>=ll.x && v.x<=ur.x && v.y>=ll.y && v.y<=ur.y;};
    bool includesPosition(const Position2D &v) { return v.x>=ll.x && v.x<=ur.x && v.y>=ll.y && v.y<=ur.y;};
    Vector2D getCenter() const {return Vector2D(((lr.x - ll.x)/2.0 + ll.x), (ul.y - ll.y)/2.0 + ll.y);};
    bool intersectsWith(const Area2D &other)
    {
        double left = fmax(ll.x, other.ll.x);
        double right = fmin(lr.x, other.lr.x);
        double top = fmin(ul.y, other.ul.y);
        double bottom = fmax(lr.y, other.lr.y);

        return ((left < right) && (bottom < top));
    };
};



#endif /* AREA2D_HPP_ */
