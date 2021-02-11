// Copyright 2015-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
