// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagnostics.hpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include "vector2d.hpp"

namespace teamplay
{

class diagnostics {
public:
    diagnostics();
    virtual ~diagnostics();

    virtual void setAiming(const bool);
    virtual bool isAiming() const;

    virtual void setShootTarget(const Point2D&);
    virtual Point2D getShootTarget() const;

private:
    bool _aiming;
    Point2D _shootTarget;
};


} /* namespace teamplay */

#endif /* DIAGNOSTICS_HPP_ */
