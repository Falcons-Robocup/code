// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Ball.hpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Coen Tempelaars
 */

#ifndef BALL_HPP_
#define BALL_HPP_

#include "vector3d.hpp"

namespace teamplay
{

class Ball {
public:
	Ball();
    virtual ~Ball();
    virtual void reset();

    virtual bool isLocationKnown() const;
    virtual bool isVelocityKnown() const;
    virtual bool isInsideField() const;
    virtual bool isAtOwnSide() const;
    virtual bool isAtOpponentSide() const;
    virtual bool isAtLeftSide() const;
    virtual bool isAtRightSide() const;
    virtual bool isInOwnPenaltyArea() const;
    virtual bool isInOpponentPenaltyArea() const;
    virtual bool isClaimedOnOpponentHalf() const;
    virtual bool mustBeAvoided() const;

    virtual Point2D getLocation() const;
    virtual Point3D getPosition() const;
    virtual Vector3D getVelocity() const;
    virtual Point2D getClaimedLocation() const;
    virtual Point3D getClaimedPosition() const;

    virtual void setPosition(const Point3D &);
    virtual void setVelocity(const Vector3D &);
    virtual void setPositionUnknown();
    virtual void setVelocityUnknown();
    virtual void setPositionClaimed(const Point3D &);
    virtual void setPositionClaimedUnknown();
    virtual void avoid();

private:
    bool _isCurrentPositionKnown;
    bool _isCurrentVelocityKnown;
    bool _isClaimedPositionKnown;
    Point3D _currentPosition;
    Vector3D _currentVelocity;
    Point3D _claimedPosition;
    bool _mustBeAvoided;
};

} /* namespace teamplay */

#endif /* BALL_HPP_ */
