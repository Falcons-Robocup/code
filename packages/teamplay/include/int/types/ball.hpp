 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ball.hpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Coen Tempelaars
 */

#ifndef BALL_HPP_
#define BALL_HPP_

#include "vector3d.hpp"

namespace teamplay
{

class ball {
public:
	ball();
    virtual ~ball();
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
