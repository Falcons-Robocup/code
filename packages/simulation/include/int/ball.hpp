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
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#ifndef BALL_HPP_
#define BALL_HPP_

#include "teamID.hpp"
#include "vector3d.hpp"

class Ball {
public:
    Point2D getLocation() const;
    Point2D getPickupLocation() const;
    Point3D getPosition() const;
    Point3D getPositionFCS(const TeamID&) const;
    float getSpeed() const;
    Vector3D getVelocity() const;
    bool isMoving() const;

    void setLocation (const Point2D&);
    void setPickupLocation (const Point2D&);
    void setPosition (const Point3D&);
    void setVelocity (const Vector3D&);
    void stopMoving();

    void teleport (const Vector2D&);
    void teleport (const float x, const float y);
    void teleport (const float x, const float y, const float z);

private:
    Point2D _pickupLocation;
    Point3D _position;
    Vector3D _velocity;
};

#endif /* BALL_HPP_ */
