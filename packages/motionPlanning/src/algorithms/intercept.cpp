 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * intercept.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Jan Feitsma
 */

#include "int/algorithms/intercept.hpp"



void calculateIntercept(
    // inputs
    Position2D const &robotPos, 
    Velocity2D const &robotVel, // ignore for now, would need to call spg
    Vector3D const &objectPos, 
    Vector3D const &objectVel, // assume object moves at constant position
    // outputs
    bool &success,
    float &timeNeeded,
    Position2D &targetPos,
    Velocity2D &targetVel)
{
    // retrieve settings
    float robotTypicalSpeed = 1.6;
    bool allowMovingForward = false;

    // initialize
    timeNeeded = 0;
    targetPos = robotPos;
    targetVel = Velocity2D();
    
    // if object is moving too slow, then calculation is straightforward: just set target onto object 
    float objectSpeed = vectorsize(Vector2D(objectVel.x, objectVel.y));
    if (objectSpeed < 0.3)
    {
        float distance = vectorsize(Vector2D(objectPos.x, objectPos.y) - robotPos.xy());
        timeNeeded = distance / robotTypicalSpeed;
        targetPos.x = objectPos.x;
        targetPos.y = objectPos.y;
        targetPos.phi = angle_between_two_points_0_2pi(robotPos.x, robotPos.y, objectPos.x, objectPos.y);
        targetVel = Velocity2D();
        success = true;
        return;
    }
    
    // transform everything to RCS where object is moving along y axis
    Vector3D objectProjection = objectPos + objectVel;
    Position2D z(objectPos.x, objectPos.y, angle_between_two_points_0_2pi(objectProjection.x, objectProjection.y, objectPos.x, objectPos.y));
    Position2D r = Position2D(robotPos).transform_fcs2rcs(z);
    float x = r.x;
    float y = r.y;
    float vo = objectSpeed;
    float vr = robotTypicalSpeed;
    //TRACE("x=%6.2f y=%6.2f", x, y);
    
    // construct and solve quadratic equation
    // NOTE: if we would include a constant deceleration for the object, we would get a 4th-order polynomial ...
    // this would require a robust solver, for instance gsl_poly_complex_solve (http://www.gnu.org/software/gsl/)
    float a = (vo*vo - vr*vr);
    float b = -2 * y * vo;
    float c = (x*x + y*y);
    float D = b*b - 4*a*c;
    //TRACE("D=%6.2f", D);
    if (D <= 0)
    {
        // no solution: object goes too fast / distance is too large
        success = false;
        return;
    }
    
    // take smallest positive solution
    float t1 = (-b - sqrt(D)) / (2*a);
    float t2 = (-b + sqrt(D)) / (2*a);
    if (t1 < 0 && t2 < 0)
    {
        success = false;
        //TRACE("only negative solutions");
        return;
    }
    timeNeeded = t2;
    if (t1 > 0) timeNeeded = t1;
    if (t2 > 0 && t2 < t1) timeNeeded = t2;
    
    // prevent robot from moving forward w.r.t. ball ?
    if (!allowMovingForward && (y > 0))
    {
        timeNeeded = y / objectSpeed;
    }
    
    // calculate target
    Vector3D v = objectPos + objectVel * timeNeeded;
    targetPos = Position2D(v.x, v.y, z.phi);
    targetVel.x = objectVel.x; // because we assume no drag / deceleration
    targetVel.y = objectVel.y;
    //TRACE("t=%6.2f", timeNeeded);
}

