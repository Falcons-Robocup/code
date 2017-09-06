 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 *  Created on: Sep 11, 2014
 *      Author: Jan Feitsma
 */

#ifndef TEAMPLAYCOMMON_H_
#define TEAMPLAYCOMMON_H_

#include "pose2d.hpp"
#include "position2d.hpp"
#include "vector2d.hpp"
#include "vector3d.hpp"

Position2D getPosition2D( const geometry::Pose2D& pose);
Position2D getPosition2D( const Point3D& point3d);

double angle_between_two_points_0_2pi(double x1, double y1, double x2, double y2);

float calc_distance(float x1, float y1, float x2, float y2);
double calc_distance(  Position2D p1, Position2D p2 );//user: cPFM
double calc_distance(  Point2D p1, Point2D p2);

bool intersect(Vector2D const &a1, Vector2D const &a2, Vector2D const &b1, Vector2D const &b2, Vector2D &result);

#endif /* TEAMPLAYCOMMON_H_ */
