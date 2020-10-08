 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * $Id: FalconsCommon.h 1966 2015-07-07 20:10:12Z jfei $
 *
 *  Created on: Sep 11, 2014
 *      Author: Jan Feitsma
 */

#ifndef FALCONSCOMMON_H_
#define FALCONSCOMMON_H_

#include <string>


int getHops(); // to separate real mode network communication from simulation (keep all traffic local)

// figure out the process id, as defined in processMapping
std::string getProcessId();

// get output of process
std::string systemStdout(std::string cmd, int bufferLimit = 512) ;

// Vector2D and Position2D class and operations
// A Position2D is a Vector2D with an orientation
#include "vector2d.hpp"
#include "position2d.hpp"
#include "area2D.hpp"

#include "vector3d.hpp"
#include "pose2d.hpp"

// Finite State Machine
#include "fsm.hpp"

// Circle class and operations
#include "circle.hpp"

// string-to-anything and anything-to-string conversions
#include "convert.hpp"

// Define the radius of a robot
#define ROBOT_RADIUS 0.26 // official limit according to MSL rulebook

#define MAX_ROBOTS 9
#define COACH_AGENTID 0

// transformations
Position2D getPosition2D( const geometry::Pose2D& pose);
Position2D getPosition2D( const Point3D& point3d);

double angle_between_two_points_0_2pi(double x1, double y1, double x2, double y2);

double angle_between_two_points_0_2pi(double x1, double y1, double x2, double y2);

// angle projections
float project_angle_0_2pi(float angle);
double project_angle_0_2pi(double angle);

float project_angle_mpi_pi(float angle);
double project_angle_mpi_pi(double angle);

// Distance and angle calculations
double calc_hypothenusa(double dX, double dY);
float calc_hypothenusa(float dX, float dY);

double calc_angle(double dX, double dY);
float calc_angle(float dX, float dY);

double calc_distance(double x1, double y1, double x2, double y2);
double calc_distance(  Position2D p1, Position2D p2 );
double calc_distance(  Point2D p1, Point2D p2);

// line/point geometry utils
float restrictValue( float a, float minA, float maxA );  //returns restricted value  within given limits
bool isValueInRange( float a, float minA, float maxA );  //check if value falls in min-max range

void calculate_line_equation(Vector2D v1, Vector2D v2, double &a, double &b, double &c);
double calc_distance_point_line(Vector2D p, double a, double b, double c);

bool intersect(Vector2D const &a1, Vector2D const &a2, Vector2D const &b1, Vector2D const &b2, Vector2D &result);

/* Fetch correct ip-address
 * First search for wlan
 * Secondly search for eth
 */
enum class connectionType
{
    INVALID,
    LAN,
    WAN,
    LOOPBACK,
    USB
};

connectionType GetPrimaryIp(char* buffer, size_t buflen);
connectionType GetPrimaryConnectionType();

// value clipping
template <typename T> void clip(T &v, T const & lo, T const & up)
{
    if (v < lo) v = lo;
    if (v > up) v = up;
}

/* Double comparison */
bool doubleIsEqual(double a, double b);
bool floatIsEqual(float a, float b);


#endif /* FALCONSCOMMON_H_ */