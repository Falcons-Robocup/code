 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

#include <ros/ros.h>
#include <sys/time.h>

// comment the next line in order to use WorldModel get_position instead of simulated position
//#define USE_SIMULATOR_PATHPLANNING_WORKAROUND

// uncomment the next line in order to use clipped speed/acceleration on robot
#define PATH_PLANNING_LOWSPEED

// uncomment the next line in order to disable angular pathplanning
//#define PATH_PLANNING_XY_ONLY

// compile time flag used to make services persistent.
// NEEDED FOR PERFORMANCE
#define USE_SERVICE_PERSISTENCY true


// wait for service but also trace start/end
void wait_for_service_with_tracing(std::string servicename);

// Main facilities for robot numbers
bool isSimulatedEnvironment();
int getRobotNumber();
bool isGoalKeeper();
char getTeamChar(); // A or B
int getHops(); // to separate real mode network communication from simulation (keep all traffic local)

// figure out the process id, as defined in processMapping
std::string getProcessId();

// get output of process
std::string systemStdout(std::string cmd, int bufferLimit = 512) ;

// runtime tracing using TRACE() macro
#include "tracer.hpp"
// see cDiagnosticsEvents.hpp for TRACE_ERROR and TRACE_INFO wrappers

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

// to have all motion control loops operate on the same frequency [Hz]
#define MOTION_FREQUENCY (20.0)
// obstacles don't need to be updated that fast
#define OBSTACLE_FREQUENCY (10.0)
// used in PathPlanning and simrobot
#define SYNC_FREQUENCY (20.0)
#define SYNC_FREQUENCY_MIXED_TEAM (20.0)

// Define the radius of a robot
// Needed for pathplanning and teamplay
#define ROBOT_RADIUS 75.0 / 100.0 / 2.0 // 75cm diameter = 37.5cm radius = 0.375

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
float calc_distance(float x1, float y1, float x2, float y2);
double calc_distance(  Position2D p1, Position2D p2 );
double calc_distance(  Point2D p1, Point2D p2);

// line/point geometry utils
float restrictValue( float a, float minA, float maxA );  //returns restricted value  within given limits
bool isValueInRange( float a, float minA, float maxA );  //check if value falls in min-max range

void calculate_line_equation(Vector2D v1, Vector2D v2, double &a, double &b, double &c);
double calc_distance_point_line(Vector2D p, double a, double b, double c);

double diff_seconds(timeval t1, timeval t2);

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

/* Generic function to load yaml, optionally simulation- or robot-specific 
 * Example 1: 
 *   when key="heartBeat", it will call rosparam load falcons/config/heartBeat.yaml 
 *   in simulation mode it will take heartBeatSim.yaml if existing
 */
void loadConfig(std::string key);
std::string determineConfig(std::string key);
std::string configFolder(); // typcially /home/robocup/falcons/code/config

#endif /* FALCONSCOMMON_H_ */
