 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cFalconsCommon.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Jan Feitsma
 */

#include "ext/FalconsCommon.h"

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <pwd.h>
#include <cstdio>
#include <string.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <unistd.h>

// tracer singleton instance
Tracer *Tracer::s_instance = 0;

// Source: http://math.stackexchange.com/questions/1201337/finding-the-angle-between-two-points
double angle_between_two_points_0_2pi(double x1, double y1, double x2, double y2)
{

    double angle = std::atan2(y2 - y1, x2 - x1);
    return project_angle_0_2pi(angle);
}

double project_angle_0_2pi(double angle)
{
	while (angle < 0) angle += 2*M_PI;
	while (angle > 2*M_PI) angle -= 2*M_PI;
	return angle;
}

float project_angle_0_2pi(float angle)
{
	while (angle < 0) angle += 2*M_PI;
	while (angle > 2*M_PI) angle -= 2*M_PI;
	return angle;
}

double project_angle_mpi_pi(double angle)
{
	while (angle < -M_PI) angle += 2*M_PI;
	while (angle > M_PI) angle -= 2*M_PI;
	return angle;
}

float project_angle_mpi_pi(float angle)
{
	while (angle < -M_PI) angle += 2*M_PI;
	while (angle > M_PI) angle -= 2*M_PI;
	return angle;
}


// position type transformations
Position2D getPosition2D( const geometry::Pose2D& pose)
{
	Position2D myPosition;
	myPosition.x= pose.x;
	myPosition.y= pose.y;
	return ( myPosition );
}

Position2D getPosition2D( const Point3D& point3d)
{
	Position2D myPosition;
	myPosition.x= (double) point3d.x;
	myPosition.y= (double) point3d.y;
	return ( myPosition );
}

// coordinate transformations
Velocity2D& Velocity2D::transform_fcs2rcs(const Position2D& robotpos)
{
	double angle = (M_PI_2 - robotpos.phi);
	Vector2D xynew = Vector2D(x, y).rotate(angle);
	x = xynew.x;
	y = xynew.y;
	// do not update vphi
	return (*this);
}

Velocity2D& Velocity2D::transform_rcs2fcs(const Position2D& robotpos)
{
	double angle = -(M_PI_2 - robotpos.phi);
	Vector2D xynew = Vector2D(x, y).rotate(angle);
	x = xynew.x;
	y = xynew.y;
	// do not update vphi
	return (*this);
}

Position2D& Position2D::transform_fcs2rcs(const Position2D& robotpos)
{  
    // first ttranslate, then rotate
	double angle = (M_PI_2 - robotpos.phi);
	Vector2D xynew = (Vector2D(x, y) - robotpos.xy()).rotate(angle);
	x = xynew.x;
	y = xynew.y;
	phi = phi + angle;
	phi = project_angle_0_2pi(phi);
	return (*this);
}

Position2D& Position2D::transform_rcs2fcs(const Position2D& robotpos)
{
    // first rotate, then translate
	double angle = - (M_PI_2 - robotpos.phi);
	Vector2D xyrot = (Vector2D(x, y)).rotate(angle);
	x = xyrot.x + robotpos.x;
	y = xyrot.y + robotpos.y;
	phi = phi + angle;
	phi = project_angle_0_2pi(phi);
	return (*this);
}

Position2D& Position2D::transform_fcs2acs(bool playing_left_to_right)
{
    // no change in case we are playing left to right
    if (!playing_left_to_right)
    {
        // rotate by half a circle
        x = -x;
        y = -y;
    	phi = project_angle_0_2pi(phi + M_PI);
    }
	return (*this);
}

Position2D& Position2D::transform_acs2fcs(bool playing_left_to_right)
{
    // no change in case we are playing left to right
    if (!playing_left_to_right)
    {
        // rotate by half a circle
        x = -x;
        y = -y;
    	phi = project_angle_0_2pi(phi + M_PI);
    }
	return (*this);
}

Velocity2D& Velocity2D::transform_fcs2acs(bool playing_left_to_right)
{
    // no change in case we are playing left to right
    if (!playing_left_to_right)
    {
        // rotate by half a circle, xy only
        x = -x;
        y = -y;
    }
	return (*this);
}

Velocity2D& Velocity2D::transform_acs2fcs(bool playing_left_to_right)
{
    // no change in case we are playing left to right
    if (!playing_left_to_right)
    {
        // rotate by half a circle, xy only
        x = -x;
        y = -y;
    }
	return (*this);
}

int getHops()
{
    if (isSimulatedEnvironment())
    {
        return 0;
    }
    return 5; 
    // one hop between coach laptop and router; one hop between robot and router
    // add a few more hops for situations like wired to our mobile rack
}

bool isSimulatedEnvironment()
{
   bool isSimulated = false;
   std::string env_simulated;
   try
   {
      char* env_sim = getenv("SIMULATED");
      env_simulated = std::string(env_sim);
   }
   catch (...)
   {
       //std::cout << "Environment variable SIMULATED not found." << std::endl; // suppress spam
       // this is an error state, suggesting something weird in scripting or stand-alone deployment
       // hence we SET the simulated flag, so at least traffic will remain local
       isSimulated = true;
   }

   if (env_simulated == "1")
   {
       isSimulated = true;
   }

   return isSimulated;
}

int getRobotNumber()
{
   int robotNumber = 0;
   std::string robot_namespace;
   size_t found_index = std::string::npos;

   if (getenv("ROS_NAMESPACE") != NULL)
   {
	   robot_namespace = getenv("ROS_NAMESPACE");
   }

   found_index = robot_namespace.find("robot");
   if (found_index != std::string::npos)
   {
	   robotNumber = atoi(&robot_namespace.at(found_index + 5));
   }

   // try dedicated env var if ROS_NAMESPACE did not tell us anything yet
   if (robotNumber == 0)
   {
      if (getenv("TURTLE5K_ROBOTNUMBER") != NULL)
      {
         robotNumber = fromStr(getenv("TURTLE5K_ROBOTNUMBER"));
      }
   }

   return robotNumber;
}

bool isGoalKeeper()
{
   return (getRobotNumber() == 1);
}

char getTeamChar() // A or B
{
   char teamChar = 'A';
   std::string robot_namespace;
   size_t found_index = std::string::npos;

   if (getenv("ROS_NAMESPACE") != NULL)
   {
	   robot_namespace = getenv("ROS_NAMESPACE");
   }

   found_index = robot_namespace.find("team");
   if (found_index != std::string::npos)
   {
	   teamChar = robot_namespace.at(found_index + 4);
   }

   return teamChar;
}

// handy function to return output of a command
// source: http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
std::string exec(const char* cmd) {
    boost::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    boost::algorithm::trim(result);
    return result;
}

std::string getProcessId()
{
    return exec("getProcessId");
}


// wait for service but also trace start/end
void wait_for_service_with_tracing(std::string servicename)
{
    TRACE("waiting for service %s to come online...", servicename.c_str());
    ros::service::waitForService(servicename);
    TRACE("service %s has come online!", servicename.c_str());
}

float restrictValue( float a, float minA, float maxA )  //returns restricted value position within given limits
{
	if( a > maxA )
		return maxA;
	if( a < minA )
		return minA;
	return a;
}

bool isValueInRange( float a, float minA, float maxA )  //check if value falls in min-max range
{
	if( a > maxA )
		return false;
	if( a < minA )
		return false;
	return true;
}

double calc_angle(double dX, double dY)
{
    // calculates the absolute angle
    // looking from X1,Y1 towards X2,Y2
    double S = 0.0;
    double th = 0.0;
    S = calc_hypothenusa(dX, dY);

    if (dY < 0) {
        if (dX == 0) {
            th = M_PI_2;
        } else {
            th = acos(-dX / S);
        }
    } else if (dY > 0) {
        if (dX == 0) {
            th = 3.0 / 2.0 * M_PI;
        } else {
            th = 2 * M_PI - acos(-dX / S);
        }
    } else if (dY == 0) {
        if (dX > 0) {
            th = 0;
        } else {
            th = M_PI;
        }
    }
    return th;
}

float calc_angle(float dX, float dY)
{
    // calculates the absolute angle
    // looking from X1,Y1 towards X2,Y2
    float S = 0.0;
    float th = 0.0;

    S = calc_hypothenusa(dX, dY);

    if (dY < 0) {
        if (dX == 0) {
            th = M_PI_2;
        } else {
            th = acos(-dX / S);
        }
    } else if (dY > 0) {
        if (dX == 0) {
            th = 3.0 / 2.0 * M_PI;
        } else {
            th = 2 * M_PI - acos(-dX / S);
        }
    } else if (dY == 0) {
        if (dX > 0) {
            th = 0;
        } else {
            th = M_PI;
        }
    }
    return th;
}

// transform a line segment (v1,v2)
// to a line equation a*x + b*y + c = 0
void calculate_line_equation(Vector2D v1, Vector2D v2, double &a, double &b, double &c)
{
    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    // dx*y - dx*v1.y - dy*x - dy*v1.x = 0
    a = -dy;
    b = dx;
    c = -1.0 * (dx*v1.y - dy*v1.x);
}

double calc_distance_point_line(Vector2D p, double a, double b, double c)
{
    if (a != 0 || b != 0)
    {
       return (a * p.x +  b * p.y + c) / sqrt(a*a + b*b);
    }
    else
    {
       return 0;
    }
}

double calc_hypothenusa(double dX, double dY)
{
    // Function to calculate the Hypothenusa (Schuine zijde)
    double S = 0.0;
    S = sqrt(pow(dX, 2) + pow(dY, 2));
    return S;
}

float calc_hypothenusa(float dX, float dY)
{
    // Function to calculate the Hypothenusa (Schuine zijde)
    float S = 0.0;
    S = sqrt(pow(dX, 2) + pow(dY, 2));
    return S;
}

double calc_distance( double x1, double y1, double x2, double y2)
{
	double dX=x2-x1;
	double dY=y2-y1;

	return calc_hypothenusa( dX, dY );
};

float calc_distance( float x1, float y1, float x2, float y2)
{
	float dX=x2-x1;
	float dY=y2-y1;

	return calc_hypothenusa( dX, dY );
};

double calc_distance( Position2D p1, Position2D p2 )
{
	double dX=p2.x - p1.x;
	double dY=p2.y - p1.y;

	return calc_hypothenusa( dX, dY );
}

double calc_distance( Point2D p1, Point2D p2 )
{
	double dX=p2.x - p1.x;
	double dY=p2.y - p1.y;

	return calc_hypothenusa( dX, dY );
}

connectionType GetPrimaryIp(char* buffer, size_t buflen)
{
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;
    bool LANFound = false;
    connectionType retVal = connectionType::INVALID;

    assert(buflen >= INET_ADDRSTRLEN);
    getifaddrs(&ifAddrStruct);

    if(!LANFound)
    {
        for (ifa = ifAddrStruct; ((ifa != NULL) && (!LANFound)); ifa = ifa->ifa_next)
        {
            /* Filter IPV4 addresses */
            if (ifa->ifa_addr->sa_family == AF_INET)
            {

                tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
                char addressBuffer[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

                if(strstr(ifa->ifa_name, "eth") != NULL)
                {
                    /* Copy IP address to output */
                    inet_ntop(AF_INET, tmpAddrPtr, buffer, buflen);

                    /* Alright meow! It is time to stop now Mack */
                    LANFound = true;
                    retVal = connectionType::LAN;
                    TRACE("Using %s IP Address %s\n", ifa->ifa_name, addressBuffer);
                }
            }
        }
    }

    for (ifa = ifAddrStruct; ((ifa != NULL) && (!LANFound)); ifa = ifa->ifa_next)
    {
        /* Filter IPV4 addresses */
        if (ifa->ifa_addr->sa_family == AF_INET)
        {

            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

            if(strstr(ifa->ifa_name, "wlan") != NULL)
            {
                /* Copy IP address to output */
                inet_ntop(AF_INET, tmpAddrPtr, buffer, buflen);

                /* Alright meow! It is time to stop now Mack */
                LANFound = true;
                retVal = connectionType::WAN;
                TRACE("Using %s IP Address %s\n", ifa->ifa_name, addressBuffer);
            }
        }
    }

    for (ifa = ifAddrStruct; ((ifa != NULL) && (!LANFound)); ifa = ifa->ifa_next)
    {
        /* Filter IPV4 addresses */
        if (ifa->ifa_addr->sa_family == AF_INET)
        {

            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

            if(strstr(ifa->ifa_name, "usb") != NULL)
            {
                /* Copy IP address to output */
                inet_ntop(AF_INET, tmpAddrPtr, buffer, buflen);

                /* Alright meow! It is time to stop now Mack */
                LANFound = true;
                retVal = connectionType::USB;
                TRACE("Using %s IP Address %s\n", ifa->ifa_name, addressBuffer);
            }
        }
    }

    if(!LANFound)
	{
		for (ifa = ifAddrStruct; ((ifa != NULL) && (!LANFound)); ifa = ifa->ifa_next)
		{
			/* Filter IPV4 addresses */
			if (ifa->ifa_addr->sa_family == AF_INET)
			{

				tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
				char addressBuffer[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

				if(strstr(ifa->ifa_name, "lo") != NULL)
				{
					/* Copy IP address to output */
					inet_ntop(AF_INET, tmpAddrPtr, buffer, buflen);

					/* Alright meow! It is time to stop now Mack */
					LANFound = true;
					retVal = connectionType::LOOPBACK;
					TRACE("Using %s IP Address %s\n", ifa->ifa_name, addressBuffer);
				}
			}
		}
	}

    if(!LANFound)
    {
    	TRACE("No adapter found for fetching an IP address");
    	ROS_ERROR("No adapter found for fetching an IP address");
    }

    if (ifAddrStruct!=NULL)
    {
        freeifaddrs(ifAddrStruct);
    }

    return retVal;
}

connectionType GetPrimaryConnectionType()
{
	struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;
    connectionType retVal = connectionType::INVALID;

    getifaddrs(&ifAddrStruct);

    if(retVal == connectionType::INVALID)
    {
        for (ifa = ifAddrStruct; ((ifa != NULL) && (retVal == connectionType::INVALID)); ifa = ifa->ifa_next)
        {
            /* Filter IPV4 addresses */
            if (ifa->ifa_addr->sa_family == AF_INET)
            {

                tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
                char addressBuffer[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

                if(strstr(ifa->ifa_name, "eth") != NULL)
                {
                    /* Alright meow! It is time to stop now Mack */
                    retVal = connectionType::LAN;
                }
            }
        }
    }

    for (ifa = ifAddrStruct; ((ifa != NULL) && (retVal == connectionType::INVALID)); ifa = ifa->ifa_next)
    {
        /* Filter IPV4 addresses */
        if (ifa->ifa_addr->sa_family == AF_INET)
        {

            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

            if(strstr(ifa->ifa_name, "wlan") != NULL)
            {
                /* Alright meow! It is time to stop now Mack */
                retVal = connectionType::WAN;
            }
        }
    }

    for (ifa = ifAddrStruct; ((ifa != NULL) && (retVal == connectionType::INVALID)); ifa = ifa->ifa_next)
    {
        /* Filter IPV4 addresses */
        if (ifa->ifa_addr->sa_family == AF_INET)
        {

            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

            if(strstr(ifa->ifa_name, "usb") != NULL)
            {
                /* Alright meow! It is time to stop now Mack */
                retVal = connectionType::USB;
            }
        }
    }

    if(retVal == connectionType::INVALID)
	{
		for (ifa = ifAddrStruct; ((ifa != NULL) && (retVal == connectionType::INVALID)); ifa = ifa->ifa_next)
		{
			/* Filter IPV4 addresses */
			if (ifa->ifa_addr->sa_family == AF_INET)
			{

				tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
				char addressBuffer[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

				if(strstr(ifa->ifa_name, "lo") != NULL)
				{
					/* Alright meow! It is time to stop now Mack */
					retVal = connectionType::LOOPBACK;
					TRACE("Using %s IP Address %s\n", ifa->ifa_name, addressBuffer);
				}
			}
		}
	}

    if(retVal == connectionType::INVALID)
    {
    	TRACE("No adapter found for fetching an IP address");
    	ROS_ERROR("No adapter found for fetching an IP address");
    }

    if (ifAddrStruct!=NULL)
    {
        freeifaddrs(ifAddrStruct);
    }

    return retVal;
}

double diff_seconds(timeval t1, timeval t2)
{
    return  (t1.tv_sec - t2.tv_sec)
            + ((t1.tv_usec - t2.tv_usec) / 1000000.0);
}

/* Double comparison */
bool doubleIsEqual(double a, double b)
{
    return fabs(a - b) < FLT_EPSILON;
}

bool floatIsEqual(float a, float b)
{
    return fabs(a - b) < DBL_EPSILON;
}

// handy function to return output of a command
// source: http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
std::string systemStdout(std::string cmd, int bufferLimit) 
{
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    boost::algorithm::trim(result);
    // limit string length
    if ((int)result.size() > bufferLimit)
    {
        result = result.substr(0, bufferLimit);
    }
    return result;
}

std::string configFolder()
{
	struct passwd *pw = NULL;
    pw = getpwuid(getuid());
    return std::string(pw->pw_dir) + "/falcons/code/config";
}

/* Generic function to load yaml, optionally simulation- or robot-specific 
 * Example 1: 
 *   when key="heartBeat", it will call rosparam load falcons/config/heartBeat.yaml 
 *   in simulation mode it will take heartBeatSim.yaml if existing
 * Example 2: 
 *   when key="BallHandlers", and current robot number is 2, 
 *   then if BallHandlersR2.yaml exists, it will be loaded
 *
 * This provides for a consistent way to manage configuration without having to rebuild components when switching 
 * from generic to robot-specific configuration.
 */
std::string determineConfig(std::string key)
{
    // determine config folder
    std::string cfgFolder = configFolder();
    std::string fileExtension = ".yaml";
    // determine yaml file to load
    std::string target = key;
    if (isSimulatedEnvironment()) // simulator override?
    {
        std::string candidate = key + "Sim";
        std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            target = candidate;
        }
        // test robot-specific
        candidate = key + "SimR" + boost::lexical_cast<std::string>(getRobotNumber());
        candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            target = candidate;
        }
    }
    else // not simulator
    {
        // test robot-specific
        std::string candidate = key + "R" + boost::lexical_cast<std::string>(getRobotNumber());
        std::string candidateFull = cfgFolder + "/" + candidate + fileExtension;
        if (boost::filesystem::exists(candidateFull))
        {
            target = candidate;
        }
    }
    // final check and return
    std::string result = cfgFolder + "/" + target + fileExtension;
    if (!boost::filesystem::exists(result))
    {
        TRACE("ERROR: could not resolve yaml file for %s", key.c_str());
    }
    else
    {
        TRACE("resolved yaml file %s -> %s", key.c_str(), result.c_str());
    }
    return result;
}

void loadConfig(std::string key)
{
    // build the command string
    std::string configFileCmd("rosparam load ");
    configFileCmd.append(determineConfig(key));
    // run the command
    TRACE("about to run system command: '%s'", configFileCmd.c_str());
    int r = system(configFileCmd.c_str());
    TRACE("got return code %d from system command '%s'", r, configFileCmd.c_str());
}


