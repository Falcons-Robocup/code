// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * falconsCommonLegacy.hpp (previously cFalconsCommon.cpp)
 *
 *  Created on: Sep 11, 2014
 *      Author: Jan Feitsma
 */

#include "ext/falconsCommonLegacy.hpp"

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
#include <boost/lexical_cast.hpp>
#include <unistd.h>

// TODO most of this stuff should be distributed to dedicated packages,
// for instance angle utilities, Position2D, Velocity2D etc. to geometry


// Source: http://math.stackexchange.com/questions/1201337/finding-the-angle-between-two-points
double angle_between_two_points_0_2pi(double x1, double y1, double x2, double y2)
{

    double angle = std::atan2(y2 - y1, x2 - x1);
    return project_angle_0_2pi(angle);
}

double project_angle_0_2pi(double angle)
{
    // sanity checks
    if ((angle > 100) || (angle < -100))
    {
        throw std::runtime_error("angle out of bounds");
    }
    while (angle < 0) angle += 2*M_PI;
    while (angle > 2*M_PI) angle -= 2*M_PI;
    return angle;
}

float project_angle_0_2pi(float angle)
{
    // sanity checks
    if ((angle > 100) || (angle < -100))
    {
        throw std::runtime_error("angle out of bounds");
    }
    while (angle < 0) angle += 2*M_PI;
    while (angle > 2*M_PI) angle -= 2*M_PI;
    return angle;
}

double project_angle_mpi_pi(double angle)
{
    // sanity checks
    if ((angle > 100) || (angle < -100))
    {
        throw std::runtime_error("angle out of bounds");
    }
    while (angle < -M_PI) angle += 2*M_PI;
    while (angle > M_PI) angle -= 2*M_PI;
    return angle;
}

float project_angle_mpi_pi(float angle)
{
    // sanity checks
    if ((angle > 100) || (angle < -100))
    {
        throw std::runtime_error("angle out of bounds");
    }
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

// based on http://stackoverflow.com/a/385355
bool intersect(Vector2D const &a1, Vector2D const &a2, Vector2D const &b1, Vector2D const &b2, Vector2D &result)
{
    float x12 = a1.x - a2.x;
    float x34 = b1.x - b2.x;
    float y12 = a1.y - a2.y;
    float y34 = b1.y - b2.y;

    float c = x12 * y34 - y12 * x34;

    if (fabs(c) < 0.01)
    {
        // No intersection
        return false;
    }
    // Intersection
    float a = a1.x * a2.y - a1.y * a2.x;
    float b = b1.x * b2.y - b1.y * b2.x;

    result.x = (a * x34 - b * x12) / c;
    result.y = (a * y34 - b * y12) / c;

    return true;
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

bool ignoreIfaName(std::string ifa_name)
{
    if (ifa_name == "enp0s31f6") return true; // the port on top-side of CPU-box, located most inward, is reserved for multiCam
    return false;
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

                if((strstr(ifa->ifa_name, "en") != NULL) ||
                    (strstr(ifa->ifa_name, "eth") != NULL))
                {
                    if (!ignoreIfaName(ifa->ifa_name))
                    {
                        /* Copy IP address to output */
                        inet_ntop(AF_INET, tmpAddrPtr, buffer, buflen);

                        /* Alright meow! It is time to stop now Mack */
                        LANFound = true;
                        retVal = connectionType::LAN;
                    }
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

            if(strstr(ifa->ifa_name, "wl") != NULL)
            {
                /* Copy IP address to output */
                inet_ntop(AF_INET, tmpAddrPtr, buffer, buflen);

                /* Alright meow! It is time to stop now Mack */
                LANFound = true;
                retVal = connectionType::WAN;
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
                }
            }
        }
    }

    if(!LANFound)
    {
        printf("No adapter found for fetching an IP address");
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

                if((strstr(ifa->ifa_name, "en") != NULL) ||
                    (strstr(ifa->ifa_name, "eth") != NULL))
                {
                    if (!ignoreIfaName(ifa->ifa_name))
                    {
                        /* Alright meow! It is time to stop now Mack */
                        retVal = connectionType::LAN;
                    }
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

            if(strstr(ifa->ifa_name, "wl") != NULL)
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
                }
            }
        }
    }

    if(retVal == connectionType::INVALID)
    {
        printf("No adapter found for fetching an IP address");
    }

    if (ifAddrStruct!=NULL)
    {
        freeifaddrs(ifAddrStruct);
    }

    return retVal;
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


