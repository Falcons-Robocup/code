// Copyright 2015-2021 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: jfeitsma
 * Creation: 2015-01-25
 *
 * Oriented vector2d with coordinate transformations.
 * FCS: field coordinate system
 * RCS: robot coordinate system
 */

#ifndef INCLUDED_POSITION2D_HPP
#define INCLUDED_POSITION2D_HPP

#include <stdio.h>
#include <sstream>
#include <string>
#include "vector2d.hpp"


// angle projections
float project_angle_0_2pi(float angle);
double project_angle_0_2pi(double angle);
float project_angle_mpi_pi(float angle);
double project_angle_mpi_pi(double angle);


class Position2D; // forward declaration



// differences w.r.t. Position2D:
// * angular speed is not automatically clipped between [0,2pi], for position it is
// * transformations do not affect angular speed, for position they do
class Velocity2D
{
    private:
    char     buf[80]; // for 2str

    public:
    double   x;
    double   y;
    double   phi;

        Velocity2D()
    {
        x = 0.0;
        y = 0.0;
        phi = 0.0;
    }

    Velocity2D(double xx, double yy, double pp)
    {
        x = xx;
        y = yy;
        phi = pp;
    }

    Vector2D xy() const
    {
        return Vector2D(x, y);
    }

    char *tostr()
    {
        sprintf(buf, "vel=(%8.4f, %8.4f, %8.4f)", x, y, phi);
        return buf;
    }

    std::string to_string() const
    {
        std::ostringstream msg;
        msg << "vel=(" << x << ", " << y << ", " << phi << ")";
        return msg.str();
    }

    double size() const
    {
        return sqrt(x*x+y*y);
    }

    Velocity2D& operator +=(const Velocity2D& other)
    {
        x += other.x;
        y += other.y;
        phi += other.phi;
        return (*this);
    }

    Velocity2D& operator -=(const Velocity2D& other)
    {
        x -= other.x;
        y -= other.y;
        phi -= other.phi;
        return (*this);
    }

    Velocity2D operator -(void) const
    {
        return (Velocity2D(-x, -y, -phi));
    }

    Velocity2D operator +(const Velocity2D& other) const
    {
        return (Velocity2D(x + other.x, y + other.y, phi + other.phi));
    }

    Velocity2D operator -(const Velocity2D& other) const
    {
        return (Velocity2D(x - other.x, y - other.y, phi - other.phi));
    }

    Velocity2D operator *(float f) const
    {
        return Velocity2D(x * f, y * f, phi * f);
    }

    Velocity2D operator /(float f) const
    {
        return (*this) * (1.0 / f);
    }

    Velocity2D& transform_fcs2rcs(const Position2D& robotpos);
    Velocity2D& transform_rcs2fcs(const Position2D& robotpos);
    Velocity2D& transform_fcs2acs(bool playing_left_to_right);
    Velocity2D& transform_acs2fcs(bool playing_left_to_right);
    // these implementations are in cFalconsCommon.cpp


}; // class Velocity2D


class Position2D
{
    private:
    char     buf[80]; // for 2str

    public:
    double   x;
    double   y;
    double   phi;

    Position2D() {
        x = 0.0;
        y = 0.0;
        phi = 0.0;
    }

    Position2D(Position2D const &other)
    {
        x = other.x;
        y = other.y;
        phi = other.phi;
    }

    Position2D(double xx, double yy, double pp) {
        x = xx;
        y = yy;
        phi = project_angle_0_2pi(pp);
    }

    Position2D& operator= (const Position2D& other)
    {
        if (this != &other)
        {
            x = other.x;
            y = other.y;
            phi = other.phi;
        }
        return *this;
    }


    Vector2D xy() const    {
        return Vector2D(x, y);
    }

    char *tostr() {
        sprintf(buf, "pos=(%8.4f, %8.4f, %8.4f)", x, y, phi);
        return buf;
    }

    std::string to_string() const
    {
        std::ostringstream msg;
        msg << "pos=(" << x << ", " << y << ", " << phi << ")";
        return msg.str();
    }

    double size() const {
        return sqrt(x*x+y*y);
    }

    Position2D& operator +=(const Position2D& other) {
        x += other.x;
        y += other.y;
        phi += other.phi;
        phi = project_angle_0_2pi(phi);
        return (*this);
    }

    Position2D& operator -=(const Position2D& other) {
        x -= other.x;
        y -= other.y;
        phi -= other.phi;
        phi = project_angle_0_2pi(phi);
        return (*this);
    }

    Position2D operator -(void) const {
        return (Position2D(-x, -y, -phi));
    }

    Position2D operator +(const Position2D& other) const {
        return (Position2D(x + other.x, y + other.y, phi + other.phi));
    }

    Position2D operator -(const Position2D& other) const {
        return (Position2D(x - other.x, y - other.y, phi - other.phi));
    }

    Position2D operator *(float scale) const {
        return (Position2D(x * scale, y * scale, phi * scale));
    }

    void update(const Velocity2D& speed, double dt) {
        x += speed.x * dt;
        y += speed.y * dt;
        phi += speed.phi * dt;
        phi = project_angle_0_2pi(phi);
    }

    Position2D& transform_fcs2rcs(const Position2D& robotpos);
    Position2D& transform_rcs2fcs(const Position2D& robotpos);
    Position2D& transform_fcs2acs(bool playing_left_to_right);
    Position2D& transform_acs2fcs(bool playing_left_to_right);
    // these implementations are in cFalconsCommon.cpp


}; // class Position2D



#endif

