 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: jfeitsma
 * Creation: 2014-04-27
 *
 * Utility class: Vector2D. Vector operations such as dot product, etc.
 * Fetched from http://www.terathon.com/code/vector2d.html
 */

#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

#include <math.h>


class Vector2D
{
    public:

		double   x;
		double   y;

        Vector2D() { x = 0.0; y = 0.0; }

        Vector2D(double r, double s)
        {
            x = r;
            y = s;
        }

        Vector2D& Set(double r, double s)
        {
            x = r;
            y = s;
            return (*this);
        }

        double& operator [](long k)
        {
            return ((&x)[k]);
        }

        double size() const
        {
            return sqrt(x*x+y*y);
        }

        const double& operator [](long k) const
        {
            return ((&x)[k]);
        }

        Vector2D& operator +=(const Vector2D& v)
        {
            x += v.x;
            y += v.y;
            return (*this);
        }

        Vector2D& operator -=(const Vector2D& v)
        {
            x -= v.x;
            y -= v.y;
            return (*this);
        }

        Vector2D& operator *=(double t)
        {
            x *= t;
            y *= t;
            return (*this);
        }

        Vector2D& operator /=(double t)
        {
            double f = 1.0F / t;
            x *= f;
            y *= f;
            return (*this);
        }

        Vector2D& operator &=(const Vector2D& v)
        {
            x *= v.x;
            y *= v.y;
            return (*this);
        }

        Vector2D operator -(void) const
        {
            return (Vector2D(-x, -y));
        }

        Vector2D operator +(const Vector2D& v) const
        {
            return (Vector2D(x + v.x, y + v.y));
        }

        Vector2D operator -(const Vector2D& v) const
        {
            return (Vector2D(x - v.x, y - v.y));
        }

        Vector2D operator *(double t) const
        {
            return (Vector2D(x * t, y * t));
        }

        Vector2D operator /(double t) const
        {
            double f = 1.0F / t;
            return (Vector2D(x * f, y * f));
        }

        double operator *(const Vector2D& v) const
        {
            return (x * v.x + y * v.y);
        }

        Vector2D operator &(const Vector2D& v) const
        {
            return (Vector2D(x * v.x, y * v.y));
        }

        bool operator ==(const Vector2D& v) const
        {
            return ((x == v.x) && (y == v.y));
        }

        bool operator !=(const Vector2D& v) const
        {
            return ((x != v.x) || (y != v.y));
        }

        Vector2D& normalize(double factor = 1.0)
        {
            *this /= sqrt(x * x + y * y);
            return *this *= factor;
        }

        Vector2D normalized(double factor = 1.0)
        {
        	return ((*this / sqrt(x * x + y * y)) * factor);
        }

        Vector2D& rotate(double angle)
        {
            float s = sin(angle);
            float c = cos(angle);
            float nx = c * x - s * y;
            float ny = s * x + c * y;
            x = nx;
            y = ny;
            return (*this);
        }

        double CrossProduct(const Vector2D & v2) const
        {
               return (x*v2.y) - (y*v2.x);
        }
};


class Point2D : public Vector2D
{
    public:

        Point2D() {}

        Point2D(double r, double s) : Vector2D(r, s) {}

        Point2D& operator &=(const Vector2D& v)
        {
            x = v.x;
            y = v.y;
            return (*this);
        }

        Point2D& operator *=(double t)
        {
            x *= t;
            y *= t;
            return (*this);
        }

        Point2D& operator /=(double t)
        {
            double f = 1.0F / t;
            x *= f;
            y *= f;
            return (*this);
        }

        Point2D operator -(void) const
        {
            return (Point2D(-x, -y));
        }

        Point2D operator +(const Vector2D& v) const
        {
            return (Point2D(x + v.x, y + v.y));
        }

        Point2D operator -(const Vector2D& v) const
        {
            return (Point2D(x - v.x, y - v.y));
        }

        Vector2D operator -(const Point2D& p) const
        {
            return (Vector2D(x - p.x, y - p.y));
        }

        Point2D operator *(double t) const
        {
            return (Point2D(x * t, y * t));
        }

        Point2D operator /(double t) const
        {
            double f = 1.0F / t;
            return (Point2D(x * f, y * f));
        }
};


inline Vector2D operator *(double t, const Vector2D& v)
{
    return (Vector2D(t * v.x, t * v.y));
}


inline double vectordot(const Vector2D& v1, const Vector2D& v2)
{
    return (v1 * v2);
}

inline double vectorsize(const Vector2D& v)
{
    return v.size();
}





#endif

