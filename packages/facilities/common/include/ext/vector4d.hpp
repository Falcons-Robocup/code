 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: ekouters
 * Creation: 2015-10-22
 *
 * Utility class: Vector4D. Vector operations such as dot product, etc.
 * Fetched from http://www.terathon.com/code/vector4d.html
 */

#ifndef VECTOR4D_HPP
#define VECTOR4D_HPP

#include "vector3d.hpp"

class Vector4D
{
    public:
    
        float   x;
        float   y;
        float   z;
        float   w;
        
        Vector4D() {}
        
        Vector4D(const Vector3D& v, float u)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            w = u;
        }

        Vector4D(float r, float s, float t, float u)
        {
            x = r;
            y = s;
            z = t;
            w = u;
        }
        
        Vector4D(const Vector3D& v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            w = 0.0F;
        }
        
        Vector4D(const Point3D& p)
        {
            x = p.x;
            y = p.y;
            z = p.z;
            w = 1.0F;
        }
        
        Vector4D(const Vector2D& v)
        {
            x = v.x;
            y = v.y;
            z = 0.0F;
            w = 0.0F;
        }
        
        Vector4D(const Point2D& p)
        {
            x = p.x;
            y = p.y;
            z = 0.0F;
            w = 1.0F;
        }
        
        Vector4D& Set(const Vector3D &v, float u)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            w = u;
            return (*this);
        }
        
        Vector4D& Set(float r, float s, float t, float u)
        {
            x = r;
            y = s;
            z = t;
            w = u;
            return (*this);
        }
        
        float& operator [](long k)
        {
            return ((&x)[k]);
        }
        
        const float& operator [](long k) const
        {
            return ((&x)[k]);
        }
        
        Vector3D& GetVector3D(void)
        {
            return (*reinterpret_cast<Vector3D *>(this));
        }
        
        const Vector3D& GetVector3D(void) const
        {
            return (*reinterpret_cast<const Vector3D *>(this));
        }
        
        Point3D& GetPoint3D(void)
        {
            return (*reinterpret_cast<Point3D *>(this));
        }
        
        const Point3D& GetPoint3D(void) const
        {
            return (*reinterpret_cast<const Point3D *>(this));
        }
        
        Vector4D& operator =(const Vector3D& v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            w = 0.0F;
            return (*this);
        }
        
        Vector4D& operator =(const Point3D& p)
        {
            x = p.x;
            y = p.y;
            z = p.z;
            w = 1.0F;
            return (*this);
        }
        
        Vector4D& operator =(const Vector2D& v)
        {
            x = v.x;
            y = v.y;
            z = 0.0F;
            w = 0.0F;
            return (*this);
        }
        
        Vector4D& operator =(const Point2D& p)
        {
            x = p.x;
            y = p.y;
            z = 0.0F;
            w = 1.0F;
            return (*this);
        }
        
        Vector4D& operator +=(const Vector4D& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
            w += v.w;
            return (*this);
        }
        
        Vector4D& operator +=(const Vector3D& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
            return (*this);
        }
        
        Vector4D& operator +=(const Vector2D& v)
        {
            x += v.x;
            y += v.y;
            return (*this);
        }
        
        Vector4D& operator -=(const Vector4D& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            w -= v.w;
            return (*this);
        }
        
        Vector4D& operator -=(const Vector3D& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            return (*this);
        }
        
        Vector4D& operator -=(const Vector2D& v)
        {
            x -= v.x;
            y -= v.y;
            return (*this);
        }
        
        Vector4D& operator *=(float t)
        {
            x *= t;
            y *= t;
            z *= t;
            w *= t;
            return (*this);
        }
        
        Vector4D& operator /=(float t)
        {
            float f = 1.0F / t;
            x *= f;
            y *= f;
            z *= f;
            w *= f;
            return (*this);
        }
        
        Vector4D& operator &=(const Vector4D& v)
        {
            x *= v.x;
            y *= v.y;
            z *= v.z;
            w *= v.w;
            return (*this);
        }
        
        Vector4D operator -(void) const
        {
            return (Vector4D(-x, -y, -z, -w));
        }
        
        Vector4D operator +(const Vector4D& v) const
        {
            return (Vector4D(x + v.x, y + v.y, z + v.z, w + v.w));
        }
        
        Vector4D operator +(const Vector3D& v) const
        {
            return (Vector4D(x + v.x, y + v.y, z + v.z, w));
        }
        
        Vector4D operator +(const Vector2D& v) const
        {
            return (Vector4D(x + v.x, y + v.y, z, w));
        }
        
        Vector4D operator -(const Vector4D& v) const
        {
            return (Vector4D(x - v.x, y - v.y, z - v.z, w - v.w));
        }
        
        Vector4D operator -(const Vector3D& v) const
        {
            return (Vector4D(x - v.x, y - v.y, z - v.z, w));
        }
        
        Vector4D operator -(const Vector2D& v) const
        {
            return (Vector4D(x - v.x, y - v.y, z, w));
        }
        
        Vector4D operator *(float t) const
        {
            return (Vector4D(x * t, y * t, z * t, w * t));
        }
        
        Vector4D operator /(float t) const
        {
            float f = 1.0F / t;
            return (Vector4D(x * f, y * f, z * f, w * f));
        }
        
        float operator *(const Vector4D& v) const
        {
            return (x * v.x + y * v.y + z * v.z + w * v.w);
        }
        
        float operator *(const Vector3D& v) const
        {
            return (x * v.x + y * v.y + z * v.z);
        }
        
        float operator *(const Point3D& p) const
        {
            return (x * p.x + y * p.y + z * p.z + w);
        }
        
        float operator *(const Vector2D& v) const
        {
            return (x * v.x + y * v.y);
        }
        
        float operator *(const Point2D& p) const
        {
            return (x * p.x + y * p.y + w);
        }
        
        Vector3D operator %(const Vector3D& v) const
        {
            return (Vector3D(y * v.z - z * v.y, z * v.x - x * v.z,
                    x * v.y - y * v.x));
        }
        
        Vector4D operator &(const Vector4D& v) const
        {
            return (Vector4D(x * v.x, y * v.y, z * v.z, w * v.w));
        }
        
        bool operator ==(const Vector4D& v) const
        {
            return ((x == v.x) && (y == v.y) && (z == v.z) && (w == v.w));
        }
        
        bool operator !=(const Vector4D& v) const
        {
            return ((x != v.x) || (y != v.y) || (z != v.z) || (w != v.w));
        }
        
        Vector4D& Normalize(void)
        {
            return (*this /= sqrtf(x * x + y * y + z * z + w * w));
        }
        
        Vector4D& RotateAboutX(float angle)
        {
            float s = sinf(angle);
            float c = cosf(angle);

            float ny = c * y - s * z;
            float nz = c * z + s * y;

            y = ny;
            z = nz;

            return (*this);
        }

        Vector4D& RotateAboutY(float angle)
        {
            float s = sinf(angle);
            float c = cosf(angle);

            float nx = c * x + s * z;
            float nz = c * z - s * x;

            x = nx;
            z = nz;

            return (*this);
        }

        Vector4D& RotateAboutZ(float angle)
        {
            float s = sinf(angle);
            float c = cosf(angle);

            float nx = c * x - s * y;
            float ny = c * y + s * x;

            x = nx;
            y = ny;

            return (*this);
        }

        Vector4D& RotateAboutAxis(float angle, const Vector3D& axis)
        {
            float s = sinf(angle);
            float c = cosf(angle);
            float k = 1.0F - c;

            float nx = x * (c + k * axis.x * axis.x) + y * (k * axis.x * axis.y - s * axis.z)
                    + z * (k * axis.x * axis.z + s * axis.y);
            float ny = x * (k * axis.x * axis.y + s * axis.z) + y * (c + k * axis.y * axis.y)
                    + z * (k * axis.y * axis.z - s * axis.x);
            float nz = x * (k * axis.x * axis.z - s * axis.y) + y * (k * axis.y * axis.z + s * axis.x)
                    + z * (c + k * axis.z * axis.z);

            x = nx;
            y = ny;
            z = nz;

            return (*this);
        }
};


inline Vector4D operator +(const Vector3D& v1, const Vector4D& v2)
{
    return (Vector4D(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v2.w));
}

inline Vector4D operator +(const Vector2D& v1, const Vector4D& v2)
{
    return (Vector4D(v1.x + v2.x, v1.y + v2.y,  v2.z, v2.w));
}

inline Vector4D operator -(const Vector3D& v1, const Vector4D& v2)
{
    return (Vector4D(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, -v2.w));
}

inline Vector4D operator -(const Vector2D& v1, const Vector4D& v2)
{
    return (Vector4D(v1.x - v2.x, v1.y - v2.y, -v2.z, -v2.w));
}

inline Vector4D operator *(float t, const Vector4D& v)
{
    return (Vector4D(t * v.x, t * v.y, t * v.z, t * v.w));
}

inline float operator *(const Vector3D& v1, const Vector4D& v2)
{
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

inline float operator *(const Point3D& v1, const Vector4D& v2)
{
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v2.w);
}

inline float operator *(const Vector2D& v1, const Vector4D& v2)
{
    return (v1.x * v2.x + v1.y * v2.y);
}

inline float operator *(const Point2D& v1, const Vector4D& v2)
{
    return (v1.x * v2.x + v1.y * v2.y + v2.w);
}

inline float vectordot(const Vector4D& v1, const Vector4D& v2)
{
    return (v1 * v2);
}

inline float vectorsize(const Vector4D& v)
{
    return (sqrtf(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w));
}

#endif

