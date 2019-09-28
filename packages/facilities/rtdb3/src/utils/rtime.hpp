 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * rtime.hpp
 *
 * RtDB2 timestamp utilities, conversions, standards.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#ifndef RTDB2_RTIME_HPP_
#define RTDB2_RTIME_HPP_

#include <string>
#include <sys/time.h>
#include <boost/function.hpp>
#include "../rtdb2/RtDB2Definitions.h" // for serialization

class rtime
{
public:
    rtime(double v = 0);
    ~rtime();
    
    // get current timestamp
    static rtime now();
    
    // run a timed loop, calling function on given frequency until it returns false
    static void loop(float frequency, boost::function<bool(void)> callback);
    
    // converters
    rtime &fromTimeval(const struct timeval tv);
    struct timeval toTimeval() const;
    std::string toStrDate() const; // handy for file names, example: 20180812_200159
    std::string toStr() const; // handy for tracing, example: 2018-08-12,20:01:59.442195
    rtime &fromDouble(double d);
    double toDouble() const;
    operator double() const { return toDouble(); }
    
    // operators
    rtime &operator+=(const rtime &other);
    rtime &operator-=(const rtime &other);
    bool operator==(const rtime &other);

    // members need to be public for serialization
    unsigned int _tvSec = 0;
    unsigned int _tvUsec = 0;
    
    SERIALIZE_DATA_FIXED(_tvSec, _tvUsec);

private:
    operator float() const; // not implemented, to force compile time error, since implicitly throwing away least-significant bits is asking for trouble
    
};

inline bool operator<(rtime lhs, const rtime& rhs)
{
    return (double)lhs < (double)rhs;
}

inline bool operator>(rtime lhs, const rtime& rhs)
{
    return (double)lhs > (double)rhs;
}

inline bool operator>=(rtime lhs, const rtime& rhs)
{
    return (double)lhs >= (double)rhs;
}

inline rtime operator+(rtime lhs, const rtime& rhs)
{
    lhs += rhs;
    return lhs;
}

inline rtime operator-(rtime lhs, const rtime& rhs)
{
    lhs -= rhs;
    return lhs;
}

inline rtime operator+(rtime lhs, float rhs)
{
    lhs += rtime(rhs);
    return lhs;
}

inline rtime operator+(rtime lhs, double rhs)
{
    lhs += rtime(rhs);
    return lhs;
}

inline rtime operator-(rtime lhs, double rhs)
{
    lhs -= rtime(rhs);
    return lhs;
}

inline rtime operator-(rtime lhs, float rhs)
{
    lhs -= rtime(rhs);
    return lhs;
}

inline rtime operator*(rtime lhs, double rhs)
{
    return rtime().fromDouble(lhs.toDouble() * rhs);
}

#endif

