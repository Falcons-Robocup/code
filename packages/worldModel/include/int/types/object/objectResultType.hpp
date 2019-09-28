 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectResultType.hpp
 *
 *  Created on: Jan 15, 2017
 *      Author: Jan Feitsma
 */

#ifndef OBJECTRESULTTYPE_HPP_
#define OBJECTRESULTTYPE_HPP_

#include <stddef.h>

class objectResultType
{
  public:
    objectResultType();
    ~objectResultType();

    void setId(const size_t id);
    void setTimestamp(const double stamp);
    void setCoordinates(const float x, const float y, const float z);
    void setVelocities(const float vx, const float vy, const float vz);

    double getTimestamp() const;
    size_t getId() const;
    float getX() const;
    float getY() const;
    float getZ() const;
    float getVX() const;
    float getVY() const;
    float getVZ() const;

  private:
    double _timestamp;
    size_t   _id;
    float _x;
    float _y;
    float _z;
    float _vx;
    float _vy;
    float _vz;
};

#endif /* OBJECTRESULTTYPE_HPP_ */

