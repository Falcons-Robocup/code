 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotType.hpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTTYPE_HPP_
#define ROBOTTYPE_HPP_

#include <stdint.h>

#include "int/types/coordinateType.hpp"

class robotClass_t
{
    public:
        robotClass_t();
        ~robotClass_t();

        static bool sortOnIncreasingRobotID(const robotClass_t& objA, const robotClass_t& objB)
        {
            return objA.getRobotID() < objB.getRobotID();
        }

        void setRobotID(const uint8_t robotID);
        void setTimestamp(const double stamp);
        void setCoordinateType(const coordinateType coordinate);
        void setCoordinates(const float x, const float y, const float theta);
        void setVelocities(const float vx, const float vy, const float vtheta);
        void setBallPossession(bool bp);

        uint8_t getRobotID() const;
        double getTimestamp() const;
        coordinateType getCoordindateType() const;
        float getX() const;
        float getY() const;
        float getTheta() const;
        float getVX() const;
        float getVY() const;
        float getVTheta() const;
        bool getBallPossession() const;

    private:
        uint8_t _robotID;
        double _timestamp;
        coordinateType _coordinates;
        float _x;
        float _y;
        float _theta;
        float _vx;
        float _vy;
        float _vtheta;
        bool _ballPossession;
};

#endif /* ROBOTTYPE_HPP_ */
