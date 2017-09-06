 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * packetStructureRefboxLogger.hpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#ifndef PACKETSTRUCTUREREFBOXLOGGER_HPP_
#define PACKETSTRUCTUREREFBOXLOGGER_HPP_

#include <cstdio>
#include <stdint.h>
#include <vector>

#include "position2d.hpp"
#include "vector2d.hpp"
#include "vector3d.hpp"

namespace packetRefboxLogger
{
    typedef struct
    {
    	uint8_t     robotId;
		Position2D  pose;
		Position2D  velocity;
		Position2D  targetPose;
		std::string intention;
		float       batteryLevel;
		bool        hasBall;
    } robotStructure;
    typedef std::vector<robotStructure> robotList;

    typedef struct
    {
    	Vector3D position;
    	Vector3D velocity;
        float    confidence;
    } ballStructure;
    typedef std::vector<ballStructure> ballList;

    typedef struct
    {
    	Vector2D position;
    	Vector2D velocity;
    	float    radius;
    	float    confidence;
    } obstacleStructure;
    typedef std::vector<obstacleStructure> obstacleList;

    typedef struct
    {
    	std::string type;
    	std::string teamName;
    	std::string globalIntention;
    	robotList robots;
    	ballList balls;
    	obstacleList obstacles;
    	size_t ageMs;
    } packetStructureDeserialized;
}

#endif /* PACKETSTRUCTUREREFBOXLOGGER_HPP_ */
