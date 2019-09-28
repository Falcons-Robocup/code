 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPacketRefboxLogger.hpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#ifndef CPACKETREFBOXLOGGER_HPP_
#define CPACKETREFBOXLOGGER_HPP_

#include <stddef.h>
#include <string>
#include <json-c/json.h>

// sharedTypes
#include "pose.hpp"
#include "vec2d.hpp"
#include "vec3d.hpp"

#include "int/types/packetStructureRefboxLogger.hpp"
#include "int/logger/cPacketAbstract.hpp"

namespace packetRefboxLogger
{
    class cPacketRefboxLogger : public cPacketAbstract
    {
        public:
            cPacketRefboxLogger();
            ~cPacketRefboxLogger();

            virtual size_t getSize();

            virtual void getSerialized(std::string &packet);

            // Team level setters
            void setType(const std::string type);
            void setTeamIntention(const std::string intention);

            // Robot level setters
            void setRobotPose(const uint8_t robotId, const pose &p);
            void setRobotTargetPose(const uint8_t robotId, const pose &p);
            void setRobotVelocity(const uint8_t robotId, const pose &v);
            void setRobotIntention(const uint8_t robotId, const std::string intention);
            void setRobotBatteryLevel(const uint8_t robotId, const float level);
            void setRobotBallPossession(const uint8_t robotId, const bool hasBall);

            // Ball setters
            void addBall(const vec3d &position, const vec3d &velocity, const float confidence);

            // Obstacle setters
            void addObstacle(const vec2d &position, const vec2d &velocity, const float confidence);

            // Global setters
            void setAgeMilliseconds(const size_t age);

        private:
            packetStructureDeserialized _mPacket;
            json_object *_jsonObject;

            /* Robot functions */
            void isRobotPresent(const uint8_t robotId, bool &isPresent, size_t &index);
            void addRobot(const uint8_t);

            /* JSON functions */
            void cleanupJSONObject();
            void generateJSON();
            void addRobotsJSON(json_object *obj);
            void addBallsJSON(json_object *obj);
            void addObstaclesJSON(json_object *obj);
};
}

#endif /* CPACKETREFBOXLOGGER_HPP_ */
