// Copyright 2016-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
