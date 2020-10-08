 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPacketRefboxLogger.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#include "int/logger/cPacketRefboxLogger.hpp"

#include <stdexcept>

using std::exception;
using std::runtime_error;
using std::string;

using namespace packetRefboxLogger;

cPacketRefboxLogger::cPacketRefboxLogger()
{
    _jsonObject = NULL;
    _mPacket.type = string("worldstate");
    _mPacket.teamName = string("Falcons");
    _mPacket.globalIntention = string();
    _mPacket.robots.clear();
    _mPacket.balls.clear();
    _mPacket.obstacles.clear();
}

cPacketRefboxLogger::~cPacketRefboxLogger()
{
    cleanupJSONObject();
}

size_t cPacketRefboxLogger::getSize()
{
    string packet;
    generateJSON();

    packet = string(json_object_to_json_string(_jsonObject));
    return packet.size();
}

void cPacketRefboxLogger::getSerialized(std::string &packet)
{
    generateJSON();

    packet = string(json_object_to_json_string(_jsonObject));
    packet.push_back(0);
}

/*
 * Team level setters
 */
void cPacketRefboxLogger::setType(const string type)
{
    _mPacket.type = type;
}

void cPacketRefboxLogger::setTeamIntention(const string intention)
{
    _mPacket.globalIntention = intention;
}

/*
 * Robot level setters
 */
void cPacketRefboxLogger::setRobotPose(const uint8_t robotId, const pose &p)
{
    size_t index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        _mPacket.robots.at(index).position = p;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::setRobotTargetPose(const uint8_t robotId, const pose &targetPose)
{
    size_t index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        _mPacket.robots.at(index).targetPose = targetPose;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::setRobotVelocity(const uint8_t robotId, const pose &velocity)
{
    size_t index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        _mPacket.robots.at(index).velocity = velocity;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::setRobotIntention(const uint8_t robotId, const string intention)
{
    size_t index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        _mPacket.robots.at(index).intention = intention;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::setRobotBatteryLevel(const uint8_t robotId, const float level)
{
    size_t index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        _mPacket.robots.at(index).batteryLevel = level;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::setRobotBallPossession(const uint8_t robotId, const bool hasBall)
{
    size_t index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        _mPacket.robots.at(index).hasBall = hasBall;
    }
    catch (exception &e)
    {
        throw e;
    }
}

/*
 * Ball setters
 */
void cPacketRefboxLogger::addBall(const vec3d &position, const vec3d &velocity, const float confidence)
{
    ballStructure ball;

    ball.position = position;
    ball.velocity = velocity;
    ball.confidence = confidence;

    _mPacket.balls.push_back(ball);
}

/*
 * Obstacle setters
 */
void cPacketRefboxLogger::addObstacle(const vec2d &position, const vec2d &velocity, const float confidence)
{
    obstacleStructure obstacle;

    obstacle.position = position;
    obstacle.velocity = velocity;
    obstacle.confidence = confidence;

    _mPacket.obstacles.push_back(obstacle);
}

/*
 * Global setters
 */
void cPacketRefboxLogger::setAgeMilliseconds(const size_t age)
{
    _mPacket.ageMs = age;
}

/*
 * Robot functions
 */
void cPacketRefboxLogger::isRobotPresent(const uint8_t robotId, bool &isPresent, size_t &index)
{
    robotList::iterator robotIter = _mPacket.robots.begin();
    isPresent = false;
    index = 0;

    try
    {
        for(robotIter = _mPacket.robots.begin(); (robotIter < _mPacket.robots.end()) && (!isPresent); robotIter++)
        {
            if(robotIter->robotId == robotId)
            {
                isPresent = true;
                index = robotIter - _mPacket.robots.begin();
            }
        }
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::addRobot(const uint8_t robotId)
{
    robotStructure robot;

    try
    {
        robot.robotId = robotId;
        pose p;
        p.x = 0.0;
        p.y = 0.0;
        p.Rz = 0.0;
        robot.position = p;
        robot.velocity = p;
        robot.targetPose = p;
        robot.intention = string();
        robot.batteryLevel = 0.0;
        robot.hasBall = false;

        _mPacket.robots.push_back(robot);
    }
    catch (exception &e)
    {
        throw e;
    }
}

/*
 * JSON functions
 */
void cPacketRefboxLogger::cleanupJSONObject()
{
    try
    {
        if (_jsonObject != NULL)
        {
            json_object_put(_jsonObject);
            _jsonObject = NULL;
        }
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::generateJSON()
{
/*
 * Format of file
 * (basesestation "push" frequency: minimum rate of 10Hz)
 * (floats should have 3 digits after the comma)
 *
 *    {
 *      "type": "worldstate",
 *      "teamName": "CBD",
 *      "intention": "Searching for the ball",
 *      "robots": [
 *        {
 *          "id": 1,
 *          "pose": [ 0.002, -8.933, 0.4932],
 *          "targetPose": [0.002, -2.993, 0.4934],
 *          "velocity": [ 0, -0.004, 0.00 ],
 *          "intention": "",
 *          "batteryLevel": 100.0,
 *          "ballEngaged": 1
 *        },
 *        (...)
 *      ],
 *      "balls": [
 *        {
 *          "position": [ -0.002, -0.001, 0],
 *          "velocity": [ 0, 0, 0 ],
 *          "confidence": 0.959
 *        },
 *        (...)
 *      ],
 *      "obstacles": [
 *        {
 *          "position": [ -0.002, -0.001],
 *          "radius" : 0.25,
 *          "velocity": [ 0, 0],
 *          "confidence": 0.959
 *        },
 *        (...)
 *      ],
 *      "ageMs": 20,                     (refbox uses age to calculate and LOG the timestamp)
 *
 *    }
 *
 */

    try
    {
        cleanupJSONObject();
        _jsonObject = json_object_new_object();

        /* Add type string */
        json_object *type = json_object_new_string(_mPacket.type.data());
        json_object_object_add(_jsonObject, "type", type);

        /* Add team name string */
        json_object *teamName = json_object_new_string(_mPacket.teamName.data());
        json_object_object_add(_jsonObject, "teamName", teamName);

        /* Add team intention */
        json_object *teamIntention = json_object_new_string(_mPacket.globalIntention.data());
        json_object_object_add(_jsonObject, "intention", teamIntention);

        /* Adding robots */
        json_object *robots = json_object_new_array();
        addRobotsJSON(robots);
        json_object_object_add(_jsonObject, "robots", robots);

        /* Adding balls */
        json_object *balls = json_object_new_array();
        addBallsJSON(balls);
        json_object_object_add(_jsonObject, "balls", balls);

        /* Adding obstacles */
        json_object *obstacles = json_object_new_array();
        addObstaclesJSON(obstacles);
        json_object_object_add(_jsonObject, "obstacles", obstacles);

        /* Add ageMs */
        json_object *ageMs = json_object_new_int(20);
        json_object_object_add(_jsonObject, "ageMs", ageMs);
    }
    catch(exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::addRobotsJSON(json_object *obj)
{
    robotList::iterator robotIter = _mPacket.robots.begin();

    try
    {
        for(robotIter = _mPacket.robots.begin(); robotIter < _mPacket.robots.end(); robotIter++)
        {
            json_object *robot = json_object_new_object();

            /* id */
            json_object *robotId = json_object_new_int(robotIter->robotId);
            json_object_object_add(robot, "id", robotId);

            /* pose */
            json_object *p = json_object_new_array();
            json_object *poseX = json_object_new_double(robotIter->position.x);
            json_object *poseY = json_object_new_double(robotIter->position.y);
            json_object *posePhi = json_object_new_double(robotIter->position.Rz);
            json_object_array_add(p,poseX);
            json_object_array_add(p,poseY);
            json_object_array_add(p,posePhi);
            json_object_object_add(robot, "pose", p);

            /* targetPose */
            json_object *targetPose = json_object_new_array();
            json_object *targetPoseX = json_object_new_double(robotIter->targetPose.x);
            json_object *targetPoseY = json_object_new_double(robotIter->targetPose.y);
            json_object *targetPosePhi = json_object_new_double(robotIter->targetPose.Rz);
            json_object_array_add(targetPose,targetPoseX);
            json_object_array_add(targetPose,targetPoseY);
            json_object_array_add(targetPose,targetPosePhi);
            json_object_object_add(robot, "targetPose", targetPose);

            /* velocity */
            json_object *velocity = json_object_new_array();
            json_object *velocityX = json_object_new_double(robotIter->velocity.x);
            json_object *velocityY = json_object_new_double(robotIter->velocity.y);
            json_object *velocityPhi = json_object_new_double(robotIter->velocity.Rz);
            json_object_array_add(velocity,velocityX);
            json_object_array_add(velocity,velocityY);
            json_object_array_add(velocity,velocityPhi);
            json_object_object_add(robot, "velocity", velocity);

            /* robot Intention */
            json_object *robotIntention = json_object_new_string(robotIter->intention.data());
            json_object_object_add(robot, "intention", robotIntention);

            /* Battery level */
            json_object *batteryLevel = json_object_new_double(robotIter->batteryLevel);
            json_object_object_add(robot, "batteryLevel", batteryLevel);

            /* ball Engaged */
            json_object *ballEngaged = json_object_new_int(robotIter->hasBall);
            json_object_object_add(robot, "ballEngaged", ballEngaged);

            /* Lastly add robot to array */
            json_object_array_add(obj, robot);
        }
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::addBallsJSON(json_object *obj)
{
    ballList::iterator ballIter = _mPacket.balls.begin();

    try
    {
        for(ballIter = _mPacket.balls.begin(); ballIter < _mPacket.balls.end(); ballIter++)
        {
            json_object *ball = json_object_new_object();

            /* position */
            json_object *position = json_object_new_array();
            json_object *positionX = json_object_new_double(ballIter->position.x);
            json_object *positionY = json_object_new_double(ballIter->position.y);
            json_object *positionZ = json_object_new_double(ballIter->position.z);
            json_object_array_add(position,positionX);
            json_object_array_add(position,positionY);
            json_object_array_add(position,positionZ);
            json_object_object_add(ball, "position", position);

            /* velocity */
            json_object *velocity = json_object_new_array();
            json_object *velocityX = json_object_new_double(ballIter->velocity.x);
            json_object *velocityY = json_object_new_double(ballIter->velocity.y);
            json_object *velocityZ = json_object_new_double(ballIter->velocity.z);
            json_object_array_add(velocity,velocityX);
            json_object_array_add(velocity,velocityY);
            json_object_array_add(velocity,velocityZ);
            json_object_object_add(ball, "velocity", velocity);

            /* confidence */
            json_object *confidence = json_object_new_double(ballIter->confidence);
            json_object_object_add(ball, "confidence", confidence);

            /* Lastly add ball to array */
            json_object_array_add(obj, ball);
        }
    }
    catch (exception &e)
    {
        throw e;
    }
}

void cPacketRefboxLogger::addObstaclesJSON(json_object *obj)
{
    obstacleList::iterator obstacleIter = _mPacket.obstacles.begin();

    try
    {
        for(obstacleIter = _mPacket.obstacles.begin(); obstacleIter < _mPacket.obstacles.end(); obstacleIter++)
        {
            json_object *obstacle = json_object_new_object();

            /* position */
            json_object *position = json_object_new_array();
            json_object *positionX = json_object_new_double(obstacleIter->position.x);
            json_object *positionY = json_object_new_double(obstacleIter->position.y);
            json_object_array_add(position,positionX);
            json_object_array_add(position,positionY);
            json_object_object_add(obstacle, "position", position);

            /* radius */
            json_object *radius = json_object_new_double(obstacleIter->radius);
            json_object_object_add(obstacle, "confidence", radius);

            /* velocity */
            json_object *velocity = json_object_new_array();
            json_object *velocityX = json_object_new_double(obstacleIter->velocity.x);
            json_object *velocityY = json_object_new_double(obstacleIter->velocity.y);
            json_object_array_add(velocity,velocityX);
            json_object_array_add(velocity,velocityY);
            json_object_object_add(obstacle, "velocity", velocity);

            /* confidence */
            json_object *confidence = json_object_new_double(obstacleIter->confidence);
            json_object_object_add(obstacle, "confidence", confidence);

            /* Lastly add obstacle to array */
            json_object_array_add(obj, obstacle);
        }
    }
    catch (exception &e)
    {
        throw e;
    }
}
